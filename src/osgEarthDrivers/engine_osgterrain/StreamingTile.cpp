/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include "StreamingTile"
#include "StreamingTerrainNode"
#include "CustomTerrainTechnique"
#include "TransparentLayer"

#include <osgEarth/Registry>
#include <osgEarth/Locators>
#include <osgEarth/MapFrame>
#include <osgEarth/NodeUtils>

#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/Node>
#include <osg/Texture2D>
#include <osgGA/EventVisitor>

#include <OpenThreads/ScopedLock>

using namespace osgEarth_engine_osgterrain;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[StreamingTile] "

//----------------------------------------------------------------------------

namespace
{
    // this progress callback checks to see whether the request being serviced is 
    // out of date with respect to the task service that is running it. It checks
    // for a disparity in frame stamps, and reports that the request should be
    // canceled if it appears the request has been abandoned by the Tile that
    // originally scheduled it.
    struct StampedProgressCallback : ProgressCallback
    {
    public:
        StampedProgressCallback(TaskRequest* request, TaskService* service):
          _request(request),
          _service(service)
        {
        }

        //todo: maybe we should pass TaskRequest in as an argument 
        bool reportProgress(double current, double total)
        {
            //Check to see if we were marked cancelled on a previous check
            if (_canceled) return _canceled;

            _canceled = (_service->getStamp() - _request->getStamp() > 2);
            return _canceled;
        }

        TaskRequest* _request;
        TaskService* _service;
    };


    // NOTE: Task requests run in background threads. So we pass in a map frame and
    // make a clone of it to use in that thread. Each Task must have its own MapFrame
    // so it's operating in its own sandbox.

    struct TileLayerRequest : public TaskRequest
    {
        TileLayerRequest( const TileKey& key, const MapFrame& mapf, OSGTileFactory* tileFactory )
            : _key( key ), 
              _mapf(mapf, "osgterrain.TileLayerRequest"), 
              _tileFactory(tileFactory), 
              _numTries(0), 
              _maxTries(3) { }

        TileKey _key;
        MapFrame _mapf;
        //osg::ref_ptr<Map> _map;
        osg::ref_ptr<OSGTileFactory> _tileFactory;
        unsigned int _numTries;
        unsigned int _maxTries;
    };

    struct TileColorLayerRequest : public TileLayerRequest
    {
        TileColorLayerRequest( const TileKey& key, const MapFrame& mapf, OSGTileFactory* tileFactory, UID layerUID )
            : TileLayerRequest( key, mapf, tileFactory ), _layerUID(layerUID) { }

        void operator()( ProgressCallback* progress )
        {
            osg::ref_ptr<ImageLayer> imageLayer = _mapf.getImageLayerByUID( _layerUID );
            if ( imageLayer.valid() )
            {
                _result = _tileFactory->createImageLayer( _mapf.getMapInfo(), imageLayer.get(), _key, progress );
                if (!wasCanceled())
                {
                  _numTries++;
                }
            }
        }
        UID _layerUID;
    };

    struct TileElevationLayerRequest : public TileLayerRequest
    {
        TileElevationLayerRequest( const TileKey& key, const MapFrame& mapf, OSGTileFactory* tileFactory )
            : TileLayerRequest( key, mapf, tileFactory )
        {
            //nop
        }

        void operator()( ProgressCallback* progress )
        {
            _result = _tileFactory->createHeightFieldLayer( _mapf, _key, true ); //exactOnly=true
            _numTries++;
        }
    };

    struct TileElevationPlaceholderLayerRequest : public TileLayerRequest
    {
        TileElevationPlaceholderLayerRequest( const TileKey& key, const MapFrame& mapf, OSGTileFactory* tileFactory, GeoLocator* keyLocator )
            : TileLayerRequest( key, mapf, tileFactory ),
              _parentKey( key.createParentKey() ),
              _keyLocator(keyLocator)
        {
            //nop
        }

        void setParentHF( osg::HeightField* parentHF )
        {
            _parentHF = parentHF; 
        }

        void setNextLOD( int nextLOD )
        {
            _nextLOD = nextLOD;
        }

        void operator()( ProgressCallback* progress )
        {
            if ( !progress->isCanceled() )
            {
                _result = _tileFactory->createPlaceholderHeightfieldLayer(
                    _parentHF.get(),
                    _parentKey,
                    _key,
                    _keyLocator.get() );
            }
        }

        osg::ref_ptr<osg::HeightField> _parentHF;
        TileKey _parentKey;
        osg::ref_ptr<GeoLocator>    _keyLocator;
        int _nextLOD;
    };

    // A task request that rebuilds a tile's terrain technique in the background. It
    // re-compiles the geometry but does NOT apply the updates (since this constitutes
    // altering the scene graph and must therefore be done in the update traversal).
    struct TileGenRequest : public TaskRequest
    {
        TileGenRequest( StreamingTile* tile, const TileUpdate& update ) :
            _tile( tile ), _update(update) { }

        void operator()( ProgressCallback* progress )
        {
            if (_tile.valid())
            {
                CustomTerrainTechnique* tech = dynamic_cast<CustomTerrainTechnique*>( _tile->getTerrainTechnique() );
                if (tech)
                {
                    tech->compile( _update, progress );
                }
            }

            //We don't need the tile anymore
            _tile = NULL;
        }

        osg::ref_ptr< StreamingTile > _tile;
        TileUpdate _update;
    };
}

//------------------------------------------------------------------------

StreamingTile::StreamingTile(const TileKey& key,
                             GeoLocator*    keyLocator, 
                             bool           quickReleaseGLObjects ) :

Tile( key, keyLocator, quickReleaseGLObjects ),

_requestsInstalled     ( false ),
_elevationLayerDirty   ( false ),
_colorLayersDirty      ( false ),
_elevationLayerUpToDate( true ),
_elevationLOD          ( key.getLevelOfDetail() ),
_useTileGenRequest     ( true )
{
    // because the lowest LOD (1) is always loaded fully:
    _elevationLayerUpToDate = _key.getLevelOfDetail() <= 1;
}

StreamingTile::~StreamingTile()
{
    //nop
}

bool
StreamingTile::cancelActiveTasks()
{
    // This method ensures that all requests owned by this object are stopped and released
    // by the corresponding task service prior to destructing the tile. Called by
    // CustomTerrain::updateTileTable().

    bool done = true;

    // Cancel all active requests
    if ( _requestsInstalled )
    {
        for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); ++i )
        {
            i->get()->cancel();
        }

        if ( _elevRequest.valid() )
        {
            _elevRequest->cancel();
        }

        if (_elevPlaceholderRequest.valid())
        {
            _elevPlaceholderRequest->cancel();
        }

        if (_tileGenRequest.valid())
        {
            _tileGenRequest->cancel();
        }
    }

    return done;
}

void
StreamingTile::setElevationLOD( int lod )
{
    _elevationLOD = lod;
    _elevationLayerUpToDate = _elevationLOD == (int)_key.getLevelOfDetail();
}

StreamingTerrainNode*
StreamingTile::getStreamingTerrain()
{
    return static_cast<StreamingTerrainNode*>( getTerrain() );
}

const StreamingTerrainNode*
StreamingTile::getStreamingTerrain() const
{
    return const_cast<StreamingTile*>(this)->getStreamingTerrain();
}

void
StreamingTile::setHasElevationHint( bool hint ) 
{
    _hasElevation = hint;
}

// returns TRUE if it's safe for this tile to load its next elevation data layer.
bool
StreamingTile::readyForNewElevation()
{
    bool ready = true;

    if ( _elevationLOD == (int)_key.getLevelOfDetail() )
    {
        ready = false;
    }
    else if ( _family[Relative::PARENT].elevLOD < 0 )
    {
        ready = false;
    }
    else
    {
        for( int i=Relative::PARENT; i<=Relative::SOUTH; i++) 
        {
            if ( _family[i].expected && _family[i].elevLOD >= 0 && _family[i].elevLOD < _elevationLOD )
            {
                ready = false;
                break;
            }
        }

        // if the next LOD is not the final, but our placeholder is up to date, we're not ready.
        if ( ready && _elevationLOD+1 < (int)_key.getLevelOfDetail() && _elevationLOD == _family[Relative::PARENT].elevLOD )
        {
            ready = false;
        }
    }

#ifdef PREEMPTIVE_DEBUG
    OE_NOTICE
        << "Tile (" << _key.str() << ") at (" << _elevationLOD << "), parent at ("
        << _family[PARENT].elevLOD << "), sibs at (";
    if ( _family[WEST].expected ) osg::notify( osg::NOTICE ) << "W=" << _family[WEST].elevLOD << " ";
    if ( _family[NORTH].expected ) osg::notify( osg::NOTICE ) << "N=" << _family[NORTH].elevLOD << " ";
    if ( _family[EAST].expected ) osg::notify( osg::NOTICE ) << "E=" << _family[EAST].elevLOD << " ";
    if ( _family[SOUTH].expected ) osg::notify( osg::NOTICE ) << "S=" << _family[SOUTH].elevLOD << " ";
    OE_NOTICE << "), ready = " << (ready? "YES" : "no") << std::endl;
#endif

    return ready;
}



// returns TRUE if it's safe for this tile to load its next elevation data layer.
bool
StreamingTile::readyForNewImagery(ImageLayer* layer, int currentLOD)
{
    bool ready = true;

    if ( currentLOD == (int)_key.getLevelOfDetail() )
    {
        ready = false;
    }
    else if ( _family[Relative::PARENT].getImageLOD( layer->getUID() ) < 0 )
    {
        ready = false;
    }
    else
    {
        for( int i=Relative::PARENT; i<=Relative::SOUTH; i++) 
        {
            if (_family[i].expected && 
                _family[i].getImageLOD( layer->getUID() ) >= 0 && 
                _family[i].getImageLOD( layer->getUID() ) < currentLOD )
            {
                ready = false;
                break;
            }
        }

        // if the next LOD is not the final, but our placeholder is up to date, we're not ready.
        if (ready &&
            currentLOD + 1 < (int)_key.getLevelOfDetail() && 
            currentLOD == _family[Relative::PARENT].getImageLOD( layer->getUID() ) )
        {
            ready = false;
        }
    }

    return ready;
}


#define PRI_IMAGE_OFFSET 0.1f // priority offset of imagery relative to elevation
#define PRI_LAYER_OFFSET 0.1f // priority offset of image layer(x) vs. image layer(x+1)

void
StreamingTile::installRequests( const MapFrame& mapf, int stamp )
{
    StreamingTerrainNode* terrain     = getStreamingTerrain();
    OSGTileFactory*   tileFactory = terrain->getTileFactory();

    bool hasElevationLayer;
    {
        Threading::ScopedReadLock sharedLock( _tileLayersMutex );
        hasElevationLayer = this->getElevationLayer() != NULL;
    }

    if ( hasElevationLayer )
    {
        resetElevationRequests( mapf );     
    }

    // safely loop through the map layers and schedule imagery updates for each:
    for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); ++i )
    {
        updateImagery( i->get(), mapf, tileFactory );
    }

    _requestsInstalled = true;
}

void
StreamingTile::resetElevationRequests( const MapFrame& mapf )
{
    if (_elevRequest.valid() && _elevRequest->isRunning()) _elevRequest->cancel();
    if (_elevPlaceholderRequest.valid() && _elevPlaceholderRequest->isRunning()) _elevPlaceholderRequest->cancel();

    StreamingTerrainNode* terrain = getStreamingTerrain();

    // this request will load real elevation data for the tile:
    _elevRequest = new TileElevationLayerRequest(_key, mapf, terrain->getTileFactory());
    float priority = (float)_key.getLevelOfDetail();
    _elevRequest->setPriority( priority );
    std::stringstream ss;
    ss << "TileElevationLayerRequest " << _key.str() << std::endl;
    std::string ssStr;
    ssStr = ss.str();
    _elevRequest->setName( ssStr );

    // this request will load placeholder elevation data for the tile:
    _elevPlaceholderRequest = new TileElevationPlaceholderLayerRequest(
        _key, mapf, terrain->getTileFactory(), _locator.get() );

    _elevPlaceholderRequest->setPriority( priority );
    ss.str("");
    ss << "TileElevationPlaceholderLayerRequest " << _key.str() << std::endl;
    ssStr = ss.str();
    _elevPlaceholderRequest->setName( ssStr );
}


// called from installRequests (cull traversal) or terrainengine (main thread) ... so be careful!
//
// this method queues up a new tile imagery request, superceding any existing request that
// might be in the queue.
void
StreamingTile::updateImagery( ImageLayer* imageLayer, const MapFrame& mapf, OSGTileFactory* tileFactory)
{
    StreamingTerrainNode* terrain = getStreamingTerrain();

    // imagery is slighty higher priority than elevation data
    TaskRequest* r = new TileColorLayerRequest( _key, mapf, tileFactory, imageLayer->getUID() );
    std::stringstream ss;
    ss << "TileColorLayerRequest " << _key.str() << std::endl;
    std::string ssStr;
    ssStr = ss.str();
    r->setName( ssStr );
    r->setState( osgEarth::TaskRequest::STATE_IDLE );

    // in image-sequential mode, we want to prioritize lower-LOD imagery since it
    // needs to come in before higher-resolution stuff. 
    if ( terrain->getLoadingPolicy().mode() == LoadingPolicy::MODE_SEQUENTIAL )
    {
        r->setPriority( -(float)_key.getLevelOfDetail() + PRI_IMAGE_OFFSET );
    }

    // in image-preemptive mode, the highest LOD should get higher load priority:
    else // MODE_PREEMPTIVE
    {
        r->setPriority( PRI_IMAGE_OFFSET + (float)_key.getLevelOfDetail());
    }

    r->setProgressCallback( new StampedProgressCallback( 
        r,
        terrain->getImageryTaskService( imageLayer->getUID() ) ) );

    //If we already have a request for this layer, remove it from the list and use the new one
    for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); )
    {
        TileColorLayerRequest* r2 = static_cast<TileColorLayerRequest*>( i->get() );
        if ( r2->_layerUID == imageLayer->getUID() )
            i = _requests.erase( i );
        else
            ++i;
    }

    //Add the new imagery request
    _requests.push_back( r );
}

// This method is called during the UPDATE TRAVERSAL in StreamingTerrain.
void
StreamingTile::servicePendingImageRequests( const MapFrame& mapf, int stamp )
{       
    // Don't do anything until we have been added to the scene graph
    if ( !_hasBeenTraversed ) return;

    // install our requests if they are not already installed:
    if ( !_requestsInstalled )
    {
        // since we're in the CULL thread, use the cull thread map frame:
        installRequests( mapf, stamp );
    }

    for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); ++i )
    {
        TileColorLayerRequest* r = static_cast<TileColorLayerRequest*>( i->get() );

        //If a request has been marked as IDLE, the TaskService has tried to service it
        //and it was either deemed out of date or was cancelled, so we need to add it again.
        if ( r->isIdle() )
        {
            //OE_NOTICE << "Queuing IR (" << _key.str() << ")" << std::endl;
            r->setStamp( stamp );
            getStreamingTerrain()->getImageryTaskService( r->_layerUID )->add( r );
        }
        else if ( !r->isCompleted() )
        {
            r->setStamp( stamp );
        }
    }    
}

// This method is called from the UPDATE TRAVERSAL, from CustomTerrain::traverse.
void
StreamingTile::servicePendingElevationRequests( const MapFrame& mapf, int stamp, bool tileTableLocked )
{
    //Don't do anything until we have been added to the scene graph
    if ( !_hasBeenTraversed ) return;

    // install our requests if they are not already installed:
    if ( !_requestsInstalled )
    {
        installRequests( mapf, stamp );
    }

    if ( _hasElevation && !_elevationLayerUpToDate && _elevRequest.valid() && _elevPlaceholderRequest.valid() )
    {  
        StreamingTerrainNode* terrain = getStreamingTerrain();

        // update the main elevation request if it's running:
        if ( !_elevRequest->isIdle() )
        {
#ifdef PREEMPTIVE_DEBUG
            OE_NOTICE << "Tile (" << _key.str() << ") .. ER not idle" << std::endl;
#endif
            
            if ( !_elevRequest->isCompleted() )
            {
                _elevRequest->setStamp( stamp );
            }
        }

        // update the placeholder request if it's running:
        else if ( !_elevPlaceholderRequest->isIdle() )
        {
#ifdef PREEMPTIVE_DEBUG
            OE_NOTICE << "Tile (" << _key.str() << ") .. PR not idle" << std::endl;
#endif
            if ( !_elevPlaceholderRequest->isCompleted() )
            {
               _elevPlaceholderRequest->setStamp( stamp );
            }
        }

        // otherwise, see if it is legal yet to start a new request:
        else if ( readyForNewElevation() )
        {
            if ( _elevationLOD + 1 == _key.getLevelOfDetail() )
            {
                _elevRequest->setStamp( stamp );
                _elevRequest->setProgressCallback( new ProgressCallback() );
                terrain->getElevationTaskService()->add( _elevRequest.get() );
#ifdef PREEMPTIVE_DEBUG
                OE_NOTICE << "..queued FE req for (" << _key.str() << ")" << std::endl;
#endif
            }
            
            else if ( _family[Relative::PARENT].elevLOD > _elevationLOD )
            {
                osg::ref_ptr<StreamingTile> parentTile;
                terrain->getTile( _family[Relative::PARENT].tileID, parentTile, !tileTableLocked );

                if ( _elevationLOD < _family[Relative::PARENT].elevLOD && parentTile.valid() )
                {
                    TileElevationPlaceholderLayerRequest* er = static_cast<TileElevationPlaceholderLayerRequest*>(_elevPlaceholderRequest.get());

                    er->setStamp( stamp );
                    er->setProgressCallback( new ProgressCallback() );
                    float priority = (float)_key.getLevelOfDetail();
                    er->setPriority( priority );
                    //TODO: should there be a read lock here when accessing the parent tile's elevation layer? GW
                    osgTerrain::HeightFieldLayer* hfLayer = static_cast<osgTerrain::HeightFieldLayer*>(parentTile->getElevationLayer());
                    er->setParentHF( hfLayer->getHeightField() );
                    er->setNextLOD( _family[Relative::PARENT].elevLOD );
                    terrain->getElevationTaskService()->add( er );
#ifdef PREEMPTIVE_DEBUG
                    OE_NOTICE << "..queued PH req for (" << _key.str() << ")" << std::endl;
#endif
                }

                else 
                {
#ifdef PREEMPTIVE_DEBUG
                    OE_NOTICE << "...tile (" << _key.str() << ") ready, but nothing to do." << std::endl;
#endif
                }
            }
        }
    }
}

void
StreamingTile::queueTileUpdate( TileUpdate::Action action, int value )
{
    if ( _useTileGenRequest )
    {
        _tileUpdates.push( TileUpdate(action, value) );
    }
    else
    {
        Tile::queueTileUpdate( action, value );
    }
}

// called from the UPDATE TRAVERSAL, because this method can potentially alter
// the scene graph.
bool
StreamingTile::serviceCompletedRequests( const MapFrame& mapf, bool tileTableLocked )
{
    //Don't do anything until we have been added to the scene graph
    if (!_hasBeenTraversed) return false;

    bool tileModified = false;

    if ( !_requestsInstalled )
        return false;

    // First service the tile generator:
    if ( _tileGenRequest.valid() && _tileGenRequest->isCompleted() )
    {
        CustomTerrainTechnique* tech = dynamic_cast<CustomTerrainTechnique*>( getTerrainTechnique() );
        if ( tech )
        {
            //TODO: consider waiting to apply if there are still more tile updates in the queue.
            if ( _tileUpdates.size() == 0 )
            {
                tileModified = tech->applyTileUpdates();
            }
        }
        _tileGenRequest = 0L;
    }


    // now deal with imagery.
    const LoadingPolicy& lp = getStreamingTerrain()->getLoadingPolicy();

    StreamingTerrainNode* terrain = getStreamingTerrain();

    //Check each layer independently.
    for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); ++i )
    {
        ImageLayer* imageLayer = i->get();

        bool checkForFinalImagery = false;

        CustomColorLayer colorLayer;
        if ( getCustomColorLayer( imageLayer->getUID(), colorLayer ) )
        {
            if ( lp.mode() == LoadingPolicy::MODE_PREEMPTIVE )
            {
                // in preemptive mode, always check for the final imagery - there are no intermediate
                // placeholders.
                checkForFinalImagery = true;
            }
            else if (lp.mode() == LoadingPolicy::MODE_SEQUENTIAL && 
                     readyForNewImagery(imageLayer, colorLayer.getLevelOfDetail()) )
            {
                // in sequential mode, we have to incrementally increase imagery resolution by
                // creating placeholders based of parent tiles, one LOD at a time.
                if ( colorLayer.getLevelOfDetail() + 1 < (int)_key.getLevelOfDetail() )
                {
                    // if the parent's image LOD is higher than ours, replace ours with the parent's
                    // since it is a higher-resolution placeholder:
                    if ( _family[Relative::PARENT].getImageLOD(colorLayer.getUID()) > colorLayer.getLevelOfDetail() )
                    {
                        osg::ref_ptr<Tile> parentTile;
                        getStreamingTerrain()->getTile( _family[Relative::PARENT].tileID, parentTile, !tileTableLocked );

                        // Set the color layer to the parent color layer as a placeholder.
                        CustomColorLayer parentColorLayer;
                        if ( parentTile->getCustomColorLayer( colorLayer.getUID(), parentColorLayer ) )
                        {
                            this->setCustomColorLayer( parentColorLayer );
                        }

                        // ... and queue up an update request.
                        queueTileUpdate( TileUpdate::UPDATE_IMAGE_LAYER, colorLayer.getUID() );
                    }
                }
                else
                {
                    // we've gone as far as we can with placeholders; time to check for the
                    // final imagery tile.
                    checkForFinalImagery = true;
                }
            }
        }

        if ( checkForFinalImagery )
        {
            // Then the image requests:
            for( TaskRequestList::iterator itr = _requests.begin(); itr != _requests.end(); )
            {
                bool increment = true;
                TileColorLayerRequest* r = static_cast<TileColorLayerRequest*>( itr->get() );
                //We only care about the current layer we are checking
                if ( r->_layerUID == imageLayer->getUID() )
                {
                    if ( itr->get()->isCompleted() )
                    {
                        if ( r->wasCanceled() )
                        {
                            //Reset the cancelled task to IDLE and give it a new progress callback.
                            r->setState( TaskRequest::STATE_IDLE );
                            r->setProgressCallback( new StampedProgressCallback(
                                r, terrain->getImageryTaskService( r->_layerUID )));
                            r->reset();
                        }
                        else // success..
                        {
                            //See if we even care about the request
                            if ( !mapf.getImageLayerByUID( r->_layerUID ) )
                            {
                                //The maplayer was probably deleted
                                OE_DEBUG << "Layer uid=" << r->_layerUID << " no longer exists, ignoring TileColorLayerRequest " << std::endl;
                                itr = _requests.erase(itr);
                                increment = false;
                            }
                            else
                            {
                                CustomColorLayerRef* result = static_cast<CustomColorLayerRef*>( r->getResult() );
                                if ( result )
                                {
                                    this->setCustomColorLayer( result->_layer );

                                    queueTileUpdate( TileUpdate::UPDATE_IMAGE_LAYER, r->_layerUID );

                                    //OE_NOTICE << "Complete IR (" << _key.str() << ") layer=" << r->_layerId << std::endl;

                                    // remove from the list (don't reference "r" after this!)
                                    itr = _requests.erase( itr );
                                    increment = false;
                                }
                                else
                                {  
                                    if (r->_numTries > r->_maxTries)
                                    {
                                        CustomColorLayer oldLayer;
                                        if ( this->getCustomColorLayer( r->_layerUID, oldLayer ) )
                                        {
                                            // apply the old color layer but with a new LOD.
                                            this->setCustomColorLayer( CustomColorLayer(
                                                oldLayer.getMapLayer(),
                                                oldLayer.getImage(),
                                                oldLayer.getLocator(),
                                                _key.getLevelOfDetail(),
                                                _key ));

                                            itr = _requests.erase( itr );
                                            increment = false;
                                            OE_DEBUG << "Tried (" << _key.str() << ") (layer uid=" << r->_layerUID << "), too many times, moving on...." << std::endl;
                                        }
                                    }
                                    else
                                    {
                                        OE_DEBUG << "IReq error (" << _key.str() << ") (layer uid=" << r->_layerUID << "), retrying" << std::endl;

                                        //The color layer request failed, probably due to a server error. Reset it.
                                        r->setState( TaskRequest::STATE_IDLE );
                                        r->reset();
                                    }
                                }
                            }
                        }
                    }
                }

                if ( increment )
                    ++itr;
            }
        }
    }

    // Finally, the elevation requests:
    if ( _hasElevation && !_elevationLayerUpToDate && _elevRequest.valid() && _elevPlaceholderRequest.valid() )
    {
        // First, check is the Main elevation request is done. If so, we will now have the final HF data
        // and can shut down the elevation requests for this tile.
        if ( _elevRequest->isCompleted() )
        {
            if ( _elevRequest->wasCanceled() )
            {
                // If the request was canceled, reset it to IDLE and reset the callback. On the next
                _elevRequest->setState( TaskRequest::STATE_IDLE );
                _elevRequest->setProgressCallback( new ProgressCallback() );            
                _elevRequest->reset();
            }
            else // success:
            {
                // if the elevation request succeeded, install the new elevation layer!
                TileElevationLayerRequest* r = static_cast<TileElevationLayerRequest*>( _elevRequest.get() );
                osg::ref_ptr<osgTerrain::HeightFieldLayer> newHFLayer = static_cast<osgTerrain::HeightFieldLayer*>( r->getResult() );
                if ( newHFLayer.valid() && newHFLayer->getHeightField() != NULL )
                {
                    newHFLayer->getHeightField()->setSkirtHeight( 
                        terrain->getTileFactory()->getTerrainOptions().heightFieldSkirtRatio().get() *
                        this->getBound().radius() );

                    // need to write-lock the layer data since we'll be changing it:
                    {
                        Threading::ScopedWriteLock lock( _tileLayersMutex );
                        this->setElevationLayer( newHFLayer.get() );
                        this->dirtyBound();
                    }

                    // the tile needs rebuilding. This will kick off a TileGenRequest.
                    queueTileUpdate( TileUpdate::UPDATE_ELEVATION );

                    // finalize the LOD marker for this tile, so other tiles can see where we are.
                    _elevationLOD = _key.getLevelOfDetail();

    #ifdef PREEMPTIVE_DEBUG
                    OE_NOTICE << "Tile (" << _key.str() << ") final HF, LOD (" << _elevationLOD << ")" << std::endl;
    #endif
                    // this was the final elev request, so mark elevation as DONE.
                    _elevationLayerUpToDate = true;

                    // GW- just reset these and leave them alone and let cancelRequests() take care of cleanup later.
                    // done with our Elevation requests!
                    //_elevRequest = 0L;
                    //_elevPlaceholderRequest = 0L;
                }
                else
                {
                    //We've tried to get the tile's elevation but couldn't.  Just mark the elevation layer as up to date and move on.
                    _elevationLOD = _key.getLevelOfDetail();
                    _elevationLayerUpToDate = true;

                    //This code will retry indefinitely.  We need to have a way to limit the number of retries since
                    //it will block neighbor tiles from loading.
                    //_elevRequest->setState( TaskRequest::STATE_IDLE );
                    //_elevRequest->reset();
                }
            }
        }

        else if ( _elevPlaceholderRequest->isCompleted() )
        {
            TileElevationPlaceholderLayerRequest* r = 
                static_cast<TileElevationPlaceholderLayerRequest*>(_elevPlaceholderRequest.get());

            if ( r->wasCanceled() )
            {
                r->setState( TaskRequest::STATE_IDLE );
                r->setProgressCallback( new ProgressCallback() );
                r->reset();
            }
            else // success:
            {
                osg::ref_ptr<osgTerrain::HeightFieldLayer> newPhLayer = static_cast<osgTerrain::HeightFieldLayer*>( r->getResult() );
                if ( newPhLayer.valid() && newPhLayer->getHeightField() != NULL )
                {
                    // install the new elevation layer.
                    {
                        Threading::ScopedWriteLock lock( _tileLayersMutex );
                        this->setElevationLayer( newPhLayer.get() );
                        this->dirtyBound();
                    }

                    // tile needs to be recompiled.
                    queueTileUpdate( TileUpdate::UPDATE_ELEVATION );

                    // update the elevation LOD for this tile, now that the new HF data is installed. This will
                    // allow other tiles to see where this tile's HF data is.
                    _elevationLOD = r->_nextLOD;

    #ifdef PREEMPTIVE_DEBUG
                    OE_NOTICE << "..tile (" << _key.str() << ") is now at (" << _elevationLOD << ")" << std::endl;
    #endif
                }
                _elevPlaceholderRequest->setState( TaskRequest::STATE_IDLE );
                _elevPlaceholderRequest->reset();
            }
        }
    }

    // if we have a new TileGenRequest, queue it up now.
    if ( _tileUpdates.size() > 0 && !_tileGenRequest.valid() ) // _tileGenNeeded && !_tileGenRequest.valid())
    {
        _tileGenRequest = new TileGenRequest( this, _tileUpdates.front() );
        _tileUpdates.pop();
        //OE_NOTICE << "tile (" << _key.str() << ") queuing new tile gen" << std::endl;
        getStreamingTerrain()->getTileGenerationTaskService()->add( _tileGenRequest.get() );
    }

    return tileModified;
}
