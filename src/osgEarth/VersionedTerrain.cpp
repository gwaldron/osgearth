/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarth/VersionedTerrain>
#include <osgEarth/Registry>
#include <osgEarth/Locators>
#include <osgEarth/EarthTerrainTechnique>
#include <osgEarth/Map>
#include <osgEarth/MapEngine>
#include <OpenThreads/ScopedLock>
#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/Node>

using namespace osgEarth;
using namespace OpenThreads;

//#define PREEMPTIVE_DEBUG 1

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
        //osg::notify(osg::NOTICE) << "Marking cancelled " << _request->getName() << std::endl;
        return _canceled;
    }

    TaskRequest* _request;
    TaskService* _service;
};


/*****************************************************************************/

struct TileLayerRequest : public TaskRequest
{
    TileLayerRequest( const TileKey* key, Map* map, MapEngine* engine )
        : _key( key ), _map(map), _engine(engine) { }

    osg::ref_ptr<const TileKey> _key;
    osg::ref_ptr<Map>           _map;
    osg::ref_ptr<MapEngine>     _engine;
};

struct TileColorLayerRequest : public TileLayerRequest
{
    TileColorLayerRequest( const TileKey* key, Map* map, MapEngine* engine, int layerIndex, unsigned int layerId )
        : TileLayerRequest( key, map, engine ), _layerIndex( layerIndex ), _layerId(layerId) { }

    void operator()( ProgressCallback* progress )
    {
        MapLayer* mapLayer = 0L;
        {
            ScopedReadLock lock( _map->getMapDataMutex() );
            if ( _layerIndex < _map->getImageMapLayers().size() )
                mapLayer = _map->getImageMapLayers()[_layerIndex].get();
        }
        if ( mapLayer )
        {
            osg::ref_ptr<GeoImage> image = mapLayer->createImage( _key.get(), progress );
            if ( image.get() )
                _result = _engine->createImageLayer( _map.get(), _key.get(), image.get() );
        }
    }
    int _layerIndex;
    unsigned int _layerId;
};

struct TileElevationLayerRequest : public TileLayerRequest
{
    TileElevationLayerRequest( const TileKey* key, Map* map, MapEngine* engine )
        : TileLayerRequest( key, map, engine )
    {
    }

    void operator()( ProgressCallback* progress )
    {
        _result = _engine->createHeightFieldLayer( _map.get(), _key.get(), true ); //exactOnly=true
    }
};

struct TileElevationPlaceholderLayerRequest : public TileLayerRequest
{
    TileElevationPlaceholderLayerRequest( const TileKey* key, Map* map, MapEngine* engine, GeoLocator* keyLocator )
        : TileLayerRequest( key, map, engine ),
          _keyLocator(keyLocator)
    {
        _parentKey = key->createParentKey();
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
        _result = _engine->createPlaceholderHeightfieldLayer(
            _parentHF.get(),
            _parentKey.get(),
            _key.get(),
            _keyLocator.get() );
    }

    osg::ref_ptr<osg::HeightField> _parentHF;
    osg::ref_ptr<const TileKey> _parentKey;
    osg::ref_ptr<GeoLocator>    _keyLocator;
    int _nextLOD;
};

struct TileGenRequest : public TaskRequest
{
    TileGenRequest( VersionedTile* tile ) : _tile(tile) { }

    void operator()( ProgressCallback* progress )
    {
        osg::ref_ptr<VersionedTile> tempRef = _tile.get();
        if ( tempRef.valid() )
        {
            EarthTerrainTechnique* tech = dynamic_cast<EarthTerrainTechnique*>( tempRef->getTerrainTechnique() );
            if ( tech )
            {
                tech->init( false );
            }
        }
    }

    osg::observer_ptr<VersionedTile> _tile;
};


/*****************************************************************************/

// family tile indicies in the _family vector
#define PARENT 0
#define WEST   1
#define NORTH  2
#define EAST   3
#define SOUTH  4


VersionedTile::VersionedTile( const TileKey* key, GeoLocator* keyLocator ) :
_key( key ),
_keyLocator( keyLocator ),
_useLayerRequests( false ),       // always set this to false here; use setUseLayerRequests() to enable
_terrainRevision( -1 ),
_tileRevision( 0 ),
_geometryRevision( 0 ),
_requestsInstalled( false ),
_elevationLayerDirty( false ),
_colorLayersDirty( false ),
_usePerLayerUpdates( false ),     // only matters when _useLayerRequests==true
_elevationLayerUpToDate( true ),
_family( 5 ),
_elevationLOD( key->getLevelOfDetail() ),
_tileRegisteredWithTerrain( false ),
_useTileGenRequest( true ),
_tileGenNeeded( false ),
_needsUpdate(false)
{
    setTileID( key->getTileId() );

    // because the lowest LOD (1) is always loaded fully:
    _elevationLayerUpToDate = _key->getLevelOfDetail() <= 1;
}

VersionedTile::~VersionedTile()
{
    //osg::notify(osg::NOTICE) << "Destroying VersionedTile " << this->getKey()->str() << std::endl;
}

void
VersionedTile::cancelRequests()
{
    //Cancel any pending requests
    if (_requestsInstalled)
    {
        for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); ++i )
        {
            if (i->get()->getState() == TaskRequest::STATE_IN_PROGRESS)
            {
                //osg::notify(osg::NOTICE) << "IR (" << _key->str() << ") in progress, cancelling " << std::endl;
            }
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
}


OpenThreads::ReadWriteMutex&
VersionedTile::getTileLayersMutex()
{
    return _tileLayersMutex;
}

const TileKey*
VersionedTile::getKey() const
{
    return _key.get();
}

void
VersionedTile::setElevationLOD( int lod )
{
    _elevationLOD = lod;
}

int
VersionedTile::getElevationLOD() const
{
    return _elevationLOD;
}

VersionedTerrain*
VersionedTile::getVersionedTerrain()
{
    if ( !_versionedTerrain.valid() )
        _versionedTerrain = static_cast<VersionedTerrain*>(getTerrain());
    return _versionedTerrain.get();
}

const VersionedTerrain*
VersionedTile::getVersionedTerrain() const
{
    return const_cast<VersionedTile*>(this)->getVersionedTerrain();
}

void
VersionedTile::setUseLayerRequests( bool value )
{
    if ( _useLayerRequests != value )
    {
        _useLayerRequests = value;   

        // if layer requests are on, we need an update traversal.
        //int oldNum = getNumChildrenRequiringUpdateTraversal();
        //setNumChildrenRequiringUpdateTraversal( _useLayerRequests? oldNum+1 : oldNum-1 );
    }
}

int
VersionedTile::getTerrainRevision() const
{
    return _terrainRevision;
}

void
VersionedTile::setTerrainRevision( int revision )
{
    _terrainRevision = revision;
}

bool
VersionedTile::isInSyncWithTerrain() const
{
    return _terrainRevision == getVersionedTerrain()->getRevision();
}

int
VersionedTile::getTileRevision() const
{
    return _tileRevision;
}

void
VersionedTile::incrementTileRevision()
{
    _tileRevision++;
}

void
VersionedTile::setHasElevationHint( bool hint ) 
{
    _hasElevation = hint;
}

bool
VersionedTile::isElevationLayerUpToDate() const 
{
    return _elevationLayerUpToDate;
}

// returns TRUE if it's safe for this tile to load its next elevation data layer.
bool
VersionedTile::readyForNewElevation()
{
    bool ready = true;

    if ( _elevationLOD == _key->getLevelOfDetail() )
    {
        ready = false;
    }
    else if ( _family[PARENT].elevLOD < 0 )
    {
        ready = false;
    }
    else
    {
        for( int i=PARENT; i<=SOUTH; i++) 
        {
            if ( _family[i].exists && _family[i].elevLOD >= 0 && _family[i].elevLOD < _elevationLOD )
            {
                ready = false;
                break;
            }
        }

        // if the next LOD is not the final, but our placeholder is up to date, we're not ready.
        if ( ready && _elevationLOD+1 < _key->getLevelOfDetail() && _elevationLOD == _family[PARENT].elevLOD )
        {
            ready = false;
        }
    }

#ifdef PREEMPTIVE_DEBUG
    osg::notify(osg::NOTICE)
        << "Tile (" << _key->str() << ") at (" << _elevationLOD << "), parent at ("
        << _family[PARENT].elevLOD << "), sibs at (";
    if ( _family[WEST].exists ) osg::notify( osg::NOTICE ) << "W=" << _family[WEST].elevLOD << " ";
    if ( _family[NORTH].exists ) osg::notify( osg::NOTICE ) << "N=" << _family[NORTH].elevLOD << " ";
    if ( _family[EAST].exists ) osg::notify( osg::NOTICE ) << "E=" << _family[EAST].elevLOD << " ";
    if ( _family[SOUTH].exists ) osg::notify( osg::NOTICE ) << "S=" << _family[SOUTH].elevLOD << " ";
    osg::notify(osg::NOTICE) << "), ready = " << (ready? "YES" : "no") << std::endl;
#endif

    return ready;
}


#define PRI_IMAGE_OFFSET 0.1f // priority offset of imagery relative to elevation
#define PRI_LAYER_OFFSET 0.1f // priority offset of image layer(x) vs. image layer(x+1)

void
VersionedTile::checkNeedsUpdate()
{
    //See if we have any completed requests
    bool hasCompletedRequests = false;

    if (_elevRequest.valid() && _elevRequest->isCompleted()) hasCompletedRequests = true;
    else if (_elevPlaceholderRequest.valid() && _elevPlaceholderRequest->isCompleted()) hasCompletedRequests = true;
    else if (_tileGenRequest.valid() && _tileGenRequest->isCompleted()) hasCompletedRequests = true;
    for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); ++i )
    {
        if (i->get()->isCompleted())
        {
            hasCompletedRequests = true;
        }
    }

    if (hasCompletedRequests && !_needsUpdate)
    {
        setNumChildrenRequiringUpdateTraversal( getNumChildrenRequiringUpdateTraversal() + 1 );
        _needsUpdate = true;
    }
    else if (!hasCompletedRequests && _needsUpdate)
    {
        setNumChildrenRequiringUpdateTraversal( getNumChildrenRequiringUpdateTraversal() - 1 );
        _needsUpdate = false;
    }
}

void
VersionedTile::installRequests( int stamp )
{
    VersionedTerrain* terrain = getVersionedTerrain();

    Map* map = terrain->getMap();
    OpenThreads::ScopedReadLock lock( map->getMapDataMutex() );

    MapEngine* engine = terrain->getEngine();

    if ( _hasElevation && this->getElevationLayer() ) // don't need a layers lock here
    {
        // TODO: insert an isKeyValid() here as well. But first, think about it....

        _elevRequest = new TileElevationLayerRequest(_key.get(), map, engine );
        //_elevRequest->setPriority( (float)_key->getLevelOfDetail() );
        float priority = (float)_key->getLevelOfDetail();
        _elevRequest->setPriority( priority );
        std::stringstream ss;
        ss << "TileElevationLayerRequest " << _key->str() << std::endl;
        _elevRequest->setName( ss.str() );

        _elevPlaceholderRequest = new TileElevationPlaceholderLayerRequest(
            _key.get(), map, engine, _keyLocator.get() );
        _elevPlaceholderRequest->setPriority( priority );
        ss.str("");
        ss << "TileElevationPlaceholderLayerRequest " << _key->str() << std::endl;
        _elevPlaceholderRequest->setName( ss.str() );
    }

    _tileGenRequest = new TileGenRequest( this );

    int numColorLayers = getNumColorLayers();
    for( int layerIndex = 0; layerIndex < numColorLayers; layerIndex++ ) 
    {
        if (layerIndex < map->getImageMapLayers().size())
        {
            MapLayer* mapLayer = map->getImageMapLayers()[layerIndex].get();
            if ( mapLayer->isKeyValid( _key.get() ) )
            {
                unsigned int layerId = mapLayer->getId();
                // imagery is slighty higher priority than elevation data
                TaskRequest* r = new TileColorLayerRequest( _key.get(), map, engine, layerIndex, layerId );
                std::stringstream ss;
                ss << "TileColorLayerRequest " << _key->str() << std::endl;
                r->setName( ss.str() );
                r->setPriority( PRI_IMAGE_OFFSET + (float)_key->getLevelOfDetail() + (PRI_LAYER_OFFSET * (float)(numColorLayers-1-layerIndex)) );
                r->setStamp( stamp );
                r->setProgressCallback( new StampedProgressCallback( r, terrain->getImageryTaskService( layerIndex ) ));
                _requests.push_back( r );
            }
        }
    }
}

// This method is called from the CULL TRAVERSAL, from TileLoaderCullCallback in MapEngine.cpp.
void
VersionedTile::servicePendingImageRequests( int stamp )
{       
    // install our requests if they are not already installed:
    if ( !_requestsInstalled )
    {
        installRequests( stamp );
        _requestsInstalled = true;
    }

    for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); ++i )
    {
        TileColorLayerRequest* r = static_cast<TileColorLayerRequest*>( i->get() );

        //If a request has been marked as IDLE, the TaskService has tried to service it
        //and it was either deemed out of date or was cancelled, so we need to add it again.
        if ( r->isIdle() )
        {
            //osg::notify(osg::NOTICE) << "Queuing IR (" << _key->str() << ")" << std::endl;
            r->setStamp( stamp );
            getVersionedTerrain()->getImageryTaskService(r->_layerId)->add( r );
        }
        else if ( !r->isCompleted() )
        {
            r->setStamp( stamp );
        }
    }

    checkNeedsUpdate();
}

// This method is called from the CULL TRAVERSAL, from VersionedTerrain::traverse.
void
VersionedTile::servicePendingElevationRequests( int stamp )
{
    // install our requests if they are not already installed:
    if ( !_requestsInstalled )
    {
        installRequests( stamp );
        _requestsInstalled = true;
    }

    // make sure we know where to find our parent and sibling tiles:
    if (getVersionedTerrain())
    {
        getVersionedTerrain()->refreshFamily( getTileID(), _family );
        //readyForNewElevation();
        //osg::notify(osg::NOTICE) << "TILE (" << _key->str() << ") _has=" << _hasElevation << ", _upToDate="
        //    << _elevationLayerUpToDate 
        //    << " ERvalid=" << _elevRequest.valid() << ", "
        //    << " PRvalid=" << _elevPlaceholderRequest.valid()
        //    << " reqInst=" << _requestsInstalled
        //    << std::endl;
    }
    else
    {
        //osg::notify(osg::NOTICE) << "TILE (" << _key->str() << ") HAS NO TERRAIN." << std::endl;
    }

    if ( _hasElevation && !_elevationLayerUpToDate && _elevRequest.valid() && _elevPlaceholderRequest.valid() )
    {  
        VersionedTerrain* terrain = getVersionedTerrain();

        // update the main elevation request if it's running:
        if ( !_elevRequest->isIdle() )
        {
#ifdef PREEMPTIVE_DEBUG
            osg::notify(osg::NOTICE) << "Tile (" << _key->str() << ") .. ER not idle" << std::endl;
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
            osg::notify(osg::NOTICE) << "Tile (" << _key->str() << ") .. PR not idle" << std::endl;
#endif
            if ( !_elevPlaceholderRequest->isCompleted() )
            {
               _elevPlaceholderRequest->setStamp( stamp );
            }
        }

        // otherwise, see if it is legal yet to start a new request:
        else if ( readyForNewElevation() )
        {
            if ( _elevationLOD + 1 == _key->getLevelOfDetail() )
            {
                _elevRequest->setStamp( stamp );
                _elevRequest->setProgressCallback( new ProgressCallback() );
                terrain->getElevationTaskService()->add( _elevRequest.get() );
#ifdef PREEMPTIVE_DEBUG
                osg::notify(osg::NOTICE) << "..queued FE req for (" << _key->str() << ")" << std::endl;
#endif
            }
            
            else if ( _family[PARENT].elevLOD > _elevationLOD )
            {
                osg::ref_ptr<VersionedTile> parentTile = _family[PARENT].tile.get();
                if ( _elevationLOD < _family[PARENT].elevLOD && parentTile.valid() )
                {
                    TileElevationPlaceholderLayerRequest* er = static_cast<TileElevationPlaceholderLayerRequest*>(_elevPlaceholderRequest.get());

                    er->setStamp( stamp );
                    er->setProgressCallback( new ProgressCallback() ); //( ck( er, terrain->getElevationTaskService()));
                    float priority = (float)_key->getLevelOfDetail();
                    er->setPriority( priority );
                    osgTerrain::HeightFieldLayer* hfLayer = static_cast<osgTerrain::HeightFieldLayer*>(parentTile->getElevationLayer());
                    er->setParentHF( hfLayer->getHeightField() );
                    er->setNextLOD( _family[PARENT].elevLOD );
                    terrain->getElevationTaskService()->add( er );
#ifdef PREEMPTIVE_DEBUG
                    osg::notify(osg::NOTICE) << "..queued PH req for (" << _key->str() << ")" << std::endl;
#endif
                }

                else 
                {
#ifdef PREEMPTIVE_DEBUG
                    osg::notify(osg::NOTICE) << "...tile (" << _key->str() << ") ready, but nothing to do." << std::endl;
#endif
                }
            }
        }
    }

    checkNeedsUpdate();
}

// This method is called from the CULL TRAVERSAL, and only is _useLayerRequests == true.
//void
//VersionedTile::servicePendingRequests( int stamp )
//{
//    VersionedTerrain* terrain = getVersionedTerrain();
//    if ( !terrain ) return;
//
//    // make sure our refernece to the parent is up to date.
//    //refreshParentTile();
//
//    if ( _requestsInstalled )
//    {
//        servicePendingImageRequests( stamp );
//        servicePendingElevationRequests( stamp );
//    }
//}

// called from the UPDATE TRAVERSAL.
void
VersionedTile::serviceCompletedRequests()
{
    if ( !_requestsInstalled )
        return;

    // First service the tile generator:
    if ( _tileGenRequest->isCompleted() )
    {
        EarthTerrainTechnique* tech = dynamic_cast<EarthTerrainTechnique*>( getTerrainTechnique() );
        if ( tech )
            tech->swapIfNecessary();

        _tileGenRequest->setState( TaskRequest::STATE_IDLE );
    }

    // Then the image requests:
    for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); )
    {
        bool increment = true;

        if ( dynamic_cast<TileColorLayerRequest*>( i->get() ) )
        {
            TileColorLayerRequest* r = static_cast<TileColorLayerRequest*>( i->get() );
            if ( r->isCompleted() )
            {
                osgTerrain::ImageLayer* imgLayer = static_cast<osgTerrain::ImageLayer*>( r->getResult() );
                if ( imgLayer )
                {
                    this->setColorLayer( r->_layerIndex, imgLayer );
                    if ( _useTileGenRequest )
                    {
                        _tileGenNeeded = true;
                    }
                    else
                    {
                        if ( _usePerLayerUpdates )
                            _colorLayersDirty = true;
                        else
                            this->setDirty( true );
                    }

                    // remove from the list
                    i = _requests.erase( i );
                    increment = false;
                    
                    //osg::notify(osg::NOTICE) << "Complet IR (" << _key->str() << ")" << std::endl;
                }
                else
                {                
                    osg::notify(osg::NOTICE) << "IReq error (" << _key->str() << "), retrying" << std::endl;

                    //The color layer request failed, probably due to a server error. Reset it.
                    r->setState( TaskRequest::STATE_IDLE );
                    r->reset();
                }
            }
            else if ( r->isCanceled() )
            {
                //Reset the cancelled task to IDLE and give it a new progress callback.
                i->get()->setState( TaskRequest::STATE_IDLE );
                i->get()->setProgressCallback( new StampedProgressCallback(
                    i->get(), getVersionedTerrain()->getImageryTaskService(r->_layerId)));
                r->reset();
            }
        }

        if ( increment )
            ++i;
    }

    // Finally, the elevation requests:
    if ( _hasElevation && !_elevationLayerUpToDate && _elevRequest.valid() && _elevPlaceholderRequest.valid() )
    {
        if ( _elevRequest->isCompleted() )
        {
            // if the elevation request succeeded, install the new elevation layer!
            TileElevationLayerRequest* er = static_cast<TileElevationLayerRequest*>( _elevRequest.get() );
            osgTerrain::HeightFieldLayer* hfLayer = static_cast<osgTerrain::HeightFieldLayer*>( er->getResult() );
            if ( hfLayer )
            {
                // need to write-lock the layer data since we'll be changing it:
                ScopedWriteLock lock( _tileLayersMutex );

                // copy the skirt height over:
                osg::HeightField* oldHF = static_cast<osgTerrain::HeightFieldLayer*>(getElevationLayer())->getHeightField();
                if ( oldHF )
                    hfLayer->getHeightField()->setSkirtHeight( oldHF->getSkirtHeight() );

                bool sameSize = true;
                if ( oldHF )
                {
                    if (oldHF->getNumColumns() != hfLayer->getNumColumns() || 
                        oldHF->getNumRows() != hfLayer->getNumRows())
                    {
                        sameSize = false;
                    }
                }

                this->setElevationLayer( hfLayer );
                
                if ( _useTileGenRequest )
                {
                    _tileGenNeeded = true;
                }
                else
                {
                    if ( _usePerLayerUpdates && sameSize )
                        _elevationLayerDirty = true;
                    else
                        this->setDirty( true );
                }
                
                _elevationLOD = _key->getLevelOfDetail();

#ifdef PREEMPTIVE_DEBUG
                osg::notify(osg::NOTICE) << "Tile (" << _key->str() << ") final HF, LOD (" << _elevationLOD << ")" << std::endl;
#endif
                _elevationLayerUpToDate = true;
                
                // done with our Elevation requests!
                _elevRequest = 0L;
                _elevPlaceholderRequest = 0L;
            }
            else
            {
                //Reque the elevation request.
                _elevRequest->setState( TaskRequest::STATE_IDLE );
                _elevRequest->reset();
            }
        }

        else if ( _elevRequest->isCanceled() )
        {
            // If the request was canceled, reset it to IDLE and reset the callback. On the next
            // servicePendingRequests, the request will be re-scheduled.
            _elevRequest->setState( TaskRequest::STATE_IDLE );
            _elevRequest->setProgressCallback( new ProgressCallback() );            
            _elevRequest->reset();
        }

        else if ( _elevPlaceholderRequest->isCompleted() )
        {
            TileElevationPlaceholderLayerRequest* er = static_cast<TileElevationPlaceholderLayerRequest*>(_elevPlaceholderRequest.get());

            osgTerrain::HeightFieldLayer* newPhLayer = static_cast<osgTerrain::HeightFieldLayer*>( er->getResult() );

            // write-lock the layer data since we'll be changing it:
            ScopedWriteLock lock( _tileLayersMutex );

            // copy the skirt height over:
            osg::HeightField* oldHF = static_cast<osgTerrain::HeightFieldLayer*>(getElevationLayer())->getHeightField();
            if ( oldHF )
                newPhLayer->getHeightField()->setSkirtHeight( oldHF->getSkirtHeight() );

            bool sameSize = true;
            if ( oldHF )
            {
                if (oldHF->getNumColumns() != newPhLayer->getNumColumns() || 
                    oldHF->getNumRows() != newPhLayer->getNumRows())
                {
                    sameSize = false;
                }
            }

            if ( newPhLayer )
            {
                this->setElevationLayer( newPhLayer );
                
                if ( _useTileGenRequest )
                {
                    _tileGenNeeded = true;
                }
                else
                {
                    if ( _usePerLayerUpdates && sameSize )
                        _elevationLayerDirty = true;
                    else
                        this->setDirty( true );
                }

                _elevationLOD = er->_nextLOD;
#ifdef PREEMPTIVE_DEBUG
                osg::notify(osg::NOTICE) << "..tile (" << _key->str() << ") is now at (" << _elevationLOD << ")" << std::endl;
#endif
            }
            _elevPlaceholderRequest->setState( TaskRequest::STATE_IDLE );
            _elevPlaceholderRequest->reset();
        }

        else if ( _elevPlaceholderRequest->isCanceled() )
        {
            _elevPlaceholderRequest->setState( TaskRequest::STATE_IDLE );
            _elevPlaceholderRequest->setProgressCallback( new ProgressCallback() );
            _elevPlaceholderRequest->reset();
        }
    }

    // if we have a new TileGenRequest, queue it up now.
    if ( _tileGenNeeded && _tileGenRequest->isIdle() )
    {
        //osg::notify(osg::NOTICE) << "tile (" << _key->str() << ") queuing new tile gen" << std::endl;

        getVersionedTerrain()->getElevationTaskService()->add( _tileGenRequest.get() );
        _tileGenNeeded = false;
    }

    checkNeedsUpdate();
}

void
VersionedTile::traverse( osg::NodeVisitor& nv )
{
    // register this tile with its terrain if we've not already done it.
    if ( !_tileRegisteredWithTerrain && getVersionedTerrain() )
    {
        getVersionedTerrain()->registerTile( this );
        _tileRegisteredWithTerrain = true;
    }

    if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
    {
        if ( getVersionedTerrain()->updateBudgetRemaining() )
        {
            bool serviceRequests = _useLayerRequests && nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR;

            if ( serviceRequests )
            {
                serviceCompletedRequests();

                if ( getDirty() ) 
                {
                    // if the whole tile is dirty, let it rebuild via the normal recourse:
                    _elevationLayerDirty = true;
                    _colorLayersDirty = true;
                }
                else if ( _elevationLayerDirty || _colorLayersDirty )
                {
                    // if the tile is only partly dirty, update it piecemeal:
                    EarthTerrainTechnique* tech = static_cast<EarthTerrainTechnique*>( getTerrainTechnique() );
                    {
                        ScopedReadLock lock( _tileLayersMutex );
                        tech->updateContent( _elevationLayerDirty, _colorLayersDirty );
                    }
                }
            }

            // continue the normal traversal. If the tile is "dirty" it will regenerate here.
            osgTerrain::TerrainTile::traverse( nv );

            if ( serviceRequests )
            {
                // bump the geometry revision if the tile's geometry was updated.
                if ( _elevationLayerDirty )
                    _geometryRevision++;

                _elevationLayerDirty = false;
                _colorLayersDirty = false;        
            }
        }
    }
    else
    {
        osgTerrain::TerrainTile::traverse( nv );
    }
}

void VersionedTile::releaseGLObjects(osg::State* state) const
{
    Group::releaseGLObjects(state);

    if (_terrainTechnique.valid())
    {
        //NOTE: crashes sometimes if OSG_RELEASE_DELAY is set -gw
        _terrainTechnique->releaseGLObjects( state );
    }
}


/****************************************************************************/


VersionedTerrain::VersionedTerrain( Map* map, MapEngine* engine ) :
_map( map ),
_engine( engine ),
_revision(0),
_numAsyncThreads( 0 ),
_updateBudgetSeconds( 0.0167 )
{
    //See if the number of threads is explicitly provided
    const optional<int>& numThreads = engine->getEngineProperties().getNumLoadingThreads();
    if (numThreads.isSet())
    {
        _numAsyncThreads = numThreads.get();
    }

    //See if an environment variable was set
    const char* env_numTaskServiceThreads = getenv("OSGEARTH_NUM_PREEMPTIVE_LOADING_THREADS");
    if ( env_numTaskServiceThreads )
    {
        _numAsyncThreads = ::atoi( env_numTaskServiceThreads );
    }

    //See if a processors per core option was provided
    if (_numAsyncThreads == 0)
    {
        const optional<int>& threadsPerProcessor = engine->getEngineProperties().getNumLoadingThreadsPerLogicalProcessor();
        if (threadsPerProcessor.isSet())
        {
            _numAsyncThreads = OpenThreads::GetNumberOfProcessors() * threadsPerProcessor.get();
        }
    }

    //Default to using 2 threads per core
    if (_numAsyncThreads == 0 )
    {
        _numAsyncThreads = OpenThreads::GetNumberOfProcessors() * 2;
    }

    osg::notify(osg::INFO) << "Using " << _numAsyncThreads << " loading threads " << std::endl;

    //Install a time-budget for update traversals that handle the tile requests.
    const char* env_updateTravBudgetSeconds = getenv("OSGEARTH_PREEMPTIVE_UPDATE_MAX_SECONDS_PER_FRAME");
    if ( env_updateTravBudgetSeconds )
    {
        _updateBudgetSeconds = ::atof( env_updateTravBudgetSeconds );
        if ( _updateBudgetSeconds <= 0.0 )
            _updateBudgetSeconds = 999.0;

        osg::notify(osg::NOTICE) << "[osgEarth] Tile update budget = " << _updateBudgetSeconds << " seconds per frame" << std::endl;
    }
}

void
VersionedTerrain::incrementRevision()
{
    // no need to lock; if we miss it, we'll get it the next time around
    _revision++;
}

int
VersionedTerrain::getRevision() const
{
    // no need to lock; if we miss it, we'll get it the next time around
    return _revision;
}

void
VersionedTerrain::getVersionedTile( const osgTerrain::TileID& tileID, osg::ref_ptr<VersionedTile>& out_tile )
{
    ScopedReadLock lock( _tilesMutex );
    TileTable::iterator i = _tiles.find( tileID );
    out_tile = i != _tiles.end()? i->second.get() : 0L;
}

void
VersionedTerrain::getVersionedTile( const osgTerrain::TileID& tileID, osg::observer_ptr<VersionedTile>& out_tile )
{
    ScopedReadLock lock( _tilesMutex );
    TileTable::iterator i = _tiles.find( tileID );
    out_tile = i != _tiles.end()? i->second.get() : 0L;
}

void
VersionedTerrain::getTerrainTiles( TerrainTileList& out_list )
{
    ScopedReadLock lock( _tilesMutex );
    for( TileTable::iterator i = _tiles.begin(); i != _tiles.end(); i++ )
    {
        out_list.push_back( i->second.get() );
    }
}

//VersionedTile*
//VersionedTerrain::getVersionedTile(const osgTerrain::TileID& tileID)
//{
//    ScopedLock<Mutex> lock(_mutex);
//
//    TerrainTileMap::iterator itr = _terrainTileMap.find(tileID);
//    if (itr == _terrainTileMap.end()) return 0;
//
//    return static_cast<VersionedTile*>(itr->second); //.get());
//}
//
//VersionedTile*
//VersionedTerrain::getVersionedTileNoLock(const osgTerrain::TileID& tileID)
//{
//    TerrainTileMap::iterator itr = _terrainTileMap.find(tileID);
//    if (itr == _terrainTileMap.end()) return 0;
//    return static_cast<VersionedTile*>(itr->second); //.get());
//}

void
VersionedTerrain::refreshFamily(const osgTerrain::TileID& tileId,
                                Relatives& family)
{
    osg::observer_ptr<VersionedTile> t;

    // parent
    family[PARENT].exists = true;
    if ( ! family[PARENT].tile.valid() )
    {
        getVersionedTile( osgTerrain::TileID( tileId.level-1, tileId.x/2, tileId.y/2 ), family[PARENT].tile );
    }
    if ( family[PARENT].tile.valid() )
        family[PARENT].elevLOD = family[PARENT].tile->getElevationLOD();

    // technically it should check for a geocentric rendering...
    bool wrapX = _map->getCoordinateSystemType() == Map::CSTYPE_GEOCENTRIC;

    unsigned int tilesX, tilesY;
    _map->getProfile()->getNumTiles( tileId.level, tilesX, tilesY );

    unsigned int x, y;

    // west
    family[WEST].exists = tileId.x > 0 || wrapX;
    family[WEST].elevLOD = -1;
    if ( family[WEST].exists )
    {
        if ( !family[WEST].tile.valid() )
        {
            x = tileId.x > 0? tileId.x-1 : tilesX-1;
            y = tileId.y;
            getVersionedTile( osgTerrain::TileID( tileId.level, x, y ), family[WEST].tile );
        }
        if ( family[WEST].tile.valid() )
            family[WEST].elevLOD = family[WEST].tile->getElevationLOD();
    }

    // north
    family[NORTH].exists = tileId.y < tilesY-1;
    family[NORTH].elevLOD = -1;
    if ( family[NORTH].exists )
    {
        if ( !family[NORTH].tile.valid() )
        {
            x = tileId.x;
            y = tileId.y < tilesY-1 ? tileId.y+1 : 0;
            getVersionedTile( osgTerrain::TileID( tileId.level, x, y ), family[NORTH].tile );
        }
        if ( family[NORTH].tile.valid() )
            family[NORTH].elevLOD = family[NORTH].tile->getElevationLOD();
    }

    // east
    family[EAST].exists = tileId.x < tilesX-1 || wrapX;
    family[EAST].elevLOD = -1;
    if ( family[EAST].exists )
    {
        if ( !family[EAST].tile.valid() )
        {
            x = tileId.x < tilesX-1 ? tileId.x+1 : 0;
            y = tileId.y;
            getVersionedTile( osgTerrain::TileID( tileId.level, x, y ), family[EAST].tile );
        }
        if ( family[EAST].tile.valid() )
            family[EAST].elevLOD = family[EAST].tile->getElevationLOD();
    }

    // south
    family[SOUTH].exists = tileId.y > 0;
    family[SOUTH].elevLOD = -1;
    if ( family[SOUTH].exists )
    {
        if ( !family[SOUTH].tile.valid() )
        {   
            x = tileId.x;
            y = tileId.y > 0 ? tileId.y-1 : tilesY-1;
            getVersionedTile( osgTerrain::TileID( tileId.level, x, y ), family[SOUTH].tile );
        }
        if ( family[SOUTH].tile.valid() )
            family[SOUTH].elevLOD = family[SOUTH].tile->getElevationLOD();
    }
}

void
VersionedTerrain::registerTile( VersionedTile* newTile )
{
    ScopedWriteLock lock( _tilesMutex );
    _tilesToAdd.push( newTile );
}

void
VersionedTerrain::updateTileTable()
{
    ScopedWriteLock lock( _tilesMutex );

    int sizeOld = _tiles.size();

    for( TileTable::iterator i = _tiles.begin(); i != _tiles.end(); )
    {
        if ( i->second.valid() && i->second->referenceCount() == 1 )
        {
            i->second->cancelRequests();
            i = _tiles.erase( i );
        }
        else
        {
            ++i;
        }

    }

    while( _tilesToAdd.size() > 0 )
    {
        _tiles[ _tilesToAdd.front()->getTileID() ] = _tilesToAdd.front().get();
        _tilesToAdd.pop();
    }

    //if ( sizeOld != _tiles.size() )
    //    osg::notify(osg::NOTICE) << "TILES = " << _tiles.size() << std::endl;
}

Map*
VersionedTerrain::getMap() {
    return _map.get();
}

MapEngine*
VersionedTerrain::getEngine() {
    return _engine.get();
}

void
VersionedTerrain::traverse( osg::NodeVisitor &nv )
{
    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
        int stamp = nv.getFrameStamp()->getFrameNumber();

        // update the list of registered tiles in the terrain. SHoudl this be here,
        // or in the update visitor? does it matter?
        updateTileTable();

        // update the frame stamp on the task services:
        {
            ScopedLock<Mutex> lock( _taskServiceMutex );
            for (TaskServiceMap::iterator i = _taskServices.begin(); i != _taskServices.end(); ++i)
            {
                i->second->setStamp( stamp );
            }
        }

        // service data requests for each tile:
        {
            // dont' need a lock here, because the only time you can write to the tile table is 
            // in updateTileTable, called earlier in this method! Besides, locking here would
            // violate double-reentrant locks in servicePending*
            //ScopedReadLock lock( _tilesMutex );

            for( TileTable::iterator i = _tiles.begin(); i != _tiles.end(); ++i )
            {
                if ( i->second.valid() && i->second->getUseLayerRequests() )
                {
                    i->second->servicePendingElevationRequests( stamp );
                }
                //else
                //{
                //    osg::notify( osg::NOTICE ) << "TILE "
                //        << i->first.level << "," << i->first.x << "," << i->first.y
                //        << "   HAS an INVALID PTR in the TABLE" << std::endl;
                //}
            }
        }
    }

    else if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
    {
        _updateStartTime = osg::Timer::instance()->tick();
    }

    osgTerrain::Terrain::traverse( nv );
}

bool
VersionedTerrain::updateBudgetRemaining() const
{
    osg::Timer_t now = osg::Timer::instance()->tick();
    return osg::Timer::instance()->delta_s( _updateStartTime, now ) < _updateBudgetSeconds;
}

TaskService*
VersionedTerrain::createTaskService( int id, int numThreads )
{
    ScopedLock<Mutex> lock( _taskServiceMutex );
    TaskService* service =  new TaskService( numThreads );
    _taskServices[id] = service;
    return service;
}

TaskService*
VersionedTerrain::getTaskService(int id)
{
    ScopedLock<Mutex> lock( _taskServiceMutex );
    TaskServiceMap::iterator itr = _taskServices.find(id);
    if (itr != _taskServices.end())
    {
        return itr->second.get();
    }
    return NULL;
}

#define ELEVATION_TASK_SERVICE_ID 9999

TaskService*
VersionedTerrain::getElevationTaskService()
{
    TaskService* service = getTaskService( ELEVATION_TASK_SERVICE_ID );
    if (!service)
    {
        service = createTaskService( ELEVATION_TASK_SERVICE_ID, 1 );
    }
    return service;
}


TaskService*
VersionedTerrain::getImageryTaskService(int layerId)
{
    TaskService* service = getTaskService( layerId );
    if (!service)
    {
        service = createTaskService( layerId, 1 );
    }
    return service;
}

void
VersionedTerrain::updateTaskServiceThreads()
{
    OpenThreads::ScopedReadLock lock(_map->getMapDataMutex());    

    //Get the maximum elevation weight
    float elevationWeight = 0.0f;
    for (MapLayerList::const_iterator itr = _map->getHeightFieldMapLayers().begin(); itr != _map->getHeightFieldMapLayers().end(); ++itr)
    {
        float w = itr->get()->getLoadWeight();
        if (w > elevationWeight) elevationWeight = w;
    }

    float totalImageWeight = 0.0f;
    for (MapLayerList::const_iterator itr = _map->getImageMapLayers().begin(); itr != _map->getImageMapLayers().end(); ++itr)
    {
        totalImageWeight += itr->get()->getLoadWeight();
    }

    float totalWeight = elevationWeight + totalImageWeight;

    if (elevationWeight > 0.0f)
    {
        //Determine how many threads each layer gets
        int numElevationThreads = (int)osg::round((float)_numAsyncThreads * (elevationWeight / totalWeight ));
        osg::notify(osg::NOTICE) << "HtFld Threads = " << numElevationThreads << std::endl;
        getElevationTaskService()->setNumThreads( numElevationThreads );
    }

    for (MapLayerList::const_iterator itr = _map->getImageMapLayers().begin(); itr != _map->getImageMapLayers().end(); ++itr)
    {
        int imageThreads = (int)osg::round((float)_numAsyncThreads * (itr->get()->getLoadWeight() / totalWeight ));
        osg::notify(osg::NOTICE) << "Image Threads for " << itr->get()->getName() << " = " << imageThreads << std::endl;
        getImageryTaskService( itr->get()->getId() )->setNumThreads( imageThreads );
    }

}

