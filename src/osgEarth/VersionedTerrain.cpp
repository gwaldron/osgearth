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


struct TileRequestProgressCallback : ProgressCallback
{
public:
    TileRequestProgressCallback(TaskRequest* request, TaskService* service):
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
    }

    void setParentTile( VersionedTile* parentTile )
    {
        _parentTile = parentTile;
    }

    void operator()( ProgressCallback* progress )
    {
        if ( _parentTile.valid() )
        {
            osg::ref_ptr<osg::HeightField> parentHF;
            {
                ScopedReadLock lock( _parentTile->getTileLayersMutex() );
                parentHF = static_cast<osgTerrain::HeightFieldLayer*>(_parentTile->getElevationLayer())->getHeightField();
            }

            if ( parentHF.valid() )
            {
                _result = _engine->createPlaceholderHeightfieldLayer(
                    parentHF.get(),
                    _parentTile->getKey(),
                    _key.get(),
                    _keyLocator.get() );
            }
        }
    }

    osg::ref_ptr<VersionedTile> _parentTile;
    osg::ref_ptr<GeoLocator>    _keyLocator;
};

/*****************************************************************************/

// neighbor tile indicies
#define WEST  0
#define NORTH 1
#define EAST  2
#define SOUTH 3


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
_neighbors( 4 ),                  // pre-allocate 4 slots. 0=W, 1=N, 2=E, 3=S.
_elevationLOD( key->getLevelOfDetail() )
{
    setTileID( key->getTileId() );

    // because the lowest LOD (1) is always loaded fully:
    _elevationLayerUpToDate = _key->getLevelOfDetail() <= 1;

    setTerrainTechnique( new EarthTerrainTechnique( keyLocator ) );
}

VersionedTile::~VersionedTile()
{
    //osg::notify(osg::NOTICE) << "Destroying VersionedTile " << this->getKey()->str() << std::endl;

    //Cancel any pending requests
    if (_requestsInstalled)
    {
        for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); ++i )
        {
            //if (i->get()->getState() == TaskRequest::STATE_IN_PROGRESS)
            //{
            //    osg::notify(osg::NOTICE) << "Request (" << (int)this << ") in progress, cancelling " << std::endl;
            //}
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
    return static_cast<VersionedTerrain*>(getTerrain());
}
const VersionedTerrain*
VersionedTile::getVersionedTerrain() const
{
    return static_cast<const VersionedTerrain*>(getTerrain());
}

void
VersionedTile::setUseLayerRequests( bool value )
{
    if ( _useLayerRequests != value )
    {
        _useLayerRequests = value;   

        // if layer requests are on, we need an update traversal.
        int oldNum = getNumChildrenRequiringUpdateTraversal();
        setNumChildrenRequiringUpdateTraversal( _useLayerRequests? oldNum+1 : oldNum-1 );
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


// this safely refreshes the _parentTile member, and should be called at the top of the 
// any traversal that plans to use the reference.
void
VersionedTile::refreshParentTile()
{
    _parentTile = _parentTileObserver.get();
    if ( !_parentTile.valid() )
    {
        osg::ref_ptr<const TileKey> _parentKey = _key->createParentKey();
        _parentTileObserver = getVersionedTerrain()->getVersionedTile( _parentKey->getTileId() );
        _parentTile = _parentTileObserver.get();
    }
}


#define PRI_IMAGE_OFFSET 0.1f // priority offset of imagery relative to elevation
#define PRI_LAYER_OFFSET 0.1f // priority offset of image layer(x) vs. image layer(x+1)

// This method is called from the CULL TRAVERSAL, and only is _useLayerRequests == true.
void
VersionedTile::servicePendingRequests( int stamp )
{
    VersionedTerrain* terrain = getVersionedTerrain();
    if ( !terrain ) return;

    // make sure our refernece to the parent is up to date.
    refreshParentTile();

    // The first time through, initialize all the data request tasks we will need to load
    // imagery, elevation, and elevation placeholders.
    if ( !_requestsInstalled )
    {
        Map* map = terrain->getMap();
        OpenThreads::ScopedReadLock lock(map->getMapDataMutex());

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
                    r->setProgressCallback( new TileRequestProgressCallback( r, terrain->getImageryTaskService( layerIndex ) ));
                    _requests.push_back( r );
                }
            }
        }

        _requestsInstalled = true;
    }

    if ( _requestsInstalled )
    {
        for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); ++i )
        {
            TileColorLayerRequest* r = static_cast<TileColorLayerRequest*>( i->get() );

            //If a request has been marked as IDLE, the TaskService has tried to service it
            //and it was either deemed out of date or was cancelled, so we need to add it again.
            if ( r->isIdle() )
            {
                //osg::notify(osg::NOTICE) << "Re-queueing request " << std::endl;
                r->setStamp( stamp );
                getVersionedTerrain()->getImageryTaskService(r->_layerId)->add( r );
            }
            else if ( !r->isCompleted() )
            {
                r->setStamp( stamp );
            }
        }

        // if we have an elevation request standing by, check to see whether the parent tile's elevation is
        // loaded. If so, it is time to schedule this tile's elevation to load.
        if ( _hasElevation && !_elevationLayerUpToDate && _parentTile.valid() )
        {  
            // if the main elevation request is idle, that means we have not yet started to load
            // the final elevation layer. Therefore we need to check to see whether we need either
            // a new placeholder or the final elevation:
            if ( _elevRequest->isIdle() )
            {
                // see whether we need to start a new placeholder request:
                if ( _parentTile->getElevationLOD() > _elevationLOD )
                {
                    if ( _elevPlaceholderRequest->isIdle() )
                    {
                        _elevPlaceholderRequest->setProgressCallback( new TileRequestProgressCallback(
                            _elevPlaceholderRequest.get(), terrain->getElevationTaskService()));
                        float priority = (float)_key->getLevelOfDetail();
                        //_elevPlaceholderRequest->setPriority( _key->getLevelOfDetail() ); // tweak?
                        _elevPlaceholderRequest->setPriority( priority );
                        static_cast<TileElevationPlaceholderLayerRequest*>(_elevPlaceholderRequest.get())->setParentTile( _parentTile.get() );
                        terrain->getElevationTaskService()->add( _elevPlaceholderRequest.get() );
                    }
                    else if ( !_elevPlaceholderRequest->isCompleted() )
                    {
                        _elevPlaceholderRequest->setStamp( stamp );
                    }
                }

                // see whether it's time to request the real elevation data:
                else if ( _parentTile->getElevationLOD() == _key->getLevelOfDetail()-1 )
                {
                    _elevRequest->setStamp( stamp );

                    _elevRequest->setProgressCallback( new TileRequestProgressCallback(
                        _elevRequest.get(), terrain->getElevationTaskService() ) );

                    terrain->getElevationTaskService()->add( _elevRequest.get() );
                }
            }
            else if ( !_elevRequest->isCompleted() )
            {
                _elevRequest->setStamp( stamp );
            }
        }
    }
}

// called from the UPDATE TRAVERSAL.
void
VersionedTile::serviceCompletedRequests()
{
    // make sure our reference to the parent tile is up to date.
    refreshParentTile();

    VersionedTerrain *vt = static_cast<VersionedTerrain*>(getTerrain());

    for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); )
    {
        TileColorLayerRequest* r = static_cast<TileColorLayerRequest*>( i->get() );
        if ( r->isCompleted() )
        {
            osgTerrain::ImageLayer* imgLayer = static_cast<osgTerrain::ImageLayer*>( r->getResult() );
            if ( imgLayer )
            {
                if (vt->canDirtyMoreTiles())
                {
                    this->setColorLayer( r->_layerIndex, imgLayer );
                    if ( _usePerLayerUpdates )
                        _colorLayersDirty = true;
                    else
                        this->setDirty( true );
                    // remove from the list
                    i = _requests.erase( i );
                }
                else
                {
                    i++;
                }
            }
            else
            {
                //The color layer request failed, probably due to a server error.  Requeue it.
                r->setState( TaskRequest::STATE_IDLE );
                ++i;
            }
        }
        else if ( r->isCanceled() )
        {
            //Reset the cancelled task to IDLE and give it a new progress callback.
            i->get()->setState( TaskRequest::STATE_IDLE );
            i->get()->setProgressCallback( new TileRequestProgressCallback(
                i->get(), getVersionedTerrain()->getImageryTaskService(r->_layerId)));
            ++i;
        }
        else
        {
            ++i;
        }
    }

    // check the progress of the elevation data...
    if ( _hasElevation && !_elevationLayerUpToDate && _elevRequest.valid() && _elevPlaceholderRequest.valid() )
    {
        if ( _elevRequest->isCompleted() )
        {
            // if the elevation request succeeded, install the new elevation layer!
            TileElevationLayerRequest* er = static_cast<TileElevationLayerRequest*>( _elevRequest.get() );
            osgTerrain::HeightFieldLayer* hfLayer = static_cast<osgTerrain::HeightFieldLayer*>( er->getResult() );
            if ( hfLayer )
            {
                if (vt->canDirtyMoreTiles())
                {
                    // need to write-lock the layer data since we'll be changing it:
                    ScopedWriteLock lock( _tileLayersMutex );

                    // copy the skirt height over:
                    osg::HeightField* oldHF = static_cast<osgTerrain::HeightFieldLayer*>(getElevationLayer())->getHeightField();
                    if ( oldHF )
                        hfLayer->getHeightField()->setSkirtHeight( oldHF->getSkirtHeight() );

                    this->setElevationLayer( hfLayer );
                    if ( _usePerLayerUpdates )
                        _elevationLayerDirty = true;
                    else
                        this->setDirty( true );

                    _elevationLOD = er->_key->getLevelOfDetail();

                    //osg::notify(osg::NOTICE) << "Tile (" << _key->str() << ") final HF, LOD (" << _elevationLOD << ")" << std::endl;
                    _elevationLayerUpToDate = true;
                }
            }
            else
            {
                //Reque the elevation request.
                _elevRequest->setState( TaskRequest::STATE_IDLE );
            }
        }

        else if ( _elevRequest->isCanceled() )
        {
            // If the request was canceled, reset it to IDLE and reset the callback. On the next
            // servicePendingRequests, the request will be re-scheduled.
            _elevRequest->setState( TaskRequest::STATE_IDLE );
            _elevRequest->setProgressCallback( new TileRequestProgressCallback(
                _elevRequest.get(), getVersionedTerrain()->getElevationTaskService()));
        }

        else if ( _elevPlaceholderRequest->isCompleted() )
        {
            if (vt->canDirtyMoreTiles())
            {
                osgTerrain::HeightFieldLayer* newPhLayer = static_cast<osgTerrain::HeightFieldLayer*>(
                    _elevPlaceholderRequest->getResult() );

                // write-lock the layer data since we'll be changing it:
                ScopedWriteLock lock( _tileLayersMutex );

                if ( newPhLayer )
                {
                    this->setElevationLayer( newPhLayer );

                    if ( _usePerLayerUpdates )
                        _elevationLayerDirty = true;
                    else
                        this->setDirty( true );

                    _elevationLOD = _parentTile->getElevationLOD();
                }
                _elevPlaceholderRequest->setState( TaskRequest::STATE_IDLE );
            }
        }

        else if ( _elevPlaceholderRequest->isCanceled() )
        {
            _elevPlaceholderRequest->setState( TaskRequest::STATE_IDLE );
            _elevPlaceholderRequest->setProgressCallback( new TileRequestProgressCallback(
                _elevPlaceholderRequest.get(), getVersionedTerrain()->getElevationTaskService()));
        }
    }
}

void
VersionedTile::traverse( osg::NodeVisitor& nv )
{
    bool serviceRequests = _useLayerRequests && nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR;

    if ( serviceRequests )
    {
        serviceCompletedRequests();

        if ( getDirty() ) 
        {
            VersionedTerrain* vt = static_cast<VersionedTerrain*>(getTerrain());
            vt->incrementNumDirty();
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

void VersionedTile::releaseGLObjects(osg::State* state) const
{
    Group::releaseGLObjects(state);

    if (_terrainTechnique.valid())
    {
        _terrainTechnique->releaseGLObjects( state );
    }
}


/****************************************************************************/


VersionedTerrain::VersionedTerrain( Map* map, MapEngine* engine ) :
_map( map ),
_engine( engine ),
_revision(0),
_numAsyncThreads( 0 ),
_numDirty(0)
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

VersionedTile*
VersionedTerrain::getVersionedTile(const osgTerrain::TileID& tileID)
{
    ScopedLock<Mutex> lock(_mutex);

    TerrainTileMap::iterator itr = _terrainTileMap.find(tileID);
    if (itr == _terrainTileMap.end()) return 0;

    return static_cast<VersionedTile*>(itr->second);
}

VersionedTile*
VersionedTerrain::getVersionedTileNoLock(const osgTerrain::TileID& tileID)
{
    TerrainTileMap::iterator itr = _terrainTileMap.find(tileID);
    if (itr == _terrainTileMap.end()) return 0;
    return static_cast<VersionedTile*>(itr->second);
}

void
VersionedTerrain::refreshNeighbors( const osgTerrain::TileID& tileId, VersionedTileObsVec& output )
{
    ScopedLock<Mutex> lock(_mutex);

    // technically it should check for a geocentric rendering...
    bool wrap = _map->getProfile()->getSRS()->isGeographic() || _map->getProfile()->getSRS()->isMercator();

    unsigned int tilesX, tilesY;
    _profile->getNumTiles( tileId.level, tilesX, tilesY );

    unsigned int x, y;

    // west
    if ( !output[WEST].valid() && ( tileId.x > 0 || wrap ) )
    {
        x = tileId.x > 0? tileId.x-1 : tilesX-1;
        y = tileId.y;
        output[WEST] = getVersionedTileNoLock( osgTerrain::TileID( tileId.level, x, y ) );
    }

    // north
    if ( !output[NORTH].valid() && ( tileId.y < tilesY-1 || wrap ) )
    {
        x = tileId.x;
        y = tileId.y < tilesY-1 ? tileId.y+1 : 0;
        output[NORTH] = getVersionedTileNoLock( osgTerrain::TileID( tileId.level, x, y ) );
    }

    // east
    if ( !output[EAST].valid() && ( tileId.x < tilesX-1 || wrap ) )
    {
        x = tileId.x < tilesX-1 ? tileId.x+1 : 0;
        y = tileId.y;
        output[EAST] = getVersionedTileNoLock( osgTerrain::TileID( tileId.level, x, y ) );
    }

    // south
    if ( !output[SOUTH].valid() && ( tileId.y > 0 || wrap ) )
    {
        x = tileId.x;
        y = tileId.y > 0 ? tileId.y-1 : tilesY-1;
        output[SOUTH] = getVersionedTileNoLock( osgTerrain::TileID( tileId.level, x, y ) );
    }
}

void 
VersionedTerrain::getTerrainTiles( TerrainTileList& out_tiles )
{
    ScopedLock<Mutex> lock(_mutex);

    out_tiles.reserve( _terrainTileMap.size() );

    for(TerrainTileSet::iterator itr = _terrainTileSet.begin();
        itr != _terrainTileSet.end();
        ++itr)
    {
        out_tiles.push_back( (*itr) );
    }
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
        ScopedLock<Mutex> lock( _taskServiceMutex );
        for (TaskServiceMap::iterator i = _taskServices.begin(); i != _taskServices.end(); ++i)
        {
            i->second->setStamp( nv.getFrameStamp()->getFrameNumber() );
        }
    }
    else if (nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
    {
        _numDirty = 0;
    }
    osgTerrain::Terrain::traverse( nv );

    if (nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
    {
        //osg::notify(osg::NOTICE) << "Frame " << nv.getFrameStamp()->getFrameNumber() << ": updated " << _numDirty << std::endl;
    }
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
        getElevationTaskService()->setNumThreads( numElevationThreads );
    }

    for (MapLayerList::const_iterator itr = _map->getImageMapLayers().begin(); itr != _map->getImageMapLayers().end(); ++itr)
    {
        int imageThreads = (int)osg::round((float)_numAsyncThreads * (itr->get()->getLoadWeight() / totalWeight ));
        osg::notify(osg::NOTICE) << "ImageThreads for " << itr->get()->getName() << " = " << imageThreads << std::endl;
        getImageryTaskService( itr->get()->getId() )->setNumThreads( imageThreads );
    }

}

