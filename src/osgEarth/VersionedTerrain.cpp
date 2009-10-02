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
#include <OpenThreads/ScopedLock>
#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/Node>

using namespace osgEarth;
using namespace OpenThreads;


/*****************************************************************************/

struct TileLayerRequest : public TaskRequest
{
    TileLayerRequest( const TileKey* key, TileLayerFactory* factory )
        : _key( key ), _factory(factory) { }
    virtual bool isColorLayerRequest() const { return false; }
    virtual bool isElevLayerRequest() const { return false; }
    osg::ref_ptr<const TileKey> _key;
    osg::ref_ptr<TileLayerFactory> _factory;
};

struct TileColorLayerRequest : public TileLayerRequest
{
    TileColorLayerRequest( const TileKey* key, TileLayerFactory* factory, int layerIndex )
        : TileLayerRequest( key, factory ), _layerIndex( layerIndex ) { }
    bool isColorLayerRequest() const { return true; }
    void operator()( TaskProgress* p )
    {
        _result = _factory->createImageLayer( _key.get(), _layerIndex );
    }
    int _layerIndex;
};

struct TileElevationLayerRequest : public TileLayerRequest
{
    TileElevationLayerRequest( const TileKey* key, TileLayerFactory* factory )
        : TileLayerRequest( key, factory ) { }
    bool isElevLayerRequest() const { return true; }
    void operator()( TaskProgress* p )
    {
        _result = _factory->createHeightFieldLayer( _key.get() );
    }
};

/*****************************************************************************/


VersionedTile::VersionedTile( const TileKey* key ) :
_key( key ),
_useLayerRequests( false ),
_terrainRevision( -1 ),
_tileRevision( 0 ),
_requestsInstalled( false ),
_elevationLayerDirty( false ),
_colorLayersDirty( false ),
_usePerLayerUpdates( false )
{
    setTileID( key->getTileId() );
    setUseLayerRequests( false );
}

const TileKey*
VersionedTile::getKey() const {
    return _key.get();
}

VersionedTerrain*
VersionedTile::getVersionedTerrain() {
    return static_cast<VersionedTerrain*>(getTerrain());
}
const VersionedTerrain*
VersionedTile::getVersionedTerrain() const {
    return static_cast<const VersionedTerrain*>(getTerrain());
}

void
VersionedTile::setUseLayerRequests( bool value )
{
    _useLayerRequests = value;   

    // if layer requests are on, we need an update traversal.
    this->setNumChildrenRequiringUpdateTraversal( value? 1 : 0 );
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
    _requestElevation = hint;
}

void
VersionedTile::servicePendingRequests( int stamp )
{
    if ( !_requestsInstalled )
    {
        TileLayerFactory* factory = getVersionedTerrain()->getTileLayerFactory();
        if ( factory )
        {
            if ( this->getElevationLayer() && _requestElevation )
            {
                TileLayerRequest* r = new TileElevationLayerRequest( _key.get(), factory );
                r->setPriority( (float)_key->getLevelOfDetail() );
                r->setStamp( stamp );
                _requests.push_back( r );
            }
            for( int layerIndex=0; layerIndex<getNumColorLayers(); layerIndex++ )
            {
                TaskRequest* r = new TileColorLayerRequest( _key.get(), factory, layerIndex );
                r->setPriority( (float)_key->getLevelOfDetail() + (0.1 * (float)layerIndex) );
                r->setStamp( stamp );
                _requests.push_back( r );
            }
        }
        _requestsInstalled = true;
    }

    if ( _requestsInstalled )
    {
        for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); ++i )
        {
            TileLayerRequest* r = static_cast<TileLayerRequest*>( i->get() );

            if ( r->isIdle() )
            {
                r->setStamp( stamp );
                VersionedTerrain* versionedTerrain = static_cast<VersionedTerrain*>(getTerrain());
                if (versionedTerrain)
                {
                    versionedTerrain->getOrCreateTaskService()->add( r );
                }
            }
            else if ( !r->isCompleted() )
            {
                r->setStamp( stamp );
            }
        }
    }
}

void VersionedTile::serviceCompletedRequests()
{
    for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); )
    {
        TileLayerRequest* r = static_cast<TileLayerRequest*>( i->get() );

        if ( r->isCompleted() )
        {
            // TODO: merge new tile data into tile.
            if ( r->isElevLayerRequest() )
            {
                TileElevationLayerRequest* er = static_cast<TileElevationLayerRequest*>( r );
                osgTerrain::HeightFieldLayer* hfLayer = static_cast<osgTerrain::HeightFieldLayer*>( er->getResult() );
                if ( hfLayer )
                {
                    this->setElevationLayer( hfLayer );
                    if ( _usePerLayerUpdates )
                        _elevationLayerDirty = true;
                    else
                        this->setDirty( true );
                }
            }
            else // if ( r->isColorLayerRequest() )
            {
                TileColorLayerRequest* cr = static_cast<TileColorLayerRequest*>( r );
                osgTerrain::ImageLayer* imgLayer = static_cast<osgTerrain::ImageLayer*>( cr->getResult() );
                if ( imgLayer )
                {
                    this->setColorLayer( cr->_layerIndex, imgLayer );
                    if ( _usePerLayerUpdates )
                        _colorLayersDirty = true;
                    else
                        this->setDirty( true );
                }
            }

            // remove from the list
            i = _requests.erase( i );
        }
        else if ( r->isCanceled() )
        {
            i = _requests.erase( i );
        }
        else
        {
            i++;
        }
    }
}

void
VersionedTile::traverse( osg::NodeVisitor& nv )
{
    if ( _useLayerRequests )
    {
        //Service any completed requests.
        if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
        {
            serviceCompletedRequests();

            if ( !getDirty() ) // if the whole tile is dirty, ignore and let it rebuild.
            {
                if ( _elevationLayerDirty || _colorLayersDirty )
                {
                    EarthTerrainTechnique* tech = static_cast<EarthTerrainTechnique*>( getTerrainTechnique() );
                    tech->updateContent( _elevationLayerDirty, _colorLayersDirty );
                }
                _elevationLayerDirty = false;
                _colorLayersDirty = false;
            }
        }
    }
    osgTerrain::TerrainTile::traverse( nv );
}

/****************************************************************************/


VersionedTerrain::VersionedTerrain( TileLayerFactory* factory ) :
_layerFactory( factory ),
_revision(0),
_numTaskServiceThreads(8)
{
    const char* env_numTaskServiceThreads = getenv("OSGEARTH_NUM_TASK_SERVICE_THREADS");
    if ( env_numTaskServiceThreads )
    {
        _numTaskServiceThreads = ::atoi( env_numTaskServiceThreads );
        osg::notify(osg::NOTICE) << "osgEarth: task service threads = " << _numTaskServiceThreads << std::endl;
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

unsigned int 
VersionedTerrain::getNumTaskServiceThreads() const
{
    return _numTaskServiceThreads;
}

void
VersionedTerrain::setNumTaskServiceThreads( unsigned int numTaskServiceThreads )
{
    _numTaskServiceThreads = numTaskServiceThreads;
}

VersionedTile*
VersionedTerrain::getVersionedTile(const osgTerrain::TileID& tileID)
{
    ScopedLock<Mutex> lock(_mutex);

    TerrainTileMap::iterator itr = _terrainTileMap.find(tileID);
    if (itr == _terrainTileMap.end()) return 0;

    return static_cast<VersionedTile*>(itr->second);
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

TileLayerFactory*
VersionedTerrain::getTileLayerFactory() const {
    return _layerFactory.get();
}

TaskService*
VersionedTerrain::getOrCreateTaskService()
{
    if ( !_taskService.valid() )
    {
        ScopedLock<Mutex> lock( _taskServiceMutex );
        // double-check
        if ( !_taskService.valid() )
        {
            _taskService = new TaskService( _numTaskServiceThreads );
        }
    }    
    return _taskService.get();
}