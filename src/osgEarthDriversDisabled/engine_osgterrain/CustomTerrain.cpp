/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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
#if 0
#include "CustomTerrain"
#include "CustomTile"
#include "TransparentLayer"

#include <osgEarth/Registry>
#include <osgEarth/Map>
#include <osgEarth/FindNode>

#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/Node>
#include <osgGA/EventVisitor>

#include <OpenThreads/ScopedLock>


using namespace osgEarth;
using namespace OpenThreads;

#define LC "[CustomTerrain] "

// setting this will enable "fast GL object release" - the engine will activity
// track tiles that expire from the scene graph, and will explicity force them
// to deallocate their GL objects (instead of waiting for OSG to "lazily" 
// release them). This is helpful for freeing up memory more quickly when 
// aggresively navigating a map.
#define QUICK_RELEASE_GL_OBJECTS 1

//#define PREEMPTIVE_DEBUG 1

//----------------------------------------------------------------------------

namespace
{
    /**
     * A draw callback to calls another, nested draw callback.
     */
    struct NestingDrawCallback : public osg::Camera::DrawCallback
    {
        NestingDrawCallback( osg::Camera::DrawCallback* next ) : _next(next) { }

        virtual void operator()( osg::RenderInfo& renderInfo ) const
        {
            dispatch( renderInfo );
        }

        void dispatch( osg::RenderInfo& renderInfo ) const
        {
            if ( _next )
                _next->operator ()( renderInfo );
        }

        osg::ref_ptr<osg::Camera::DrawCallback> _next;
    };


    // a simple draw callback, to be installed on a Camera, that tells all CustomTerrains to
    // release GL memory on any expired tiles.
    struct QuickReleaseGLCallback : public NestingDrawCallback
    {
	    typedef std::vector< osg::observer_ptr< CustomTerrain > > ObserverTerrainList;

        QuickReleaseGLCallback( CustomTerrain* terrain, osg::Camera::DrawCallback* next )
            : NestingDrawCallback(next), _terrain(terrain) { }

        virtual void operator()( osg::RenderInfo& renderInfo ) const
        {
            osg::ref_ptr<CustomTerrain> terrainSafe = _terrain.get();
            if ( terrainSafe.valid() )
            {
                terrainSafe->releaseGLObjectsForTiles( renderInfo.getState() );
            }
            dispatch( renderInfo );
        }

        osg::observer_ptr<CustomTerrain> _terrain;
    };
}

//----------------------------------------------------------------------------

//TODO:  Register with the callback when we are first created...
// immediately release GL memory for any expired tiles.
// called from the DRAW thread
void
CustomTerrain::releaseGLObjectsForTiles(osg::State* state)
{
    OpenThreads::ScopedLock<Mutex> lock( _tilesToReleaseMutex );

    //if ( _tilesToRelease.size() > 0 )
    //{
    //    OE_INFO << "Releasing " << _tilesToRelease.size() << " tiles" << std::endl;
    //}

    while( _tilesToRelease.size() > 0 )
    {
        _tilesToRelease.front()->releaseGLObjects( state );
        _tilesToRelease.pop();
    }
}

CustomTerrain::CustomTerrain(const MapFrame& update_mapf, 
                             const MapFrame& cull_mapf, 
                             OSGTileFactory* tileFactory,
                             bool            quickReleaseGLObjects ) :
_revision(0),
_tileFactory( tileFactory ),
_numLoadingThreads( 0 ),
_registeredWithReleaseGLCallback( false ),
_update_mapf( update_mapf ),
_cull_mapf( cull_mapf ),
_onDemandDelay( 2 ),
_quickReleaseGLObjects( quickReleaseGLObjects ),
_quickReleaseCallbackInstalled( false ),
_alwaysUpdate( false )
{
    this->setThreadSafeRefUnref( true );

    _loadingPolicy = _tileFactory->getTerrainOptions().loadingPolicy().get();

    if ( _loadingPolicy.mode() != LoadingPolicy::MODE_SERIAL && _loadingPolicy.mode() != LoadingPolicy::MODE_PARALLEL )
    {
        setNumChildrenRequiringUpdateTraversal( 1 );
        _alwaysUpdate = true;
        _numLoadingThreads = computeLoadingThreads(_loadingPolicy);
        OE_INFO << LC << "Using a total of " << _numLoadingThreads << " loading threads " << std::endl;
    }
    else
    {        
        // undo the setting in osgTerrain::Terrain
        //setNumChildrenRequiringUpdateTraversal( 0 );

        // the EVENT_VISITOR will reset this to 0 once the "delay" is expired.
        _alwaysUpdate = false;
        setNumChildrenRequiringUpdateTraversal( 1 );
    }

    // register for events in order to support ON_DEMAND frame scheme
    setNumChildrenRequiringEventTraversal( 1 );
}

CustomTerrain::~CustomTerrain()
{
    //nop
}

void
CustomTerrain::incrementRevision()
{
    // no need to lock; if we miss it, we'll get it the next time around
    _revision++;
}

int
CustomTerrain::getRevision() const
{
    // no need to lock; if we miss it, we'll get it the next time around
    return _revision;
}

void
CustomTerrain::getCustomTile(const osgTerrain::TileID& tileID,
                             osg::ref_ptr<CustomTile>& out_tile,
                             bool lock )
{
    if ( lock )
    {
        Threading::ScopedReadLock lock( _tilesMutex );
        TileTable::iterator i = _tiles.find( tileID );
        out_tile = i != _tiles.end()? i->second.get() : 0L;
    }
    else
    {
        TileTable::iterator i = _tiles.find( tileID );
        out_tile = i != _tiles.end()? i->second.get() : 0L;
    }
}

void
CustomTerrain::getCustomTiles( TileVector& out )
{
    Threading::ScopedReadLock lock( _tilesMutex );
    out.clear();
    out.reserve( _tiles.size() );
    for( TileTable::const_iterator i = _tiles.begin(); i != _tiles.end(); ++i )
        out.push_back( i->second.get() );
}

const LoadingPolicy&
CustomTerrain::getLoadingPolicy() const
{
    return _loadingPolicy;
}

// This method is called by CustomTerrain::traverse() in the UPDATE TRAVERSAL.
void
CustomTerrain::refreshFamily(const MapInfo& mapInfo,
                             //const osgTerrain::TileID& tileId,
                             const TileKey& key,
                             Relative* family,
                             bool tileTableLocked )
{
    osgTerrain::TileID tileId = key.getTileId();

    // geocentric maps wrap around in the X dimension.
    bool wrapX = mapInfo.isGeocentric();
    unsigned int tileCountX, tileCountY;
    mapInfo.getProfile()->getNumTiles( tileId.level, tileCountX, tileCountY );

    // Relative::PARENT
    {
        family[Relative::PARENT].expected = true; // TODO: is this always correct?
        family[Relative::PARENT].elevLOD = -1;
        family[Relative::PARENT].imageLODs.clear();
        family[Relative::PARENT].tileID = osgTerrain::TileID( tileId.level-1, tileId.x/2, tileId.y/2 );

        osg::ref_ptr<CustomTile> parent;
        getCustomTile( family[Relative::PARENT].tileID, parent, !tileTableLocked );
        if ( parent.valid() )
        {
            family[Relative::PARENT].elevLOD = parent->getElevationLOD();

            ColorLayersByUID relLayers;
            parent->getCustomColorLayers( relLayers );

            for( ColorLayersByUID::const_iterator i = relLayers.begin(); i != relLayers.end(); ++i )
            {
                family[Relative::PARENT].imageLODs[i->first] = i->second.getLevelOfDetail();
            }
        }
    }

    // Relative::WEST
    {
        family[Relative::WEST].expected = tileId.x > 0 || wrapX;
        family[Relative::WEST].elevLOD = -1;
        family[Relative::WEST].imageLODs.clear();
        family[Relative::WEST].tileID = osgTerrain::TileID( tileId.level, tileId.x > 0? tileId.x-1 : tileCountX-1, tileId.y );
        osg::ref_ptr<CustomTile> west;
        getCustomTile( family[Relative::WEST].tileID, west, !tileTableLocked );
        if ( west.valid() )
        {
            family[Relative::WEST].elevLOD = west->getElevationLOD();

            ColorLayersByUID relLayers;
            west->getCustomColorLayers( relLayers );

            for( ColorLayersByUID::const_iterator i = relLayers.begin(); i != relLayers.end(); ++i )
            {
                family[Relative::WEST].imageLODs[i->first] = i->second.getLevelOfDetail();
            }
        }
    }

    // Relative::NORTH
    {
        family[Relative::NORTH].expected = tileId.y < (int)tileCountY-1;
        family[Relative::NORTH].elevLOD = -1;
        family[Relative::NORTH].imageLODs.clear();
        family[Relative::NORTH].tileID = osgTerrain::TileID( tileId.level, tileId.x, tileId.y < (int)tileCountY-1 ? tileId.y+1 : 0 );
        osg::ref_ptr<CustomTile> north;
        getCustomTile( family[Relative::NORTH].tileID, north, !tileTableLocked );
        if ( north.valid() )
        {
            family[Relative::NORTH].elevLOD = north->getElevationLOD();

            ColorLayersByUID relLayers;
            north->getCustomColorLayers( relLayers );

            for( ColorLayersByUID::const_iterator i = relLayers.begin(); i != relLayers.end(); ++i )
            {
                family[Relative::NORTH].imageLODs[i->first] = i->second.getLevelOfDetail();
            }
        }
    }

    // Relative::EAST
    {
        family[Relative::EAST].expected = tileId.x < (int)tileCountX-1 || wrapX;
        family[Relative::EAST].elevLOD = -1;
        family[Relative::EAST].imageLODs.clear();
        family[Relative::EAST].tileID = osgTerrain::TileID( tileId.level, tileId.x < (int)tileCountX-1 ? tileId.x+1 : 0, tileId.y );
        osg::ref_ptr<CustomTile> east;
        getCustomTile( family[Relative::EAST].tileID, east, !tileTableLocked );
        if ( east.valid() )
        {
            family[Relative::EAST].elevLOD = east->getElevationLOD();

            ColorLayersByUID relLayers;
            east->getCustomColorLayers( relLayers );

            for( ColorLayersByUID::const_iterator i = relLayers.begin(); i != relLayers.end(); ++i )
            {
                family[Relative::EAST].imageLODs[i->first] = i->second.getLevelOfDetail();
            }
        }
    }

    // Relative::SOUTH
    {
        family[Relative::SOUTH].expected = tileId.y > 0;
        family[Relative::SOUTH].elevLOD = -1;
        family[Relative::SOUTH].imageLODs.clear();
        family[Relative::SOUTH].tileID = osgTerrain::TileID( tileId.level, tileId.x, tileId.y > 0 ? tileId.y-1 : tileCountY-1 );
        osg::ref_ptr<CustomTile> south;
        getCustomTile( family[Relative::SOUTH].tileID, south, !tileTableLocked );
        if ( south.valid() )
        {
            family[Relative::SOUTH].elevLOD = south->getElevationLOD();

            ColorLayersByUID relLayers;
            south->getCustomColorLayers( relLayers );

            for( ColorLayersByUID::const_iterator i = relLayers.begin(); i != relLayers.end(); ++i )
            {
                family[Relative::SOUTH].imageLODs[i->first] = i->second.getLevelOfDetail();
            }
        }
    }
}

OSGTileFactory*
CustomTerrain::getTileFactory() {
    return _tileFactory.get();
}

#if 0
void
CustomTerrain::addTerrainCallback( TerrainCallback* cb )
{
    _terrainCallbacks.push_back( cb );
}
#endif

void
CustomTerrain::registerTile( CustomTile* newTile )
{
    Threading::ScopedWriteLock exclusiveTileTableLock( _tilesMutex );
    _tiles[ newTile->getTileID() ] = newTile;
    //OE_INFO << LC << "Registered tiles = " << _tiles.size() << std::endl;
}

unsigned int
CustomTerrain::getNumTasksRemaining() const
{
    ScopedLock<Mutex> lock(const_cast<CustomTerrain*>(this)->_taskServiceMutex );
    unsigned int total = 0;
    for (TaskServiceMap::const_iterator itr = _taskServices.begin(); itr != _taskServices.end(); ++itr)
    {
        total += itr->second->getNumRequests();
    }
    return total;
}

void
CustomTerrain::traverse( osg::NodeVisitor &nv )
{
    // UPDATE runs whenever a Tile runs its update traversal on the first pass.
    // i.e., only runs then a new Tile is born.
    if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
    {
        // if the terrain engine requested "quick release", install the quick release
        // draw callback now.
        if ( _quickReleaseGLObjects && !_quickReleaseCallbackInstalled )
        {
            osg::Camera* cam = findFirstParentOfType<osg::Camera>( this );
            if ( cam )
            {
                cam->setPostDrawCallback( new QuickReleaseGLCallback( this, cam->getPostDrawCallback() ) );
                _quickReleaseCallbackInstalled = true;
                OE_INFO << LC << "Quick release enabled" << std::endl;
            }
        }

        // this stamp keeps track of when requests are dispatched. If a request's stamp gets too
        // old, it is considered "expired" and subject to cancelation
        int stamp = nv.getFrameStamp()->getFrameNumber();

        // Collect any "dead" tiles and queue them for shutdown.
        {
            Threading::ScopedWriteLock tileTableExclusiveLock( _tilesMutex );

            for( TileTable::iterator i = _tiles.begin(); i != _tiles.end(); )
            {
                CustomTile* tile = i->second.get();
                if ( tile->getNumParents() == 0 && tile->getHasBeenTraversed() )
                {
                    _tilesToShutDown.push_back( tile );
                    
                    // i is incremented prior to calling erase, but i's previous value goes to erase,
                    // maintaining validity
                    _tiles.erase( i++ );
                }
                else
                    ++i;

            }
        }

        // Remove any dead tiles from the main tile table, while at the same time queuing 
        // any tiles that require quick-release. This criticial section requires an exclusive
        // lock on the main tile table.
        {
            Threading::ScopedMutexLock tilesToReleaseExclusiveLock( _tilesToReleaseMutex );

            // Shut down any dead tiles once there tasks are complete.
            for( TileList::iterator i = _tilesToShutDown.begin(); i != _tilesToShutDown.end(); )
            {
                CustomTile* tile = i->get();
                if ( tile && tile->cancelRequests() )
                {
                    if ( _quickReleaseGLObjects && _quickReleaseCallbackInstalled )
                    {
                        _tilesToRelease.push( tile );
                    }

                    i = _tilesToShutDown.erase( i );
                }
                else
                    ++i;
            }
        }

//        OE_NOTICE << "Tiles = " << _tiles.size() << std::endl;

        if ( _loadingPolicy.mode() == LoadingPolicy::MODE_SEQUENTIAL || _loadingPolicy.mode() == LoadingPolicy::MODE_PREEMPTIVE )
        {
            // update the frame stamp on the task services. This is necessary to support 
            // automatic request cancelation for image requests.
            {
                ScopedLock<Mutex> lock( _taskServiceMutex );
                for (TaskServiceMap::iterator i = _taskServices.begin(); i != _taskServices.end(); ++i)
                {
                    i->second->setStamp( stamp );
                }
            }

            // next, go through the live tiles and process update-traversal requests. This
            // requires a read-lock on the master tiles table.
            TileList updatedTiles;
            {
                Threading::ScopedReadLock tileTableReadLock( _tilesMutex );

                for( TileTable::const_iterator i = _tiles.begin(); i != _tiles.end(); ++i )
                {
                    CustomTile* tile = i->second.get();

                    // update the neighbor list for each tile.
                    refreshFamily( _update_mapf.getMapInfo(), tile->getKey(), tile->getFamily(), true );

                    if ( tile->getUseLayerRequests() ) // i.e., sequential or preemptive mode
                    {
                        tile->servicePendingElevationRequests( _update_mapf, stamp, true );                   
                        tile->serviceCompletedRequests( _update_mapf, true );
                        //if ( tileModified && _terrainCallbacks.size() > 0 )
                        //{
                        //    updatedTiles.push_back( tile );
                        //}
                    }
                }
            }
        }

#if 0
        // finally, notify listeners of tile modifications.
        if ( updatedTiles.size() > 0 )
        {
            for( TerrainCallbackList::iterator n = _terrainCallbacks.begin(); n != _terrainCallbacks.end(); ++n )
            {
                n->get()->onTerrainTilesUpdated( updatedTiles );
            }
        }
#endif
    }

    else if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
#if 0
        // check each terrain tile for requests (if not in standard mode)
        if ( _loadingPolicy.mode() != LoadingPolicy::MODE_STANDARD )
        {
            int frameStamp = nv.getFrameStamp()->getFrameNumber();

            // make a thread-safe copy of the tile table
            CustomTileVector tiles;
            getCustomTiles( tiles );

            for( CustomTileVector::iterator i = tiles.begin(); i != tiles.end(); ++i )
            {
                CustomTile* tile = i->get();
                tile->servicePendingImageRequests( _cull_mapf, frameStamp );
            }
        }
#endif
    } 
    
    else if ( nv.getVisitorType() == osg::NodeVisitor::EVENT_VISITOR )
    {
        // in OSG's "ON_DEMAND" frame scheme, OSG runs the event visitor as part of the
        // test to see if a frame is needed. In sequential/preemptive mode, we need to 
        // check whether there are any pending tasks running. 

        // In addition, once the tasks run out, we continue to delay on-demand rendering
        // for another full frame so that the event dispatchers can catch up.

        if ( _tilesToShutDown.size() > 0 )
        {
            setDelay( 2 );
        }

        else if ( _onDemandDelay <= 0 )
        {
            int numTasks = getNumTasksRemaining();
            if ( numTasks > 0 )
            {
                setDelay( 2 );
            }
        }

        //OE_INFO << "Tasks = " << numTasks << std::endl;

        if ( _onDemandDelay > 0 )
        {
            osgGA::EventVisitor* ev = dynamic_cast<osgGA::EventVisitor*>( &nv );
            ev->getActionAdapter()->requestRedraw();
            decDelay();
        }
    }

    osgTerrain::Terrain::traverse( nv );
}

void
CustomTerrain::setDelay( unsigned frames )
{
    if ( _onDemandDelay == 0 && !_alwaysUpdate )
    {
        ADJUST_UPDATE_TRAV_COUNT( this, 1 );
    }
    _onDemandDelay = frames;
}

void
CustomTerrain::decDelay()
{
    _onDemandDelay--;
    if ( _onDemandDelay == 0 && !_alwaysUpdate )
    {
        ADJUST_UPDATE_TRAV_COUNT(this, -1);
    }
}

TaskService*
CustomTerrain::createTaskService( const std::string& name, int id, int numThreads )
{
    ScopedLock<Mutex> lock( _taskServiceMutex );

    // first, double-check that the service wasn't created during the locking process:
    TaskServiceMap::iterator itr = _taskServices.find(id);
    if (itr != _taskServices.end())
        return itr->second.get();

    // ok, make a new one
    TaskService* service =  new TaskService( name, numThreads );
    _taskServices[id] = service;
    return service;
}

TaskService*
CustomTerrain::getTaskService(int id)
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
#define TILE_GENERATION_TASK_SERVICE_ID 10000

TaskService*
CustomTerrain::getElevationTaskService()
{
    TaskService* service = getTaskService( ELEVATION_TASK_SERVICE_ID );
    if (!service)
    {
        service = createTaskService( "elevation", ELEVATION_TASK_SERVICE_ID, 1 );
    }
    return service;
}


TaskService*
CustomTerrain::getImageryTaskService(int layerId)
{
    TaskService* service = getTaskService( layerId );
    if (!service)
    {
        std::stringstream buf;
        buf << "layer " << layerId;
        std::string bufStr = buf.str();
        service = createTaskService( bufStr, layerId, 1 );
    }
    return service;
}

TaskService*
CustomTerrain::getTileGenerationTaskSerivce()
{
    TaskService* service = getTaskService( TILE_GENERATION_TASK_SERVICE_ID );
    if (!service)
    {
        int numCompileThreads = 
            _loadingPolicy.numCompileThreads().isSet() ? osg::maximum( 1, _loadingPolicy.numCompileThreads().value() ) :
            (int)osg::maximum( 1.0f, _loadingPolicy.numCompileThreadsPerCore().value() * (float)GetNumberOfProcessors() );

        service = createTaskService( "tilegen", TILE_GENERATION_TASK_SERVICE_ID, numCompileThreads );
    }
    return service;
}

void
CustomTerrain::updateTaskServiceThreads( const MapFrame& mapf )
{
    //Get the maximum elevation weight
    float elevationWeight = 0.0f;
    for (ElevationLayerVector::const_iterator itr = mapf.elevationLayers().begin(); itr != mapf.elevationLayers().end(); ++itr)
    {
        ElevationLayer* layer = itr->get();
        float w = layer->getTerrainLayerOptions().loadingWeight().value();
        if (w > elevationWeight) elevationWeight = w;
    }

    float totalImageWeight = 0.0f;
    for (ImageLayerVector::const_iterator itr = mapf.imageLayers().begin(); itr != mapf.imageLayers().end(); ++itr)
    {
        totalImageWeight += itr->get()->getTerrainLayerOptions().loadingWeight().value();
    }

    float totalWeight = elevationWeight + totalImageWeight;

    if (elevationWeight > 0.0f)
    {
        //Determine how many threads each layer gets
        int numElevationThreads = (int)osg::round((float)_numLoadingThreads * (elevationWeight / totalWeight ));
        OE_INFO << LC << "Elevation Threads = " << numElevationThreads << std::endl;
        getElevationTaskService()->setNumThreads( numElevationThreads );
    }

    for (ImageLayerVector::const_iterator itr = mapf.imageLayers().begin(); itr != mapf.imageLayers().end(); ++itr)
    {
        const TerrainLayerOptions& opt = itr->get()->getTerrainLayerOptions();
        int imageThreads = (int)osg::round((float)_numLoadingThreads * (opt.loadingWeight().value() / totalWeight ));
        OE_INFO << LC << "Image Threads for " << itr->get()->getName() << " = " << imageThreads << std::endl;
        getImageryTaskService( itr->get()->getUID() )->setNumThreads( imageThreads );
    }
}
#endif