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
#include "StreamingTerrainNode"
#include "StreamingTile"
#include "TransparentLayer"

#include <osgEarth/Registry>
#include <osgEarth/MapFrame>
#include <osgEarth/MapInfo>
#include <osgEarth/NodeUtils>

#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/Node>
#include <osgGA/EventVisitor>

#include <OpenThreads/ScopedLock>

using namespace osgEarth_engine_osgterrain;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[StreamingTerrainNode] "

//----------------------------------------------------------------------------

StreamingTerrainNode::StreamingTerrainNode(const MapFrame& update_mapf, 
                                           const MapFrame& cull_mapf, 
                                           OSGTileFactory* tileFactory,
                                           bool            quickReleaseGLObjects ) :

TerrainNode( update_mapf, cull_mapf, tileFactory, quickReleaseGLObjects ),
_numLoadingThreads( 0 )
{
    _loadingPolicy = tileFactory->getTerrainOptions().loadingPolicy().get();

    setNumChildrenRequiringUpdateTraversal( 1 );
    _alwaysUpdate = true;
    _numLoadingThreads = computeLoadingThreads(_loadingPolicy);

    OE_INFO << LC << "Using a total of " << _numLoadingThreads << " loading threads " << std::endl;
}

StreamingTerrainNode::~StreamingTerrainNode()
{
    //nop
}

Tile*
StreamingTerrainNode::createTile(const TileKey& key, GeoLocator* locator) const
{
    return new StreamingTile( key, locator, this->getQuickReleaseGLObjects() );
}

// This method is called by StreamingTerrainNode::traverse() in the UPDATE TRAVERSAL.
void
StreamingTerrainNode::refreshFamily(const MapInfo&           mapInfo,
                                    const TileKey&           key,
                                    StreamingTile::Relative* family,
                                    bool                     tileTableLocked )
{
    osgTerrain::TileID tileId = key.getTileId();

    // geocentric maps wrap around in the X dimension.
    bool wrapX = mapInfo.isGeocentric();
    unsigned int tileCountX, tileCountY;
    mapInfo.getProfile()->getNumTiles( tileId.level, tileCountX, tileCountY );

    // Relative::PARENT
    {
        family[StreamingTile::Relative::PARENT].expected = true; // TODO: is this always correct?
        family[StreamingTile::Relative::PARENT].elevLOD = -1;
        family[StreamingTile::Relative::PARENT].imageLODs.clear();
        family[StreamingTile::Relative::PARENT].tileID = osgTerrain::TileID( tileId.level-1, tileId.x/2, tileId.y/2 );

        osg::ref_ptr<StreamingTile> parent;
        getTile( family[StreamingTile::Relative::PARENT].tileID, parent, !tileTableLocked );
        if ( parent.valid() )
        {
            family[StreamingTile::Relative::PARENT].elevLOD = parent->getElevationLOD();

            ColorLayersByUID relLayers;
            parent->getCustomColorLayers( relLayers );

            for( ColorLayersByUID::const_iterator i = relLayers.begin(); i != relLayers.end(); ++i )
            {
                family[StreamingTile::Relative::PARENT].imageLODs[i->first] = i->second.getLevelOfDetail();
            }
        }
    }

    // Relative::WEST
    {
        family[StreamingTile::Relative::WEST].expected = tileId.x > 0 || wrapX;
        family[StreamingTile::Relative::WEST].elevLOD = -1;
        family[StreamingTile::Relative::WEST].imageLODs.clear();
        family[StreamingTile::Relative::WEST].tileID = osgTerrain::TileID( tileId.level, tileId.x > 0? tileId.x-1 : tileCountX-1, tileId.y );
        osg::ref_ptr<StreamingTile> west;
        getTile( family[StreamingTile::Relative::WEST].tileID, west, !tileTableLocked );
        if ( west.valid() )
        {
            family[StreamingTile::Relative::WEST].elevLOD = west->getElevationLOD();

            ColorLayersByUID relLayers;
            west->getCustomColorLayers( relLayers );

            for( ColorLayersByUID::const_iterator i = relLayers.begin(); i != relLayers.end(); ++i )
            {
                family[StreamingTile::Relative::WEST].imageLODs[i->first] = i->second.getLevelOfDetail();
            }
        }
    }

    // Relative::NORTH
    {
        family[StreamingTile::Relative::NORTH].expected = tileId.y < (int)tileCountY-1;
        family[StreamingTile::Relative::NORTH].elevLOD = -1;
        family[StreamingTile::Relative::NORTH].imageLODs.clear();
        family[StreamingTile::Relative::NORTH].tileID = osgTerrain::TileID( tileId.level, tileId.x, tileId.y < (int)tileCountY-1 ? tileId.y+1 : 0 );
        osg::ref_ptr<StreamingTile> north;
        getTile( family[StreamingTile::Relative::NORTH].tileID, north, !tileTableLocked );
        if ( north.valid() )
        {
            family[StreamingTile::Relative::NORTH].elevLOD = north->getElevationLOD();

            ColorLayersByUID relLayers;
            north->getCustomColorLayers( relLayers );

            for( ColorLayersByUID::const_iterator i = relLayers.begin(); i != relLayers.end(); ++i )
            {
                family[StreamingTile::Relative::NORTH].imageLODs[i->first] = i->second.getLevelOfDetail();
            }
        }
    }

    // Relative::EAST
    {
        family[StreamingTile::Relative::EAST].expected = tileId.x < (int)tileCountX-1 || wrapX;
        family[StreamingTile::Relative::EAST].elevLOD = -1;
        family[StreamingTile::Relative::EAST].imageLODs.clear();
        family[StreamingTile::Relative::EAST].tileID = osgTerrain::TileID( tileId.level, tileId.x < (int)tileCountX-1 ? tileId.x+1 : 0, tileId.y );
        osg::ref_ptr<StreamingTile> east;
        getTile( family[StreamingTile::Relative::EAST].tileID, east, !tileTableLocked );
        if ( east.valid() )
        {
            family[StreamingTile::Relative::EAST].elevLOD = east->getElevationLOD();

            ColorLayersByUID relLayers;
            east->getCustomColorLayers( relLayers );

            for( ColorLayersByUID::const_iterator i = relLayers.begin(); i != relLayers.end(); ++i )
            {
                family[StreamingTile::Relative::EAST].imageLODs[i->first] = i->second.getLevelOfDetail();
            }
        }
    }

    // Relative::SOUTH
    {
        family[StreamingTile::Relative::SOUTH].expected = tileId.y > 0;
        family[StreamingTile::Relative::SOUTH].elevLOD = -1;
        family[StreamingTile::Relative::SOUTH].imageLODs.clear();
        family[StreamingTile::Relative::SOUTH].tileID = osgTerrain::TileID( tileId.level, tileId.x, tileId.y > 0 ? tileId.y-1 : tileCountY-1 );
        osg::ref_ptr<StreamingTile> south;
        getTile( family[StreamingTile::Relative::SOUTH].tileID, south, !tileTableLocked );
        if ( south.valid() )
        {
            family[StreamingTile::Relative::SOUTH].elevLOD = south->getElevationLOD();

            ColorLayersByUID relLayers;
            south->getCustomColorLayers( relLayers );

            for( ColorLayersByUID::const_iterator i = relLayers.begin(); i != relLayers.end(); ++i )
            {
                family[StreamingTile::Relative::SOUTH].imageLODs[i->first] = i->second.getLevelOfDetail();
            }
        }
    }
}

unsigned
StreamingTerrainNode::getNumActiveTasks() const
{
    ScopedLock<Mutex> lock(const_cast<StreamingTerrainNode*>(this)->_taskServiceMutex );

    unsigned int total = 0;
    for (TaskServiceMap::const_iterator itr = _taskServices.begin(); itr != _taskServices.end(); ++itr)
    {
        total += itr->second->getNumRequests();
    }
    return total;
}

void
StreamingTerrainNode::updateTraversal( osg::NodeVisitor& nv )
{
    // this stamp keeps track of when requests are dispatched. If a request's stamp gets too
    // old, it is considered "expired" and subject to cancelation
    int stamp = nv.getFrameStamp()->getFrameNumber();

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
    {
        Threading::ScopedReadLock tileTableReadLock( _tilesMutex );

        for( TileTable::const_iterator i = _tiles.begin(); i != _tiles.end(); ++i )
        {
            StreamingTile* tile = static_cast<StreamingTile*>( i->second.get() );

            // update the neighbor list for each tile.
            refreshFamily( _update_mapf.getMapInfo(), tile->getKey(), tile->getFamily(), true );

            tile->servicePendingElevationRequests( _update_mapf, stamp, true );                   
            tile->serviceCompletedRequests( _update_mapf, true );
        }
    }
}

TaskService*
StreamingTerrainNode::createTaskService( const std::string& name, int id, int numThreads )
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
StreamingTerrainNode::getTaskService(int id)
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
StreamingTerrainNode::getElevationTaskService()
{
    TaskService* service = getTaskService( ELEVATION_TASK_SERVICE_ID );
    if (!service)
    {
        service = createTaskService( "elevation", ELEVATION_TASK_SERVICE_ID, 1 );
    }
    return service;
}


TaskService*
StreamingTerrainNode::getImageryTaskService(int layerId)
{
    TaskService* service = getTaskService( layerId );
    if (!service)
    {
        std::stringstream buf;
        buf << "layer " << layerId;
        std::string bufStr;
        bufStr = buf.str();
        service = createTaskService( bufStr, layerId, 1 );
    }
    return service;
}

TaskService*
StreamingTerrainNode::getTileGenerationTaskService()
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
StreamingTerrainNode::updateTaskServiceThreads( const MapFrame& mapf )
{
    //Get the maximum elevation weight
    float elevationWeight = 0.0f;
    for (ElevationLayerVector::const_iterator itr = mapf.elevationLayers().begin(); itr != mapf.elevationLayers().end(); ++itr)
    {
        ElevationLayer* layer = itr->get();
        float w = layer->getElevationLayerOptions().loadingWeight().value();
        if (w > elevationWeight) elevationWeight = w;
    }

    float totalImageWeight = 0.0f;
    for (ImageLayerVector::const_iterator itr = mapf.imageLayers().begin(); itr != mapf.imageLayers().end(); ++itr)
    {
        totalImageWeight += itr->get()->getImageLayerOptions().loadingWeight().value();
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
        const TerrainLayerOptions& opt = itr->get()->getImageLayerOptions();
        int imageThreads = (int)osg::round((float)_numLoadingThreads * (opt.loadingWeight().value() / totalWeight ));
        OE_INFO << LC << "Image Threads for " << itr->get()->getName() << " = " << imageThreads << std::endl;
        getImageryTaskService( itr->get()->getUID() )->setNumThreads( imageThreads );
    }
}
