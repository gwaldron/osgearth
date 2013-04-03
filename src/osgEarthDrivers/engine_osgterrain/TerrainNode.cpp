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
#include "TerrainNode"
#include "Tile"
#include "TransparentLayer"

#include <osgEarth/Registry>
#include <osgEarth/Map>
#include <osgEarth/NodeUtils>
#include <osgEarth/ThreadingUtils>

#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/Node>
#include <osgGA/EventVisitor>

using namespace osgEarth_engine_osgterrain;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[Terrain] "

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


    // a simple draw callback, to be installed on a Camera, that tells all Terrains to
    // release GL memory on any expired tiles.
    struct QuickReleaseGLCallback : public NestingDrawCallback
    {
	    typedef std::vector< osg::observer_ptr<TerrainNode> > ObserverTerrainList;

        QuickReleaseGLCallback( TerrainNode* terrain, osg::Camera::DrawCallback* next )
            : NestingDrawCallback(next), _terrain(terrain) { }

        virtual void operator()( osg::RenderInfo& renderInfo ) const
        {
            osg::ref_ptr<TerrainNode> terrainSafe = _terrain.get();
            if ( terrainSafe.valid() )
            {
                terrainSafe->releaseGLObjectsForTiles( renderInfo.getState() );
            }
            dispatch( renderInfo );
        }

        osg::observer_ptr<TerrainNode> _terrain;
    };
}

//----------------------------------------------------------------------------

TerrainNode::TerrainNode(const MapFrame& update_mapf, 
                         const MapFrame& cull_mapf, 
                         OSGTileFactory* tileFactory,
                         bool            quickReleaseGLObjects ) :

_tileFactory( tileFactory ),
_registeredWithReleaseGLCallback( false ),
_update_mapf( update_mapf ),
_cull_mapf( cull_mapf ),
_onDemandDelay( 2 ),
_quickReleaseGLObjects( quickReleaseGLObjects ),
_quickReleaseCallbackInstalled( false ),
_alwaysUpdate( false ),
_sampleRatio( 1.0f ),
_verticalScale( 1.0f )
{
    this->setThreadSafeRefUnref( true );

    // the EVENT_VISITOR will reset this to 0 once the "delay" is expired.
    _alwaysUpdate = false;
    setNumChildrenRequiringUpdateTraversal( 1 );

    // register for events in order to support ON_DEMAND frame scheme
    setNumChildrenRequiringEventTraversal( 1 );    
}

TerrainNode::~TerrainNode()
{
    // detach all the tiles from the terrain first. Otherwise osgTerrainNode::Terrain
    // will crap out.
    for( TileTable::iterator i = _tiles.begin(); i != _tiles.end(); ++i )
    {
        i->second->attachToTerrain( 0L );
    }
    _tiles.clear();
}

void
TerrainNode::setTechniquePrototype( TerrainTechnique* value )
{
    _techPrototype = value;
}

TerrainTechnique*
TerrainNode::cloneTechnique() const
{
    return osg::clone( _techPrototype.get(), osg::CopyOp::DEEP_COPY_ALL );
}

Tile*
TerrainNode::createTile(const TileKey& key, GeoLocator* keyLocator) const
{
    return new Tile( key, keyLocator, this->getQuickReleaseGLObjects() );
}

void
TerrainNode::setVerticalScale( float value )
{
    if ( value != _verticalScale )
    {
        _verticalScale = value;
    }
}

void
TerrainNode::setSampleRatio( float value )
{
    if ( value != _sampleRatio )
    {
        _sampleRatio = value;
    }
}

void
TerrainNode::getTiles( TileVector& out )
{
    Threading::ScopedReadLock lock( _tilesMutex );
    out.clear();
    out.reserve( _tiles.size() );
    for( TileTable::const_iterator i = _tiles.begin(); i != _tiles.end(); ++i )
        out.push_back( i->second.get() );
}

void
TerrainNode::registerTile( Tile* newTile )
{
    Threading::ScopedWriteLock exclusiveTileTableLock( _tilesMutex );
    _tiles[ newTile->getTileId() ] = newTile;
}

// immediately release GL memory for any expired tiles.
// called from the DRAW thread (QuickReleaseGLCallback). 
void
TerrainNode::releaseGLObjectsForTiles(osg::State* state)
{
    OpenThreads::ScopedLock<Mutex> lock( _tilesToReleaseMutex );

    for( TileVector::iterator i = _tilesToRelease.begin(); i != _tilesToRelease.end(); ++i )
    {
        i->get()->releaseGLObjects( state );
    }

    _tilesToRelease.clear();

    //while( _tilesToRelease.size() > 0 )
    //{
    //    _tilesToRelease.front()->releaseGLObjects( state );
    //    _tilesToRelease.pop();
    //}
}

void
TerrainNode::traverse( osg::NodeVisitor &nv )
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

        // Collect any "dead" tiles and queue them for shutdown. Since UPDATE only runs
        // when new tiles arrive, this clears out old tiles from the queue at that time.
        // Another approach might be to use an observer_ptr instead...but then we may
        // not be able to use the quick-release.
        {
            Threading::ScopedWriteLock tileTableExclusiveLock( _tilesMutex );

            for( TileTable::iterator i = _tiles.begin(); i != _tiles.end(); )
            {
                Tile* tile = i->second.get();
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
        if ( _tilesToShutDown.size() > 0 ) // quick no-lock check..
        {
            Threading::ScopedMutexLock tilesToReleaseExclusiveLock( _tilesToReleaseMutex );

            // Shut down any dead tiles once there tasks are complete.
            for( TileList::iterator i = _tilesToShutDown.begin(); i != _tilesToShutDown.end(); )
            {
                Tile* tile = i->get();
                if ( tile && tile->cancelActiveTasks() )
                {
                    if ( _quickReleaseGLObjects && _quickReleaseCallbackInstalled )
                    {
                        _tilesToRelease.push_back( tile );
                    }

                    i = _tilesToShutDown.erase( i );
                }
                else
                    ++i;
            }
        }

        // call subclass to continue update..
        updateTraversal( nv );
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
            unsigned numActiveTasks = getNumActiveTasks();
            if ( numActiveTasks > 0 )
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

        //OE_INFO << "Tiles = " << _tiles.size() << std::endl;
    }

    osg::Group::traverse( nv );
}

void
TerrainNode::setDelay( unsigned frames )
{
    if ( _onDemandDelay == 0 && !_alwaysUpdate )
    {
        ADJUST_UPDATE_TRAV_COUNT( this, 1 );
    }
    _onDemandDelay = frames;
}

void
TerrainNode::decDelay()
{
    _onDemandDelay--;
    if ( _onDemandDelay == 0 && !_alwaysUpdate )
    {
        ADJUST_UPDATE_TRAV_COUNT(this, -1);
    }
}
