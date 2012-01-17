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

#include <osgEarth/Terrain>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

#define LC "[Terrain] "

using namespace osgEarth;

//---------------------------------------------------------------------------

namespace
{
    struct BaseOp : public osg::Operation
    {
        BaseOp(Terrain* terrain ) : osg::Operation("",false), _terrain(terrain) { }
        osg::ref_ptr<Terrain> _terrain;
    };

    struct OnTileAddedOperation : public BaseOp
    {
        TileKey _key;
        osg::observer_ptr<osg::Node> _node;

        OnTileAddedOperation(const TileKey& key, osg::Node* node, Terrain* terrain)
            : BaseOp(terrain), _key(key), _node(node) { }

        void operator()(osg::Object*)
        {
            osg::ref_ptr<osg::Node> node_safe = _node.get();
            if ( node_safe.valid() && _terrain.valid() )
                _terrain->fireTileAdded( _key, node_safe.get() );
        }
    };
}

//---------------------------------------------------------------------------

Terrain::Terrain(osg::Node* graph, const Profile* mapProfile, bool geocentric) :
_graph     ( graph ),
_profile   ( mapProfile ),
_geocentric( geocentric )
{
    //nop
}

bool
Terrain::getHeight(const osg::Vec3d& mapPos, double& out_height) const
{
    if ( !_graph.valid() )
        return 0L;

    // trivially reject a point that lies outside the terrain:
    if ( !_profile->getExtent().contains(mapPos.x(), mapPos.y()) )
        return 0L;

    // calculate the endpoints for an intersection test:
    osg::Vec3d start(mapPos.x(), mapPos.y(),  50000.0);
    osg::Vec3d end  (mapPos.x(), mapPos.y(), -50000.0);

    if ( _geocentric )
    {
        getSRS()->transformToECEF(start, start);
        getSRS()->transformToECEF(end, end);
    }

    osgUtil::LineSegmentIntersector* lsi = new osgUtil::LineSegmentIntersector(start, end);
    osgUtil::IntersectionVisitor iv( lsi );
    _graph->accept( iv );

    osgUtil::LineSegmentIntersector::Intersections& results = lsi->getIntersections();
    if ( !results.empty() )
    {
        const osgUtil::LineSegmentIntersector::Intersection& firstHit = *results.begin();
        osg::Vec3d hit = firstHit.getWorldIntersectPoint();

        if ( _geocentric )
        {
            getSRS()->transformFromECEF(hit, hit);
        }

        out_height = hit.z();
        return true;
    }
    return false;
}

void
Terrain::addTerrainCallback( TerrainCallback* cb, osg::Referenced* clientData )
{
    if ( cb )
    {
        CallbackRecord rec;
        rec._callback = cb;
        rec._clientData = clientData;
        rec._hasClientData = clientData != 0L;

        Threading::ScopedWriteLock exclusiveLock( _callbacksMutex );
        _callbacks.push_back( rec );
    }
}

void
Terrain::removeTerrainCallback( TerrainCallback* cb )
{
    Threading::ScopedWriteLock exclusiveLock( _callbacksMutex );

    for( CallbackList::iterator i = _callbacks.begin(); i != _callbacks.end(); )
    {
        CallbackRecord& rec = *i;
        if ( rec._callback.get() == cb )
        {
            i = _callbacks.erase( i );
        }
        else
        {
            ++i;
        }
    }
}

void
Terrain::removeTerrainCallbacksWithClientData( osg::Referenced* cd )
{
    Threading::ScopedWriteLock exclusiveLock( _callbacksMutex );

    for( CallbackList::iterator i = _callbacks.begin(); i != _callbacks.end(); )
    {
        CallbackRecord& rec = *i;
        if ( rec._hasClientData && rec._clientData.get() == cd )
        {
            i = _callbacks.erase( i );
        }
        else
        {
            ++i;
        }
    }
}

void
Terrain::notifyTileAdded( const TileKey& key, osg::Node* node )
{
    if ( !node )
    {
        OE_WARN << LC << "notify with a null node!" << std::endl;
    }

    if ( _updateOperationQueue.valid() )
    {
        _updateOperationQueue->add( new OnTileAddedOperation(key, node, this) );
    }
}

void
Terrain::fireTileAdded( const TileKey& key, osg::Node* node )
{
    Threading::ScopedReadLock sharedLock( _callbacksMutex );

    for( CallbackList::iterator i = _callbacks.begin(); i != _callbacks.end(); )
    {
        CallbackRecord& rec = *i;

        bool keep = true;
        osg::ref_ptr<osg::Referenced> clientData_safe = rec._clientData.get();

        // if the client data has gone away, disarcd the callback.
        if ( rec._hasClientData && !clientData_safe.valid() )
        {
            keep = false;
        }
        else
        {
            TerrainCallbackContext context( this, clientData_safe.get() );
            rec._callback->onTileAdded( key, node, context );

            // if the callback set the "remove" flag, discard the callback.
            if ( context._remove )
                keep = false;
        }

        if ( keep )
            ++i;
        else
            i = _callbacks.erase( i );
    }
}
