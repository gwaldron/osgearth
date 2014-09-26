/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include <osgEarth/DPLineSegmentIntersector>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>
#include <osgViewer/View>

#define LC "[Terrain] "

using namespace osgEarth;

//---------------------------------------------------------------------------

namespace
{
    struct BaseOp : public osg::Operation
    {
        BaseOp(Terrain* terrain, bool keepByDefault) : osg::Operation("",keepByDefault), _terrain(terrain) { }
        osg::observer_ptr<Terrain> _terrain;
    };

    struct OnTileAddedOperation : public BaseOp
    {
        TileKey _key;
        osg::observer_ptr<osg::Node> _node;
        unsigned _count;

        OnTileAddedOperation(const TileKey& key, osg::Node* node, Terrain* terrain)
            : BaseOp(terrain, true), _key(key), _node(node), _count(0) { }

        void operator()(osg::Object*)
        {
            if ( getKeep() == false )
                return;

            ++_count;
            osg::ref_ptr<Terrain>   terrain;
            osg::ref_ptr<osg::Node> node;

            if ( _terrain.lock(terrain) && _node.lock(node) )
            {
                if ( node->getNumParents() > 0 )
                {
                    //OE_NOTICE << LC << "FIRING onTileAdded for " << _key.str() << " (tries=" << _count << ")" << std::endl;
                    terrain->fireTileAdded( _key, node.get() );
                    this->setKeep( false );
                }
                else
                {
                    //OE_NOTICE << LC << "Deferring onTileAdded for " << _key.str() << std::endl;
                }
            }
            else
            {
                // nop; tile expired; let it go.
                //OE_NOTICE << "Tile expired before notification: " << _key.str() << std::endl;
                this->setKeep( false );
            }
        }
    };
}

//---------------------------------------------------------------------------

Terrain::Terrain(osg::Node* graph, const Profile* mapProfile, bool geocentric, const TerrainOptions& terrainOptions ) :
_graph         ( graph ),
_profile       ( mapProfile ),
_geocentric    ( geocentric ),
_terrainOptions( terrainOptions )
{
    //nop
}

bool
Terrain::getHeight(osg::Node*              patch,
                   const SpatialReference* srs,
                   double                  x, 
                   double                  y, 
                   double*                 out_hamsl,
                   double*                 out_hae    ) const
{
    if ( !_graph.valid() && !patch )
        return 0L;

    // convert to map coordinates:
    if ( srs && !srs->isHorizEquivalentTo(getSRS()) )
    {
        srs->transform2D(x, y, getSRS(), x, y);
    }

    // trivially reject a point that lies outside the terrain:
    if ( !getProfile()->getExtent().contains(x, y) )
        return 0L;

    const osg::EllipsoidModel* em = getSRS()->getEllipsoid();
    double r = std::min( em->getRadiusEquator(), em->getRadiusPolar() );

    // calculate the endpoints for an intersection test:
    osg::Vec3d start(x, y, r);
    osg::Vec3d end  (x, y, -r);

    if ( isGeocentric() )
    {
        const SpatialReference* ecef = getSRS()->getECEF();
        getSRS()->transform(start, ecef, start);
        getSRS()->transform(end,   ecef, end);
    }

    DPLineSegmentIntersector* lsi = new DPLineSegmentIntersector( start, end );
    lsi->setIntersectionLimit(osgUtil::Intersector::LIMIT_NEAREST);

    osgUtil::IntersectionVisitor iv( lsi );
    iv.setTraversalMask( ~_terrainOptions.secondaryTraversalMask().value() );

    if ( patch )
        patch->accept( iv );
    else
        _graph->accept( iv );

    osgUtil::LineSegmentIntersector::Intersections& results = lsi->getIntersections();
    if ( !results.empty() )
    {
        const osgUtil::LineSegmentIntersector::Intersection& firstHit = *results.begin();
        osg::Vec3d hit = firstHit.getWorldIntersectPoint();

        getSRS()->transformFromWorld(hit, hit, out_hae);
        if ( out_hamsl )
            *out_hamsl = hit.z();

        return true;
    }
    return false;
}


bool
Terrain::getHeight(const SpatialReference* srs,
                   double                  x, 
                   double                  y,
                   double*                 out_hamsl,
                   double*                 out_hae ) const
{
    return getHeight( (osg::Node*)0L, srs, x, y, out_hamsl, out_hae );
}


bool
Terrain::getWorldCoordsUnderMouse(osg::View* view, float x, float y, osg::Vec3d& out_coords ) const
{
    osgViewer::View* view2 = dynamic_cast<osgViewer::View*>(view);
    if ( !view2 || !_graph.valid() )
        return false;

    osgUtil::LineSegmentIntersector::Intersections intersections;

    osg::NodePath nodePath;
    nodePath.push_back( _graph.get() );

    // fine but computeIntersections won't travers a masked Drawable, a la quadtree.
    unsigned traversalMask = ~_terrainOptions.secondaryTraversalMask().value();


#if 0
    // Old code, uses the computeIntersections method directly but sufferes from floating point precision problems.
    if ( view2->computeIntersections( x, y, nodePath, intersections, traversalMask ) )
    {
        // find the first hit under the mouse:
        osgUtil::LineSegmentIntersector::Intersection first = *(intersections.begin());
        out_coords = first.getWorldIntersectPoint();
        return true;
    }
    return false;    

#else
    // New code, uses the code from osg::View::computeIntersections but uses our DPLineSegmentIntersector instead to get around floating point precision issues.
    float local_x, local_y = 0.0;
    const osg::Camera* camera = view2->getCameraContainingPosition(x, y, local_x, local_y);
    if (!camera) camera = view2->getCamera();

    osg::Matrixd matrix;
    if (nodePath.size()>1)
    {
        osg::NodePath prunedNodePath(nodePath.begin(),nodePath.end()-1);
        matrix = osg::computeLocalToWorld(prunedNodePath);
    }

    matrix.postMult(camera->getViewMatrix());
    matrix.postMult(camera->getProjectionMatrix());

    double zNear = -1.0;
    double zFar = 1.0;
    if (camera->getViewport())
    {
        matrix.postMult(camera->getViewport()->computeWindowMatrix());
        zNear = 0.0;
        zFar = 1.0;
    }

    osg::Matrixd inverse;
    inverse.invert(matrix);

    osg::Vec3d startVertex = osg::Vec3d(local_x,local_y,zNear) * inverse;
    osg::Vec3d endVertex = osg::Vec3d(local_x,local_y,zFar) * inverse;

    // Use a double precision line segment intersector
    //osg::ref_ptr< osgUtil::LineSegmentIntersector > picker = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::MODEL, startVertex, endVertex);
    osg::ref_ptr< DPLineSegmentIntersector > picker = new DPLineSegmentIntersector(osgUtil::Intersector::MODEL, startVertex, endVertex);

    // Limit it to one intersection, we only care about the first
    picker->setIntersectionLimit( osgUtil::Intersector::LIMIT_NEAREST );

    osgUtil::IntersectionVisitor iv(picker.get());
    iv.setTraversalMask(traversalMask);
    nodePath.back()->accept(iv);

    if (picker->containsIntersections())
    {        
        intersections = picker->getIntersections();
        // find the first hit under the mouse:
        osgUtil::LineSegmentIntersector::Intersection first = *(intersections.begin());
        out_coords = first.getWorldIntersectPoint();        
        return true;       
    }
    return false;        
#endif
}


bool
Terrain::getWorldCoordsUnderMouse(osg::View* view,
                                  float x, float y,
                                  osg::Vec3d& out_coords,
                                  osg::ref_ptr<osg::Node>& out_node ) const
{
    osgViewer::View* view2 = dynamic_cast<osgViewer::View*>(view);
    if ( !view2 || !_graph.valid() )
        return false;

    osgUtil::LineSegmentIntersector::Intersections results;

    osg::NodePath path;
    path.push_back( _graph.get() );

    // fine but computeIntersections won't travers a masked Drawable, a la quadtree.
    unsigned mask = ~_terrainOptions.secondaryTraversalMask().value();

    if ( view2->computeIntersections( x, y, path, results, mask ) )
    {
        // find the first hit under the mouse:
        osgUtil::LineSegmentIntersector::Intersection first = *(results.begin());
        out_coords = first.getWorldIntersectPoint();
        for( osg::NodePath::reverse_iterator j = first.nodePath.rbegin(); j != first.nodePath.rend(); ++j ) {
            if ( !(*j)->getName().empty() ) {
                out_node = (*j);
                break;
            }
        }
        return true;
    }
    return false;
}


void
Terrain::addTerrainCallback( TerrainCallback* cb )
{
    if ( cb )
    {        
        Threading::ScopedWriteLock exclusiveLock( _callbacksMutex );
        _callbacks.push_back( cb );
        ++_callbacksSize; // atomic increment
    }
}

void
Terrain::removeTerrainCallback( TerrainCallback* cb )
{
    Threading::ScopedWriteLock exclusiveLock( _callbacksMutex );

    for( CallbackList::iterator i = _callbacks.begin(); i != _callbacks.end(); )
    {        
        if ( i->get() == cb )
        {
            i = _callbacks.erase( i );
            --_callbacksSize;
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

    osg::ref_ptr<osg::OperationQueue> queue;
    if ( _callbacksSize > 0 && _updateOperationQueue.lock(queue) )
    {
        queue->add( new OnTileAddedOperation(key, node, this) );
    }
}

void
Terrain::fireTileAdded( const TileKey& key, osg::Node* node )
{
    Threading::ScopedReadLock sharedLock( _callbacksMutex );

    for( CallbackList::iterator i = _callbacks.begin(); i != _callbacks.end(); )
    {       
        TerrainCallbackContext context( this );
        i->get()->onTileAdded( key, node, context );

        // if the callback set the "remove" flag, discard the callback.
        if ( context.markedForRemoval() )
            i = _callbacks.erase( i );
        else
            ++i;
    }
}


void
Terrain::accept( osg::NodeVisitor& nv )
{
    _graph->accept( nv );
}


//---------------------------------------------------------------------------

#undef  LC
#define LC "[TerrainPatch] "

TerrainPatch::TerrainPatch(osg::Node* patch, const Terrain* terrain) :
_patch  ( patch ),
_terrain( terrain )
{
    //nop
    if ( patch == 0L || terrain == 0L )
    {
        OE_WARN << "ILLEGAL: Created a TerrainPatch with a NULL parameter" << std::endl;
    }
}


bool
TerrainPatch::getHeight(const SpatialReference* srs,
                        double                  x, 
                        double                  y,
                        double*                 out_hamsl,
                        double*                 out_hae ) const
{
    if ( _terrain && _patch )
    {
        return _terrain->getHeight( _patch.get(), srs, x, y, out_hamsl, out_hae );
    }
    else
    {
        return false;
    }
}
