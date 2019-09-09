/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgViewer/View>

#define LC "[Terrain] "

using namespace osgEarth;

//---------------------------------------------------------------------------

Terrain::OnTileAddedOperation::OnTileAddedOperation(const TileKey& key, osg::Node* node, Terrain* terrain)
    : osg::Operation("OnTileAdded", true),
        _terrain(terrain), _key(key), _node(node), _count(0), _delay(0) { }

void Terrain::OnTileAddedOperation::operator()(osg::Object*)
{
    if ( getKeep() == false )
        return;

    if (_delay-- > 0)
        return;

    ++_count;
    osg::ref_ptr<Terrain>   terrain;
    osg::ref_ptr<osg::Node> node;
    
    if ( _terrain.lock(terrain) && (!_key.valid() || _node.lock(node)) )
    {
        if (_key.valid())
            terrain->fireTileAdded( _key, node.get() );
        else
            terrain->fireTileAdded( _key, 0L );

    }
    else
    {
        // nop; tile expired; let it go.
        OE_DEBUG << "Tile expired before notification: " << _key.str() << std::endl;
    }

    this->setKeep( false );
}

//---------------------------------------------------------------------------

Terrain::Terrain(osg::Node* graph, const Profile* mapProfile, const TerrainOptions& terrainOptions ) :
_graph         ( graph ),
_profile       ( mapProfile ),
_terrainOptions( terrainOptions )
{
    _updateQueue = new osg::OperationQueue();
}

void
Terrain::update()
{
    _updateQueue->runOperations();
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

    if (srs && srs->isGeographic())
    {
        // perturb polar latitudes slightly to prevent intersection anomaly at the poles
        if (getSRS()->isGeographic())
        {
            if (osg::equivalent(y, 90.0))
                y -= 1e-7;
            else if (osg::equivalent(y, -90))
                y += 1e-7;
        }
    }

    const osg::EllipsoidModel* em = getSRS()->getEllipsoid();
    double r = osg::minimum( em->getRadiusEquator(), em->getRadiusPolar() );

    // calculate the endpoints for an intersection test:
    osg::Vec3d start(x, y, r);
    osg::Vec3d end  (x, y, -r);

    if ( getSRS()->isGeographic() )
    {
        const SpatialReference* ecef = getSRS()->getGeocentricSRS();
        getSRS()->transform(start, ecef, start);
        getSRS()->transform(end,   ecef, end);
    }

    osgUtil::LineSegmentIntersector* lsi = new osgUtil::LineSegmentIntersector( start, end );
    lsi->setIntersectionLimit(osgUtil::Intersector::LIMIT_NEAREST);

    osgUtil::IntersectionVisitor iv( lsi );
 
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

    float local_x, local_y = 0.0;
    const osg::Camera* camera = view2->getCameraContainingPosition(x, y, local_x, local_y);
    if (!camera)
        camera = view2->getCamera();

    // Build a matrix that transforms from the terrain/world space
    // to either clip or window space, depending on whether we have
    // a viewport. Is it even possible to not have a viewport? -gw
    osg::Matrixd matrix;

    // compensate for any transforms applied between terrain and camera:
    osg::Matrix terrainRefFrame = osg::computeLocalToWorld(_graph->getParentalNodePaths()[0]);
    matrix.postMult(terrainRefFrame);

    matrix.postMult(camera->getViewMatrix());
    matrix.postMult(camera->getProjectionMatrix());

    double zNear = -1.0;
    double zFar = 1.0;
    if (camera->getViewport())
    {
        matrix.postMult(camera->getViewport()->computeWindowMatrix());
        zNear = 0.0, zFar = 1.0;
    }

    osg::Matrixd inverse;
    inverse.invert(matrix);

    osg::Vec3d startVertex = osg::Vec3d(local_x,local_y,zNear) * inverse;
    osg::Vec3d endVertex = osg::Vec3d(local_x,local_y,zFar) * inverse;

    osg::ref_ptr< osgUtil::LineSegmentIntersector > picker = 
        new osgUtil::LineSegmentIntersector(osgUtil::Intersector::MODEL, startVertex, endVertex);

    // Limit it to one intersection; we only care about the nearest.
    picker->setIntersectionLimit( osgUtil::Intersector::LIMIT_NEAREST );

    osgUtil::IntersectionVisitor iv(picker.get());
    _graph->accept(iv);

    bool good = false;
    if (picker->containsIntersections())
    {        
        out_coords = picker->getIntersections().begin()->getWorldIntersectPoint();
        good = true;       
    }
    return good;
}

void
Terrain::addTerrainCallback( TerrainCallback* cb )
{
    if ( cb )
    {        
        removeTerrainCallback( cb );

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

    if (_callbacksSize > 0)
    {
        if (!key.valid())
            OE_WARN << LC << "notifyTileAdded with key = NULL\n";

        _updateQueue->add(new OnTileAddedOperation(key, node, this));
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
Terrain::notifyMapElevationChanged()
{
    if (_callbacksSize > 0)
    {
        OnTileAddedOperation* op = new OnTileAddedOperation(TileKey::INVALID, 0L, this);
        op->_delay = 1; // let the terrain update before applying this
        _updateQueue->add(op);
    }
}

void
Terrain::fireMapElevationChanged()
{
    // nop
}
void
Terrain::accept( osg::NodeVisitor& nv )
{
    _graph->accept( nv );
}
