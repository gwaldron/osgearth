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
#include <osgEarthUtil/ObjectPlacer>
#include <osgEarth/FindNode>
#include <osgEarth/MapNode>
#include <osgEarth/SpatialReference>
#include <osgSim/LineOfSight>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>
#include <osg/MatrixTransform>
#include <osg/CoordinateSystemNode>

using namespace osgEarthUtil;
using namespace osgEarth;

struct CachingReadCallback : public osgSim::DatabaseCacheReadCallback
{
    CachingReadCallback(int maxReads) : _maxReads(maxReads), _reads(0) { }
    void reset() { _reads = 0; }
    virtual osg::Node* readNodeFile(const std::string& filename) {
        if ( _reads < _maxReads ) {
            _reads++;
            return osgSim::DatabaseCacheReadCallback::readNodeFile(filename);
        }
        else {
            return NULL;
        }
    }
    int _reads, _maxReads;
    osg::ref_ptr<osg::Node> _lastNodeRead;
};

ObjectPlacer::ObjectPlacer( osg::Node* terrain, int traversalMask, bool clamp, int maxLevel ) :
_clamp( clamp ),
_traversalMask( traversalMask )
{
    _mapNode = findTopMostNodeOfType<osgEarth::MapNode>( terrain );
    _csn = findTopMostNodeOfType<osg::CoordinateSystemNode>( terrain );
    _readCallback = new CachingReadCallback( maxLevel );
}

bool
ObjectPlacer::clampGeocentric( osg::CoordinateSystemNode* csn, double lat_rad, double lon_rad, osg::Vec3d& out ) const
{
    osg::Vec3d start, end;
    
    csn->getEllipsoidModel()->convertLatLongHeightToXYZ( lat_rad, lon_rad, 50000, start.x(), start.y(), start.z() );
    csn->getEllipsoidModel()->convertLatLongHeightToXYZ( lat_rad, lon_rad, -50000, end.x(), end.y(), end.z() );
    osgUtil::LineSegmentIntersector* i = new osgUtil::LineSegmentIntersector( start, end );
    
    osgUtil::IntersectionVisitor iv;
    iv.setIntersector( i );
    static_cast<CachingReadCallback*>(_readCallback.get())->reset();
    iv.setReadCallback( _readCallback.get() );
    iv.setTraversalMask( _traversalMask );

    _mapNode->accept( iv );

    osgUtil::LineSegmentIntersector::Intersections& results = i->getIntersections();
    if ( !results.empty() )
    {
        const osgUtil::LineSegmentIntersector::Intersection& result = *results.begin();
        out = result.matrix.valid() ?
            result.localIntersectionPoint * (*result.matrix) :
            result.localIntersectionPoint;
        return true;            
    }
    return false;
}

bool
ObjectPlacer::clampProjected( osg::CoordinateSystemNode* csn, double x, double y, osg::Vec3d& out ) const
{
    osg::Vec3d start( x, y, 50000 );
    osg::Vec3d end(x, y, -50000);
    osgUtil::LineSegmentIntersector* i = new osgUtil::LineSegmentIntersector( start, end );
    
    osgUtil::IntersectionVisitor iv;
    iv.setIntersector( i );
    static_cast<CachingReadCallback*>(_readCallback.get())->reset();
    iv.setReadCallback( _readCallback.get() );
    iv.setTraversalMask( _traversalMask );

    _mapNode->accept( iv );

    osgUtil::LineSegmentIntersector::Intersections& results = i->getIntersections();
    if ( !results.empty() )
    {
        const osgUtil::LineSegmentIntersector::Intersection& result = *results.begin();
        out = result.matrix.valid() ?
            result.localIntersectionPoint * (*result.matrix) :
            result.localIntersectionPoint;
        return true;            
    }
    return false;
}

bool
ObjectPlacer::createPlacerMatrix( double lat_deg, double lon_deg, double height, osg::Matrixd& out_result ) const
{
    if ( !_mapNode.valid() || !_csn.valid() )
    {
        osg::notify( osg::WARN ) << "osgEarthUtil::ObjectPlacer: terrain is missing either a Map or CSN node" << std::endl;             
        return false;
    }

    // see whether this is a geocentric model:
    bool is_geocentric = _csn.valid() && _csn->getEllipsoidModel() != NULL;

    const SpatialReference* srs = _mapNode->getMap()->getProfile()->getSRS();

    // now build a matrix:
    if ( is_geocentric )
    {
        double lat_rad = osg::DegreesToRadians( lat_deg );
        double lon_rad = osg::DegreesToRadians( lon_deg );

        if ( _clamp )
        {
            osg::Vec3d c;
            if ( clampGeocentric( _csn.get(), lat_rad, lon_rad, c ) )
            {
                srs->getEllipsoid()->computeLocalToWorldTransformFromXYZ( c.x(), c.y(), c.z(), out_result );
            }
        }
        else
        {
            srs->getEllipsoid()->computeLocalToWorldTransformFromLatLongHeight( lat_rad, lon_rad, height, out_result );
        }
    }
    else // projected or "flat geographic"
    {
        osg::Vec3d local(0, 0, height);
        
        // first convert the input coords to the map srs:
        srs->getGeographicSRS()->transform( lon_deg, lat_deg, srs, local.x(), local.y());

        if ( _clamp )
        {
            clampProjected( _csn.get(), local.x(), local.y(), local );
            local.z() += height;
        }
        out_result = osg::Matrixd::translate( local );
    }

    return true;
}

osg::Node*
ObjectPlacer::placeNode( osg::Node* node, double lat_deg, double lon_deg, double height ) const
{
    osg::Node* result = NULL;

    osg::Matrixd matrix;
    if ( createPlacerMatrix( lat_deg, lon_deg, height, matrix ) )
    {
        osg::MatrixTransform* mt = new osg::MatrixTransform( matrix );
        mt->addChild( node );
        result = mt;
    }

    return result;
}
