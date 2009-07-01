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
#include <osgSim/HeightAboveTerrain>
#include <osg/MatrixTransform>
#include <osg/CoordinateSystemNode>

using namespace osgEarthUtil;
using namespace osgEarth;

ObjectPlacer::ObjectPlacer( osg::Node* terrain, bool clamp ) :
_clamp( clamp )
{
    _mapNode = findTopMostNodeOfType<osgEarth::MapNode>( terrain );
    _csn = findTopMostNodeOfType<osg::CoordinateSystemNode>( terrain );
}

static double
getHAT( osg::CoordinateSystemNode* csn, double x, double y, double z )
{
    osgSim::HeightAboveTerrain hat;
    hat.setLowestHeight( -10000 );
    int index = hat.addPoint( osg::Vec3d( x, y, z ) );
    hat.computeIntersections( csn ); // input node must be a CSN
    return hat.getHeightAboveTerrain( index );
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

    const SpatialReference* srs = _mapNode->getProfile()->getSRS();

    // now build a matrix:
    if ( !is_geocentric ) // projected or "flat geographic"
    {
        double local_x, local_y, local_z = height;
        
        // first convert the input coords to the map srs:
        srs->getGeographicSRS()->transform( lon_deg, lat_deg, srs, local_x, local_y );

        if ( _clamp )
        {
            local_z = getHAT( _csn.get(), local_x, local_y, 0 ) + height;
        }
        out_result = osg::Matrixd::translate( local_x, local_y, local_z );
    }
    else // geocentric
    {    
        double lat_rad = osg::DegreesToRadians( lat_deg );
        double lon_rad = osg::DegreesToRadians( lon_deg );

        if ( _clamp )
        {
            double gx, gy, gz; // geocentric/ecef
            srs->getEllipsoid()->convertLatLongHeightToXYZ( lat_rad, lon_rad, 0, gx, gy, gz );
            double z = getHAT( _csn.get(), gx, gy, gz );
            srs->getEllipsoid()->computeLocalToWorldTransformFromLatLongHeight( lat_rad, lon_rad, z+height, out_result );
        }
        else
        {
            srs->getEllipsoid()->computeLocalToWorldTransformFromLatLongHeight( lat_rad, lon_rad, height, out_result );
        }
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
