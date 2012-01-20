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

#include <osgEarth/LocalTangentPlane>
#include <osg/Math>
#include <osg/Notify>
#include <sstream>
#include <algorithm>

using namespace osgEarth;

#define LC "[LTP] "

// --------------------------------------------------------------------------

LTPSpatialReference::LTPSpatialReference( void* handle, const osg::Vec3d& worldPointLLA ) :
SpatialReference( handle, false ),
_worldPointLLA  ( worldPointLLA )
{
    //todo, set proper init string
}

void
LTPSpatialReference::_init()
{
    SpatialReference::_init();

    _is_user_defined = true;
    _is_contiguous   = true;
    _is_ltp          = true;
    _is_geographic   = false;
    _name            = "ENU Local Tangent Plane";

    // set up the LTP matrixes.

    getEllipsoid()->computeLocalToWorldTransformFromLatLongHeight(
        osg::DegreesToRadians(_worldPointLLA.y()),
        osg::DegreesToRadians(_worldPointLLA.x()),
        _worldPointLLA.z(),
        _local2world);

    _world2local.invert( _local2world );
}

bool
LTPSpatialReference::preTransform(double& x, double& y, double& z, void* context) const
{
    osg::Vec3d world = osg::Vec3d(x,y,z) * _local2world;
    double lat, lon, height;
    getEllipsoid()->convertXYZToLatLongHeight(world.x(), world.y(), world.z(), lat, lon, height);
    x = osg::RadiansToDegrees(lon);
    y = osg::RadiansToDegrees(lat);
    z = height;
    return true;
}

bool
LTPSpatialReference::postTransform(double& x, double& y, double& z, void* context) const
{
    osg::Vec3d world;
    getEllipsoid()->convertLatLongHeightToXYZ(
        osg::DegreesToRadians(y), osg::DegreesToRadians(x), z,
        world.x(), world.y(), world.z() );
    osg::Vec3d local = world * _world2local;
    x = local.x(), y = local.y(), z = local.z();
    return true;
}

bool
LTPSpatialReference::_isEquivalentTo( const SpatialReference* srs ) const
{
    return 
        srs->isLTP() && 
        _worldPointLLA == static_cast<const LTPSpatialReference*>(srs)->_worldPointLLA ;
    // todo: check the reference ellipsoids
}
