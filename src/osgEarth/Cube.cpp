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

#include <osgEarth/Cube>
#include <osg/Math>
#include <osg/Notify>
#include <sstream>
#include <algorithm>

using namespace osgEarth;

SquarePolarLocator::SquarePolarLocator()
//const osgTerrain::Locator& prototype ) :
//osgTerrain::Locator( prototype )
{
    // assumption: incoming key and image extent are square polar.
}

bool
SquarePolarLocator::convertLocalToModel( const osg::Vec3d& local, osg::Vec3d& world ) const
{
#if ((OPENSCENEGRAPH_MAJOR_VERSION <= 2) && (OPENSCENEGRAPH_MINOR_VERSION < 8))
    // OSG 2.7 bug workaround: bug fix in Locator submitted by GW
    const_cast<SquarePolarLocator*>(this)->_inverse.invert( _transform );
#endif

    if ( _coordinateSystemType == GEOCENTRIC )
    {
        // tile extents (lat/long)
        GeoExtent tile_extent(
            NULL,
            osg::RadiansToDegrees(_transform(3,0)),
            osg::RadiansToDegrees(_transform(3,1)),
            osg::RadiansToDegrees(_transform(3,0)+_transform(0,0)),
            osg::RadiansToDegrees(_transform(3,1)+_transform(1,1) ) );        

        double lat = tile_extent.yMax() - tile_extent.height() * local.x();
        double theta = atan2( local.x(), local.y() );
        double lon = tile_extent.xMin() + tile_extent.width() * sin( theta );

        _ellipsoidModel->convertLatLongHeightToXYZ(
            osg::DegreesToRadians(lat),
            osg::DegreesToRadians(lon),
            local.z(),
            world.x(), world.y(), world.z() );

        return true;
    }
    return false;
}


bool
SquarePolarLocator::convertModelToLocal(const osg::Vec3d& world, osg::Vec3d& local) const
{
#if ((OPENSCENEGRAPH_MAJOR_VERSION <= 2) && (OPENSCENEGRAPH_MINOR_VERSION < 8))
    // OSG 2.7 bug workaround: bug fix in Locator submitted by GW
    const_cast<SquarePolarLocator*>(this)->_inverse.invert( _transform );
#endif

    switch(_coordinateSystemType)
    {
    case(GEOCENTRIC):
        {
            //TODO: implement this correctly... this is just stub code from Locator
            double longitude, latitude, height;

            _ellipsoidModel->convertXYZToLatLongHeight(world.x(), world.y(), world.z(),
                latitude, longitude, height );

            local = osg::Vec3d(longitude, latitude, height) * _inverse;
            return true;
        }


    case(GEOGRAPHIC):
    case(PROJECTED):
        // Neither of these is supported for this locator..
        {        
            local = world * _inverse;
            return true;      
        }
    }    

    return false;
}

