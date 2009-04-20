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

#include <osgEarth/Mercator>
#include <osg/Math>
#include <osg/Notify>
#include <sstream>
#include <algorithm>
#include <string.h>

using namespace osgEarth;

#define PROPERTY_FILTER           "filter"
#define VALUE_FILTER_NEAREST      "nearest"
#define VALUE_FILTER_LINEAR       "linear"

/********************************************************************/

const double Mercator::MIN_X = -20037508.34; //2789248;
const double Mercator::MIN_Y = -20037508.34; //2789248;
const double Mercator::MAX_X =  20037508.34; //2789248;
const double Mercator::MAX_Y =  20037508.34; //2789248;

#define MERC_MAX_LAT  85.084059050110383
#define MERC_MIN_LAT -85.084059050110383


#define PIXELS_PER_TILE 256

void Mercator::metersToLatLon(const double &x, const double &y, double &lat, double &lon)
{
    lon = x / MAX_X * 180.0;
    lat = osg::RadiansToDegrees( 2.0 * atan( exp( (y / MAX_Y) * osg::PI ) ) - .5*osg::PI );

    //Clamp the values to appropriate ranges.
    lon = osg::clampBetween(lon, -180.0, 180.0);
    lat = osg::clampBetween(lat, -90.0, 90.0);
}

void Mercator::latLongToMeters(const double &lat, const double &lon, double &x, double &y)
{
    x = lon * MAX_X / 180.0;
    y = log(tan((90 + lat) * osg::PI / 360)) / (osg::PI / 180.0);
    y = y * MAX_Y / 180.0;

    //Clamp the values to appropriate ranges
    x = osg::clampBetween(x, MIN_X, MAX_X);
    y = osg::clampBetween(y, MIN_Y, MAX_Y);
}

int
Mercator::longLatToPixelXY(double lon, double lat, unsigned int lod, int tile_size,
                                  unsigned int& out_x, unsigned int& out_y )
{
    double x = (lon + 180.0) / 360.0;
    double sin_lat = sin( osg::DegreesToRadians( lat ) );
    double y = 0.5 - log( (1+sin_lat) / (1-sin_lat) ) / (4*osg::PI);

    double map_size = (double)TileKey::getMapSizePixels( tile_size, lod );
    
    double raw_x = x * map_size + 0.5;
    double raw_y = y * map_size + 0.5;

    double clamp_x = osg::clampBetween(raw_x, (double)0, map_size - 1);
    double clamp_y = osg::clampBetween(raw_y, (double)0, map_size - 1);

    out_x = (unsigned int)clamp_x;
    out_y = (unsigned int)clamp_y;

    return raw_y < clamp_y? -1 : raw_y > clamp_y ? 1 : 0;
}

void
Mercator::pixelXYtoTileXY(unsigned int x, unsigned int y,
                          int tile_size,
                          unsigned int& out_tile_x,
                          unsigned int& out_tile_y)
{
    out_tile_x = x/tile_size;
    out_tile_y = y/tile_size;
}

/********************************************************************/

static double
lonToU(double lon) {
    return (lon + 180.0) / 360.0;
}

static double
latToV(double lat) {
    double sin_lat = sin( osg::DegreesToRadians( lat ) );
    return 0.5 - log( (1+sin_lat) / (1-sin_lat) ) / (4*osg::PI);
}

static void
getUV(const GeoExtent& ext,
      double lon, double lat,
      double& out_u, double& out_v)
{
    out_u = (lon-ext.xMin())/ext.width();

    double vmin = latToV( osg::clampBetween( ext.yMax(), MERC_MIN_LAT, MERC_MAX_LAT ) );
    double vmax = latToV( osg::clampBetween( ext.yMin(), MERC_MIN_LAT, MERC_MAX_LAT ) );
    double vlat = latToV( osg::clampBetween( lat, MERC_MIN_LAT, MERC_MAX_LAT ) );

    out_v = (vlat-vmin)/(vmax-vmin);
}


MercatorLocator::MercatorLocator( const osgTerrain::Locator& prototype, const GeoExtent& image_ext ) :
osgTerrain::Locator( prototype )
{
    // assumption: incoming extent is Mercator SRS
    _image_ext = image_ext.transform( image_ext.getSRS()->getGeographicSRS() );
}


bool
MercatorLocator::convertLocalToModel(const osg::Vec3d& local, osg::Vec3d& world) const
{
    switch(_coordinateSystemType)
    {
        case(GEOCENTRIC):
        {
            return Locator::convertLocalToModel( local, world );
        }
        case(GEOGRAPHIC):
        {        
            return Locator::convertLocalToModel( local, world );
        }
        case(PROJECTED):
        {        
            return Locator::convertLocalToModel( local, world );
        }
    }    

    return false;
}

bool
MercatorLocator::convertModelToLocal(const osg::Vec3d& world, osg::Vec3d& local) const
{
    // OSG 2.7 bug workaround: bug fix in Locator submitted by GW on 10/3/2008:
    const_cast<MercatorLocator*>(this)->_inverse.invert( _transform );

    switch(_coordinateSystemType)
    {
    case(GEOCENTRIC):
        {
            double longitude, latitude, height;

            _ellipsoidModel->convertXYZToLatLongHeight(world.x(), world.y(), world.z(),
                latitude, longitude, height );

            local = osg::Vec3d(longitude, latitude, height) * _inverse;

            double lon_deg = osg::RadiansToDegrees(longitude);
            double lat_deg = osg::RadiansToDegrees(latitude);
            double xr, yr;

            //GeoExtent tile_extent(
            //    NULL,
            //    osg::RadiansToDegrees(_transform(3,0)),
            //    osg::RadiansToDegrees(_transform(3,1)),
            //    osg::RadiansToDegrees(_transform(3,0)+_transform(0,0)),
            //    osg::RadiansToDegrees(_transform(3,1)+_transform(1,1) ) );

            getUV( _image_ext, lon_deg, lat_deg, xr, yr );

            local.x() = xr;
            local.y() = 1.0-yr;
            return true;
        }


    case(GEOGRAPHIC):
        {        
            local = world * _inverse;

            osg::Vec3d w = world;
            double lon_deg = w.x();
            double lat_deg = w.y();

            double xr, yr;
            getUV( _image_ext, lon_deg, lat_deg, xr, yr );

            local.x() = xr;
            local.y() = 1.0-yr;
            return true;
        }

    case(PROJECTED):
        {        
            local = world * _inverse;
            return true;      
        }
    }    

    return false;
}

