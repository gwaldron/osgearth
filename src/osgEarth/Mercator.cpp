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

#define MIN_X -20037508.34
#define MIN_Y -20037508.34
#define MAX_X 20037508.34
#define MAX_Y 20037508.34

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

static double
clamp( double n, double min_value, double max_value )
{
    return osg::minimum( osg::maximum( n, min_value ), max_value );
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

    double clamp_x = clamp(raw_x, 0, map_size - 1);
    double clamp_y = clamp(raw_y, 0, map_size - 1);

    out_x = (unsigned int)clamp_x;
    out_y = (unsigned int)clamp_y;

    return raw_y < clamp_y? -1 : raw_y > clamp_y ? 1 : 0;
    //return raw_x == clamp_x && raw_y == clamp_y;
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

static osg::Image*
sharpen( osg::Image* input )
{
    int filter[9] = { 0, -1, 0, -1, 5, -1, 0, -1, 0 };
    osg::Image* output = new osg::Image( *input );
    for( int t=1; t<input->t()-1; t++ )
    {
        for( int s=1; s<input->s()-1; s++ )
        {
            int pixels[9] = {
                *(int*)input->data(s-1,t-1), *(int*)input->data(s,t-1), *(int*)input->data(s+1,t-1),
                *(int*)input->data(s-1,t  ), *(int*)input->data(s,t  ), *(int*)input->data(s+1,t  ),
                *(int*)input->data(s-1,t+1), *(int*)input->data(s,t+1), *(int*)input->data(s+1,t+1) };

            int shifts[4] = { 0, 8, 16, 32 };

            for( int c=0; c<4; c++ ) // components
            {
                int mask = 0xff << shifts[c];
                int sum = 0;
                for( int i=0; i<9; i++ )
                {
                    sum += ((pixels[i] & mask) >> shifts[c]) * filter[i];
                }
                sum = sum > 255? 255 : sum < 0? 0 : sum;
                output->data(s,t)[c] = sum;
            }
        }
    }
    return output;
}

/*********************************************************************/


MercatorLocator::MercatorLocator( const osgTerrain::Locator& rhs, int _tile_size, unsigned int _lod ) :
osgTerrain::Locator(rhs), tile_size( _tile_size ), lod(_lod)
{
    //NOP
}

MercatorLocator::MercatorLocator( int _tile_size, unsigned int _lod ) :
tile_size(_tile_size), lod( _lod )
{
    //NOP
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
            //world = local * _transform;

            //double lon_deg = world.x();
            //double lat_deg = world.y();
            //unsigned int px, py;
            //MercatorTileKey::longLatToPixelXY( lon_deg, lat_deg, lod, px, py );
            //if ( py % tile_size != 0 )
            //{
            //    py %= tile_size;
            //    world.y() = ((double)tile_size-(double)py)/(double)tile_size;
            //    //osg::notify(osg::NOTICE) << "lat="<<lat_deg<<", lon="<<lon_deg<<", py="<<py<<", worldy="<<world.y()<<std::endl;
            //}
            return true;      
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
            unsigned int px, py;
            Mercator::longLatToPixelXY( lon_deg, lat_deg, lod, tile_size, px, py );
            if ( py % tile_size != 0 )
            {
                py %= tile_size;
                local.y() = ((double)tile_size-(double)py)/(double)tile_size;
                //osg::notify(osg::NOTICE) << "KEY="<<key->str()<<": lat="<<lat_deg<<", lon="<<lon_deg<<", py="<<py<<", localy="<<local.y()<<std::endl;
            }

            return true;      
        }
    case(GEOGRAPHIC):
        {        
            local = world * _inverse;

            osg::Vec3d w = world;
            double lon_deg = w.x();
            double lat_deg = w.y();

            unsigned int px, py;
            Mercator::longLatToPixelXY( lon_deg, lat_deg, lod, tile_size, px, py );
            if ( py % tile_size != 0 )
            {
                py %= tile_size;
                local.y() = ((double)tile_size-(double)py)/(double)tile_size;
               // osg::notify(osg::NOTICE) << "lod="<<lod<<", lat="<<lat_deg<<", lon="<<lon_deg<<", py="<<py<<", local.y="<<local.y()<<std::endl;
            }

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
