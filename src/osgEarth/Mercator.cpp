///* -*-c++-*- */
///* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
// * Copyright 2008-2009 Pelican Ventures, Inc.
// * http://osgearth.org
// *
// * osgEarth is free software; you can redistribute it and/or modify
// * it under the terms of the GNU Lesser General Public License as published by
// * the Free Software Foundation; either version 2 of the License, or
// * (at your option) any later version.
// *
// * This program is distributed in the hope that it will be useful,
// * but WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// * GNU Lesser General Public License for more details.
// *
// * You should have received a copy of the GNU Lesser General Public License
// * along with this program.  If not, see <http://www.gnu.org/licenses/>
// */
//
//#include <osgEarth/Mercator>
//#include <osg/Math>
//#include <osg/Notify>
//#include <sstream>
//#include <algorithm>
//#include <string.h>
//
//using namespace osgEarth;
//
//#define MERC_MAX_LAT  85.084059050110383
//#define MERC_MIN_LAT -85.084059050110383
//
//static double
//lonToU(double lon) {
//    return (lon + 180.0) / 360.0;
//}
//
//static double
//latToV(double lat) {
//    double sin_lat = sin( osg::DegreesToRadians( lat ) );
//    return 0.5 - log( (1+sin_lat) / (1-sin_lat) ) / (4*osg::PI);
//}
//
//static void
//getUV(const GeoExtent& ext,
//      double lon, double lat,
//      double& out_u, double& out_v)
//{
//    out_u = (lon-ext.xMin())/ext.width();
//
//    double vmin = latToV( osg::clampBetween( ext.yMax(), MERC_MIN_LAT, MERC_MAX_LAT ) );
//    double vmax = latToV( osg::clampBetween( ext.yMin(), MERC_MIN_LAT, MERC_MAX_LAT ) );
//    double vlat = latToV( osg::clampBetween( lat, MERC_MIN_LAT, MERC_MAX_LAT ) );
//
//    out_v = (vlat-vmin)/(vmax-vmin);
//}
//
//
//MercatorLocator::MercatorLocator( const osgTerrain::Locator& prototype, const GeoExtent& image_ext ) :
//osgTerrain::Locator( prototype )
//{
//    // assumption: incoming extent is Mercator SRS
//    _image_ext = image_ext.transform( image_ext.getSRS()->getGeographicSRS() );
//}
//
//
//bool
//MercatorLocator::convertLocalToModel(const osg::Vec3d& local, osg::Vec3d& world) const
//{
//    switch(_coordinateSystemType)
//    {
//        case(GEOCENTRIC):
//        {
//            return Locator::convertLocalToModel( local, world );
//        }
//        case(GEOGRAPHIC):
//        {        
//            return Locator::convertLocalToModel( local, world );
//        }
//        case(PROJECTED):
//        {        
//            return Locator::convertLocalToModel( local, world );
//        }
//    }    
//
//    return false;
//}
//
//bool
//MercatorLocator::convertModelToLocal(const osg::Vec3d& world, osg::Vec3d& local) const
//{
//    // OSG 2.7 bug workaround: bug fix in Locator submitted by GW on 10/3/2008:
//    const_cast<MercatorLocator*>(this)->_inverse.invert( _transform );
//
//    switch(_coordinateSystemType)
//    {
//    case(GEOCENTRIC):
//        {
//            double longitude, latitude, height;
//
//            _ellipsoidModel->convertXYZToLatLongHeight(world.x(), world.y(), world.z(),
//                latitude, longitude, height );
//
//            local = osg::Vec3d(longitude, latitude, height) * _inverse;
//
//            double lon_deg = osg::RadiansToDegrees(longitude);
//            double lat_deg = osg::RadiansToDegrees(latitude);
//            double xr, yr;
//
//            //GeoExtent tile_extent(
//            //    NULL,
//            //    osg::RadiansToDegrees(_transform(3,0)),
//            //    osg::RadiansToDegrees(_transform(3,1)),
//            //    osg::RadiansToDegrees(_transform(3,0)+_transform(0,0)),
//            //    osg::RadiansToDegrees(_transform(3,1)+_transform(1,1) ) );
//
//            getUV( _image_ext, lon_deg, lat_deg, xr, yr );
//
//            local.x() = xr;
//            local.y() = 1.0-yr;
//            return true;
//        }
//
//
//    case(GEOGRAPHIC):
//        {        
//            local = world * _inverse;
//
//            osg::Vec3d w = world;
//            double lon_deg = w.x();
//            double lat_deg = w.y();
//
//            double xr, yr;
//            getUV( _image_ext, lon_deg, lat_deg, xr, yr );
//
//            local.x() = xr;
//            local.y() = 1.0-yr;
//            return true;
//        }
//
//    case(PROJECTED):
//        {        
//            local = world * _inverse;
//            return true;      
//        }
//    }    
//
//    return false;
//}
//
