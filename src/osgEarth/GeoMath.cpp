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

#include <osgEarth/GeoMath>

using namespace osgEarth;

double
GeoMath::distance(double lat1Rad, double lon1Rad, double lat2Rad, double lon2Rad, double radius)
{  	
    double dLat = (lat2Rad-lat1Rad);
    double dLon = (lon2Rad-lon1Rad); 
    double a = sin(dLat/2.0) * sin(dLat/2.0) +
               cos(lat1Rad) *  cos(lat2Rad) * 
               sin(dLon/2.0) * sin(dLon/2.0); 
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0-a)); 
    double d = radius * c;
    return d;
}

double
GeoMath::bearing(double lat1Rad, double lon1Rad,
                 double lat2Rad, double lon2Rad)
{
    double dLat = (lat2Rad-lat1Rad);
    double dLon = (lon2Rad-lon1Rad); 
    double y = sin(dLon) * cos(lat2Rad);
    double x = cos(lat1Rad)*sin(lat2Rad) - sin(lat1Rad)*cos(lat2Rad)*cos(dLon);
    double brng = atan2(y, x);
    return brng;
}

void
GeoMath::midpoint(double lat1Rad, double lon1Rad,
                  double lat2Rad, double lon2Rad,
                  double &out_latRad, double &out_lonRad)
{     
    double dLat = (lat2Rad-lat1Rad);
    double dLon = (lon2Rad-lon1Rad); 

    double cosLat1 = cos(lat1Rad);
    double cosLat2 = cos(lat2Rad);
    double sinLat1 = sin(lat1Rad);
    double sinLat2 = sin(lat2Rad);


    double Bx = cosLat2 * cos(dLon);
    double By = cosLat2 * sin(dLon);
    out_latRad = atan2(sinLat1+sinLat2,
                       sqrt( (cosLat1+Bx)*(cosLat1+Bx) + By*By) ); 
    out_lonRad = lon1Rad + atan2(By, cosLat1 + Bx);
}

void
GeoMath::destination(double lat1Rad, double lon1Rad,
                     double bearingRad, double distance,
                     double &out_latRad, double &out_lonRad,
                     double radius)
{
    double dR = distance / radius;
    out_latRad = asin( sin(lat1Rad)*cos(dR) + 
                       cos(lat1Rad)*sin(dR)*cos(bearingRad) );
    out_lonRad = lon1Rad + atan2(sin(bearingRad)*sin(dR)*cos(lat1Rad), 
                                 cos(dR)-sin(lat1Rad)*sin(out_latRad));
}