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
GeoMath::distance(const std::vector< osg::Vec3d > &points, double radius)
{
    double length = 0;

    if (points.size() > 1)
    {
        for (unsigned int i = 0; i < points.size()-1; ++i)
        {
            const osg::Vec3d& current = points[i];
            const osg::Vec3d& next    = points[i+1];
            length += GeoMath::distance(osg::DegreesToRadians(current.y()), osg::DegreesToRadians(current.x()),
                osg::DegreesToRadians(next.y()), osg::DegreesToRadians(next.x()), radius);
        }
    }
    return length;
}

double
GeoMath::distance(const osg::Vec3d& p1, const osg::Vec3d& p2, const SpatialReference* srs )
{
    if ( srs == 0L || srs->isProjected() )
    {
        return (p2-p1).length();
    }
    else
    {
        return distance(
            osg::DegreesToRadians( p1.y() ), osg::DegreesToRadians( p1.x() ),
            osg::DegreesToRadians( p2.y() ), osg::DegreesToRadians( p2.x() ),
            srs->getEllipsoid()->getRadiusEquator() );
    }
}

double
GeoMath::bearing(double lat1Rad, double lon1Rad,
                 double lat2Rad, double lon2Rad)
{
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

void
GeoMath::interpolate(double lat1Rad, double lon1Rad,
                     double lat2Rad, double lon2Rad,
                     double t,
                     double& out_latRad, double& out_lonRad)
{
    static osg::EllipsoidModel em;

    osg::Vec3d v0, v1;

    em.convertLatLongHeightToXYZ(lat1Rad, lon1Rad, 0, v0.x(), v0.y(), v0.z());
    v0.normalize();
    em.convertLatLongHeightToXYZ(lat2Rad, lon2Rad, 0, v1.x(), v1.y(), v1.z());
    v1.normalize();

    osg::Vec3d axis = v0 ^ v1;
    double angle = acos( v0 * v1 );
    osg::Quat q( angle * t, axis );

    v0 = q * v0;
    v0 *= 0.5 * (em.getRadiusEquator()+em.getRadiusPolar());

    double dummy;
    em.convertXYZToLatLongHeight( v0.x(), v0.y(), v0.z(), out_latRad, out_lonRad, dummy );
}

double
GeoMath::rhumbDistance(double lat1Rad, double lon1Rad,
                       double lat2Rad, double lon2Rad,
                       double radius)
{
    double dLat = (lat2Rad - lat1Rad);
    double dLon = osg::absolute(lon2Rad - lon1Rad);

    double dPhi = log(tan(lat2Rad/2.0+osg::PI/4.0)/tan(lat1Rad/2.0+osg::PI/4.0));
    double q = (!osg::isNaN(dLat/dPhi)) ? dLat/dPhi : cos(lat1Rad);  // E-W line gives dPhi=0
    // if dLon over 180� take shorter rhumb across 180� meridian:
    if (dLon > osg::PI) dLon = 2.0*osg::PI - dLon;
    double dist = sqrt(dLat*dLat + q*q*dLon*dLon) * radius; 
    return dist;
}

double
GeoMath::rhumbDistance(const std::vector< osg::Vec3d > &points, double radius)
{
    double length = 0;
    if (points.size() > 1)
    {
        for (unsigned int i = 0; i < points.size()-1; ++i)
        {
            const osg::Vec3d& current = points[i];
            const osg::Vec3d& next    = points[i+1];
            length += GeoMath::rhumbDistance(osg::DegreesToRadians(current.y()), osg::DegreesToRadians(current.x()),
                                             osg::DegreesToRadians(next.y()), osg::DegreesToRadians(next.x()), radius);                                             
        }
    }
    return length;
}

double
GeoMath::rhumbBearing(double lat1Rad, double lon1Rad,
                      double lat2Rad, double lon2Rad)
{
  double dLon = lon2Rad - lon1Rad;
  
  double dPhi = log(tan(lat2Rad/2.0+osg::PI/4.0)/tan(lat1Rad/2.0+osg::PI/4.0));
  if (osg::absolute(dLon) > osg::PI) dLon = dLon > 0.0 ? -(2.0*osg::PI-dLon) : (2.0*osg::PI+dLon);
  double brng = atan2(dLon, dPhi);
  return fmod(brng + 2.0 * osg::PI, 2.0 * osg::PI);

}

void
GeoMath::rhumbDestination(double lat1Rad, double lon1Rad,
                          double bearing, double distance,
                          double &out_latRad, double &out_lonRad,
                          double radius)
{ 
  double R = radius;
  double d = distance / R;

  double lat2Rad = lat1Rad + d*cos(bearing);
  double dLat = lat2Rad-lat1Rad;
  double dPhi = log(tan(lat2Rad/2.0+osg::PI/4.0)/tan(lat1Rad/2.0+osg::PI/4.0));
  double q = (!osg::isNaN(dLat/dPhi)) ? dLat/dPhi : cos(lat1Rad);  // E-W line gives dPhi=0
  double dLon = d*sin(bearing)/q;
  // check for some daft bugger going past the pole
  if (osg::absolute(lat2Rad) > osg::PI/2.0) lat2Rad = lat2Rad > 0.0 ? osg::PI-lat2Rad : -(osg::PI-lat2Rad);
  //double lon2Rad = (lon1Rad+dLon+3.0*Math.PI)%(2.0*osg::PI) - osg::PI;
  double lon2Rad = fmod((lon1Rad+dLon+3.0*osg::PI),(2.0*osg::PI)) - osg::PI;

  out_latRad = lat2Rad;
  out_lonRad = lon2Rad;
}

