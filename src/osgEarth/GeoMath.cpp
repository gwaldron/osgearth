/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
GeoMath::greatCircleMinMaxLatitude(double lat1Rad, double lon1Rad,
                                   double lat2Rad, double lon2Rad,
                                   double& out_minLatRad, double& out_maxLatRad)
{
    out_minLatRad = std::min(lat1Rad, lat2Rad);
    out_maxLatRad = std::max(lat1Rad, lat2Rad);

    // apply some spherical trig
    // http://en.wikipedia.org/wiki/Spherical_trigonometry

    double a = fabs(bearing(lat1Rad, lon1Rad, lat2Rad, lon2Rad)); // initial azimuth from p1=>p2
    double b = fabs(bearing(lat2Rad, lon2Rad, lat1Rad, lon1Rad)); // initial azimuth from p2=>p1

    // form a spherical triangle with the 2 points and the north pole, and 
    // use the law of sines to calculate the point at which the great circle
    // crosses a meridian (thereby make a right angle and demarking the maximum
    // latitude of the arc). Test whether that point actually lies between the
    // two points: the angles made by each point and the north pole must be
    // less than 90 degrees.

    double B = osg::PI_2 - lat1Rad;                       // angle between p1 and the pole
    if ( a < osg::PI_2 && b < osg::PI_2 )
        out_maxLatRad = std::max( out_maxLatRad, osg::PI_2 - asin(sin(B)*sin(a)) );
    //out_maxLatRad = a < osg::PI_2 && b < osg::PI_2 ? 
    //    osg::PI_2 - asin( sin(B)*sin(a) ) : 
    //    std::max(lat1Rad,lat2Rad);

    // flip over to the triangle formed by the south pole:
    a = osg::PI - a, b = osg::PI - b;
    B = osg::PI - B; //lat1Rad - (-osg::PI_2);

    if ( a < osg::PI_2 && b < osg::PI_2 )
        out_minLatRad = std::min( out_minLatRad, -osg::PI_2 + asin(sin(B)*sin(a)) );
    //out_minLatRad = a < osg::PI_2 && b < osg::PI_2 ? 
    //    osg::PI_2 - asin( sin(B)*sin(a) ) :
    //    std::min(lat1Rad,lat2Rad);

    //OE_INFO 
    //    << "a = " << osg::RadiansToDegrees(a)
    //    << ", b = " << osg::RadiansToDegrees(b)
    //    << ", maxLat = " << osg::RadiansToDegrees(out_maxLatRad)
    //    << ", minLat = " << osg::RadiansToDegrees(out_minLatRad)
    //    << std::endl;
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
    static osg::EllipsoidModel em; // questionable. make non-static?

    osg::Vec3d v0, v1;

    em.convertLatLongHeightToXYZ(lat1Rad, lon1Rad, 0, v0.x(), v0.y(), v0.z());
    double r0 = v0.length();
    v0.normalize();
    em.convertLatLongHeightToXYZ(lat2Rad, lon2Rad, 0, v1.x(), v1.y(), v1.z());
    double r1 = v1.length();
    v1.normalize();

    osg::Vec3d axis = v0 ^ v1;
    double angle = acos( v0 * v1 );
    osg::Quat q( angle * t, axis );

    v0 = (q * v0) * 0.5*(r0 + r1);

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
    // if dLon over 180° take shorter rhumb across 180° meridian:
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

unsigned
GeoMath::interesectLineWithSphere(const osg::Vec3d& p0,
                                  const osg::Vec3d& p1,
                                  double            R,
                                  osg::Vec3d&       out_i0,
                                  osg::Vec3d&       out_i1)
{
    unsigned hits = 0;

    // http://stackoverflow.com/questions/6533856/ray-sphere-intersection

    osg::Vec3d d = p1-p0;

    double A = d * d;
    double B = 2.0 * (d * p0);
    double C = (p0 * p0) - R*R;

    // now solve the quadratic A + B*t + C*t^2 = 0.
    double D = B*B - 4.0*A*C;
    if ( D >= 0 )
    {
        if ( osg::equivalent(D, 0.0) )
        {
            // one root (line is tangent to sphere)
            double t = -B/(2.0*A);
            //if (t >= 0.0 && t <= 1.0)
            {
                out_i0 = p0 + d*t;
                ++hits;
            }
        }
        else
        {
            // two roots (line passes through sphere twice)
            double sqrtD = sqrt(D);
            double t0 = (-B + sqrtD)/(2.0*A);
            double t1 = (-B - sqrtD)/(2.0*A);
            
            //if ( t0 >= 0.0 && t0 <= 1.0 )
            {
                out_i0 = p0 + d*t0;
                ++hits;
            }

            //if ( t1 >= 0.0 && t1 <= 1.0 )
            {
                if (hits == 0)
                    out_i0 = p0 + d*t1;
                else
                    out_i1 = p0 + d*t1;
                ++hits;
            }
        }
    }

    return hits;
}

unsigned
GeoMath::intersectLineWithPlane(const osg::Vec3d& p0,
                                const osg::Vec3d& p1,
                                const osg::Plane& plane,
                                osg::Vec3d&       out_p)
{
    osg::Vec3d V = p1-p0;
    V.normalize();
    double denom = plane.dotProductNormal(V);

    // if N*V == 0, line is parallel to the plane
    if ( osg::equivalent(denom, 0.0) )
    {
        // if p0 lies on the plane, line is coincident with plane
        // and intersections are infinite
        if ( osg::equivalent(plane.distance(p0), 0.0) )
        {
            out_p = p0;
            return 2;
        }
        else
        {
            // line does not intersect plane.
            return 0;
        }
    }
    else
    {
        // one intersection:
        double t = -(plane.dotProductNormal(p0) + plane[3])/denom;
        out_p = p0 + V*t;
        return 1;
    }
}

bool
GeoMath::isPointVisible(const osg::Vec3d& eye,
                        const osg::Vec3d& target,
                        double            R)
{
    double r2 = R*R;

    // same quadrant:
    if ( eye * target >= 0.0 )
    {
        double d2 = eye.length2();
        double horiz2 = d2 - r2;
        double dist2 = (target-eye).length2();
        if ( dist2 < horiz2 )
        {
            return true;
        }
    }

    // different quadrants:
    else
    {
        // there's a horizon between them; now see if the thing is visible.
        // find the triangle formed by the viewpoint, the target point, and 
        // the center of the earth.
        double a = (target-eye).length();
        double b = target.length();
        double c = eye.length();

        // Heron's formula for triangle area:
        double s = 0.5*(a+b+c);
        double area = 0.25*sqrt( s*(s-a)*(s-b)*(s-c) );

        // Get the triangle's height:
        double h = (2*area)/a;

        if ( h >= R )
        {
            return true;
        }
    }

    return false;
}