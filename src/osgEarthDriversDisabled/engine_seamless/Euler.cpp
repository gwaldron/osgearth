/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2010 Pelican Ventures, Inc.
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

#include "Euler"

#include <algorithm>
#include <vector>

#include <osg/Math>
#include <osg/CoordinateSystemNode>

#include <osgEarth/Registry>
#include <ogr_api.h>
#include <ogr_spatialref.h>

#include "AutoBuffer"

#define LC "[seamless::EULER] "

namespace seamless
{
using namespace std;
using namespace osg;
using namespace osgEarth;

// Constants used througout

const double PiOver12 = PI / 12.0;
const double PiOver12Inv = 12.0 / PI;
const double sqrt2 = 1.41421356237309504880; // sqrt(2)
const double sqrt33 = .5773502691896258; // sqrt(3) / 3

namespace euler
{
bool lineLineIntersect(
    const Vec3d& p1, const Vec3d& p2, const Vec3d& p3, const Vec3d& p4,
    Vec3d& pa,Vec3d& pb, double& mua, double& mub);

// Convert lat, lon to a unit vector on a sphere (direction cosine)

inline Vec3d latLon2xyz(double lat_deg, double lon_deg)
{
    Vec3d result;
    result.z() = sin(DegreesToRadians(lat_deg));
    double hyp = sqrt(1 - result.z() * result.z());
    double long_rad = DegreesToRadians(lon_deg);
    result.x() = hyp * cos(long_rad);
    result.y() = hyp * sin(long_rad);
    return result;
}

// On a boundary, return the lower-numbered face.
inline int getFace(const Vec3d& vec)
{
    double absx = fabs(vec.x());
    double absy = fabs(vec.y());
    double absz = fabs(vec.z());
    // pole faces
    if (absz > (absx + 1e-11) && absz > (absy + 1e-11))
    {
        if (vec.z() > 0.0)
            return 4;
        else
            return 5;
    }
    // One of the X faces, unless on a border
    else if (absx > absy || equivalent(absx, absy, 1e-11))
    {
        if (vec.x() > 0.0)
            return 0;
        else if (equivalent(vec.x(), -vec.y(), 1e-11))
            return 1;           // Boundary between 1 and 2
        else
            return 2;
    }
    // One of the Y faces
    else
    {
        if (vec.y() > 0.0)
            return 1;
        else
            return 3;
    }
}

// Convert unit vector to coordinates on a face. q is normal to the
// face. r and s correspond to x and y face coordinates. q is stored
// in the z member of Vec3d so that the r,s,q form a nice orthogonal
// coordinate system.

Vec3d xyz2qrs(const Vec3d& xyz, int face)
{
    switch (face)
    {
    case 0:
        return Vec3d(xyz.y(), xyz.z(), xyz.x());
    case 1:
        return Vec3d(-xyz.x(), xyz.z(), xyz.y());
    case 2:
        return Vec3d(-xyz.y(), xyz.z(), -xyz.x());
    case 3:
        return Vec3d(xyz.x(), xyz.z(), -xyz.y());
    case 4:
        return Vec3d(xyz.y(), -xyz.x(), xyz.z());
    case 5:
        return Vec3d(xyz.y(), xyz.x(), -xyz.z());
    default:
        return Vec3d(0, 0, 0);
    }
}

bool latLonToFaceCoords(double lat_deg, double lon_deg,
                        double& out_x, double& out_y, int& out_face,
                        int faceHint)
{
    if (lat_deg > 90.0 || lat_deg < -90.0
        || lon_deg < -180.0 || lon_deg > 180.0)
        return false;
    Vec3d xyz = latLon2xyz(lat_deg, lon_deg);
    out_face = faceHint >= 0 ? faceHint : getFace(xyz);
    Vec3d qrs = xyz2qrs(xyz, out_face);

    double xang = atan2(qrs[0], qrs[2]);
    double yang = atan2(qrs[1], qrs[2]);
    out_x = xang / osg::PI_4;
    out_y = yang / osg::PI_4;
    return true;
}

Vec3d face2qrs(const Vec2d& face)
{
    double xang = face[0] * osg::PI_4;
    double yang = face[1] * osg::PI_4;
    double sx = sin(xang), cx = cos(xang);
    double ty = tan(yang);
    // phi is a latitude measure, in the longitudinal plane
    double tanPhi = cx * ty;
    double radical = sqrt(1 + tanPhi * tanPhi);
    double c = 1.0 / radical;
    Vec3d result;
    double b = tanPhi * c;      // b gets the right sign
    result.x() = c * sx;
    result.y() = b;
    result.z() = c * cx;
    return result;
}

Vec3d qrs2xyz(const Vec3d& local, int face)
{
    switch (face)
    {
    case 0:
        return Vec3d(local.z(), local.x(), local.y());
        break;
    case 1:
        return Vec3d(-local.x(), local.z(), local.y());
        break;
    case 2:
        return Vec3d(-local.z(), -local.x(), local.y());
    case 3:
        return Vec3d(local.x(), -local.z(), local.y());
    case 4:
        return Vec3d(-local.y(),local.x(), local.z());
    case 5:
        return Vec3d(local.y(), local.x(), -local.z());
    default:
        return Vec3d(0,0,0);
    }
}

// Face 0 is centered at 0, 0 lat long or 0,0,0. faces 1-3 follow
// around the equator in direction of increasing longitude
// (east). Face 4 is the North Pole, Face 5 the South.

Vec3d face2dc(int faceNum, const Vec2d& faceCoord)
{
    Vec3d local = face2qrs(faceCoord);
    return qrs2xyz(local, faceNum);
}

bool faceCoordsToLatLon(double x, double y, int face,
                        double& out_lat_deg, double& out_lon_deg)
{
    double lat, lon;
    const double l = x * osg::PI_4;
    const double ty = tan(y * osg::PI_4);
    if (face < 4)
    {
        lon = face * osg::PI_2 + l;
        lon = fmod(lon + osg::PI, 2.0 * osg::PI) - osg::PI;
        lat = atan(cos(l) * ty);
    }
    else
    {
        const double tx = tan(x * osg::PI_4);
        lat = osg::PI_2 - atan(sqrt(tx * tx + ty * ty));
        if (face == 5)
        {
            lon = atan2(tx, ty);
            lat = -lat;
        }
        else
        {
            lon = atan2(tx, -ty);
        }
    }
    out_lon_deg = RadiansToDegrees(lon);
    out_lat_deg = RadiansToDegrees(lat);
    return true;
}

bool cubeToFace(double& in_out_x, double& in_out_y, int& out_face )
{
    double x, y;
    if (in_out_x > 1.0 + 1e-11)
    {
        double face = floor(in_out_x);
        x = in_out_x - face;
        if (x < 1e-11)
        {
            face += -1.0;
            x += 1.0;
        }
        y = in_out_y - 1.0;
        out_face = static_cast<int>(face);
    }
    else
    {
        if (in_out_y > 2.0 + 1e-11)
        {
            out_face = 4;
            y = in_out_y - 2.0;
        }
        else if (in_out_y < 1.0 + 1e-11)
        {
            out_face = 5;
            y = in_out_y;
        }
        else
        {
            out_face = 0;
            y = in_out_y - 1.0;
        }
        x = in_out_x;
    }
    in_out_x = x * 2.0 - 1.0;
    in_out_y = y * 2.0 - 1.0;
    return true;
}

bool cubeToFace(double& in_out_xmin, double& in_out_ymin,
                double& in_out_xmax, double& in_out_ymax,
                int& out_face)
{
    double xmin, xmax, ymin, ymax;
    if (in_out_ymin > 1.0 - 1e-11 && in_out_ymax < 2.0 + 1e-11)
    {
        double faceMin = floor(in_out_xmin + 1e-11);
        double faceMax = floor(in_out_xmax - 1e-11);
        if (faceMin != faceMax)
        {
            OE_WARN << LC << "Min face <> Max face!\n";
            return false;
        }
        xmin = in_out_xmin - faceMin;
        xmax = in_out_xmax - faceMin;
        ymin = in_out_ymin - 1.0;
        ymax = in_out_ymax - 1.0;
        out_face = static_cast<int>(faceMin);
    }
    else if (in_out_ymin > 2.0 - 1e-11 && in_out_ymax > 2.0 + 1e-11)
    {
        out_face = 4;
        ymin = in_out_ymin - 2.0;
        ymax = in_out_ymax - 2.0;
        xmin = in_out_xmin;
        xmax = in_out_xmax;
    }
    else if (in_out_ymax < 1.0 + 1e-11)
    {
        out_face = 5;
        ymin = in_out_ymin;
        ymax = in_out_ymax;
        xmin = in_out_xmin;
        xmax = in_out_xmax;
    }
    else
    {
        OE_WARN << LC << "can't determine face for ("
                << in_out_xmin << ", " << in_out_ymin << "), ("
                << in_out_xmax << ", " << in_out_ymax << ")\n";
        return false;
    }
    in_out_xmin = xmin * 2.0 - 1.0;
    in_out_xmax = xmax * 2.0 - 1.0;
    in_out_ymin = ymin * 2.0 - 1.0;
    in_out_ymax = ymax * 2.0 - 1.0;
    return true;
}

bool faceToCube(double& in_out_x, double& in_out_y, int face)
{
    double x = (in_out_x + 1.0) * .5;
    double y = (in_out_y + 1.0) * .5;
    if (face < 4)
    {
        in_out_x = x + face;
        in_out_y = y + 1.0;
    }
    else
    {
        in_out_x = x;
        if (face == 4)
            in_out_y = y + 2.0;
        else
            in_out_y = y;
    }
    return true;
}

double arcLength(const Vec2d& coord1, const Vec2d& coord2, int face)
{
    if (coord1.x() != coord2.x() && coord1.y() != coord2.y())
    {
        Vec3d geo1 = face2dc(face, coord1);
        Vec3d geo2 = face2dc(face, coord2);
        Vec3d norm = geo1 ^ geo2;
        return atan2(norm.length(), geo1 * geo2) * WGS_84_RADIUS_EQUATOR;
    }
    double x1, y1, x2, y2;
    if (coord1.x() == coord2.x())
    {
        x1 = coord1.x() * PI_4;  y1 = coord1.y() * PI_4;
        x2 = coord2.x() * PI_4;  y2 = coord2.y() * PI_4;
    }
    else
    {
        x1 = coord1.y() * PI_4;  y1 = coord1.x() * PI_4;
        x2 = coord2.y() * PI_4;  y2 = coord2.x() * PI_4;
    }
    double tanPhi1 = cos(x1) * tan(y1);
    double tanPhi2 = cos(x2) * tan(y2);
    // tan(phi2 - phi1) = (tan(phi2) - tan(hi1)) / (1 + tan(phi2) * tan(phi1))
    return fabs(atan2(tanPhi2 - tanPhi1, 1 + tanPhi2 * tanPhi1))
        * WGS_84_RADIUS_EQUATOR;

}

// For debugging
// Find the point closest to P3 on the line segment from P1 to P2
static osg::Vec3d closestPointOnLine(const osg::Vec3d &p1,
                                     const osg::Vec3d& p2,
                                     const osg::Vec3d& p3)
{
    Vec3d vec = p2 - p1;
    double len2 = vec.length2();
    if (equivalent(len2, 0))
        return p1;
    double u = ((p3 - p1) * vec) / len2;
    if (u <= 0.0)
        return p1;
    else if (u >= 1.0)
        return p2;
    else
        return p1 + vec * u;
}

double distanceToLine(const Vec3d& p1, const Vec3d& p2, const Vec3d& p3)
{
    Vec3d pt = closestPointOnLine(p1, p2, p3);
    return (p3 - pt).length();
}

double distanceToSegment(const Vec3d& p,
                         const Vec3d& geo1, const Vec3d& geo2,
                         const Vec3d& norm)
{
    // Find the distance to the closet point on the circle that
    // contains the segment. If that point lies on the segment, that's
    // the shortest distance; otherwise, the distance to one of the
    // end points is the shortest.

    // Project p into plane of circle
    Vec3d q = p - norm * (norm * p);
    // If q = (0, 0, 0) -- the center of the circle -- then all points
    // on the circle are equidistant.
    // qnorm will be on unit circle
    const double r = WGS_84_RADIUS_EQUATOR;
    if (equivalent(q.length2(), 0))
    {
        return sqrt(r * r + p.length2());
    }
    Vec3d qnorm = q / q.length();

    // Vec3d x = q * r / q.length();
    const Vec3d zero;
    Vec3d end1, end2;
    double mua, mub;
    if (lineLineIntersect(zero, qnorm, geo1, geo2, end1, end2, mua, mub)
        && mub >= 0.0 && mub <= 1.0)
    {
        return (p - qnorm * r).length();
    }
    else
    {
        return minimum((p - geo1 * r).length(), (p - geo2 * r).length());
    }
}

// The normal will point towards +s or +r

Vec3d getNormalToSegment(const Vec2d& coord1, const Vec2d& coord2, int face)
{
    if (coord1.x() == coord2.x())
    {
        double xang = coord1.x() * PI_4;
        double sx = sin(xang), cx = cos(xang);
        Vec3d qrsNormal(cx, 0.0, -sx);
        return qrs2xyz(qrsNormal, face);
    }
    else
    {
        double yang = coord1.y() * PI_4;
        double sy = sin(yang), cy = cos(yang);
        Vec3d qrsNormal(0.0, cy, -sy);
        return qrs2xyz(qrsNormal, face);
    }
}

double distanceToSegment(const Vec3d& p,
                         const Vec2d& coord1, const Vec2d& coord2, int face)
{
    Vec3d norm = getNormalToSegment(coord1, coord2, face);
    Vec3d geo1 = face2dc(face, coord1);
    Vec3d geo2 = face2dc(face, coord2);
    return distanceToSegment(p, geo1, geo2, norm);
}

/*
   Calculate the line segment PaPb that is the shortest route between
   two lines P1P2 and P3P4. Calculate also the values of mua and mub where
      Pa = P1 + mua (P2 - P1)
      Pb = P3 + mub (P4 - P3)
   Return FALSE if no solution exists.

   Ripped off from http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline3d/
*/
bool lineLineIntersect(
   const Vec3d& p1, const Vec3d& p2, const Vec3d& p3, const Vec3d& p4,
   Vec3d& pa,Vec3d& pb, double& mua, double& mub)
{
    Vec3d p13, p43, p21;
    double d1343, d4321, d1321, d4343, d2121;
    double numer, denom;

    p13 = p1 - p3;
    p43 = p4 - p3;
    if (equivalent(p43.length2(), 0.0, 1e-11))
        return false;
    p21 = p2 - p1;
    if (equivalent(p21.length2(), 0.0, 1e-11))
        return false;

    d1343 = p13.x() * p43.x() + p13.y() * p43.y() + p13.z() * p43.z();
    d4321 = p43.x() * p21.x() + p43.y() * p21.y() + p43.z() * p21.z();
    d1321 = p13.x() * p21.x() + p13.y() * p21.y() + p13.z() * p21.z();
    d4343 = p43.x() * p43.x() + p43.y() * p43.y() + p43.z() * p43.z();
    d2121 = p21.x() * p21.x() + p21.y() * p21.y() + p21.z() * p21.z();

    denom = d2121 * d4343 - d4321 * d4321;
    if (equivalent(denom, 0.0, 1e-11))
        return false;
    numer = d1343 * d4321 - d1321 * d4343;

    mua = numer / denom;
    mub = (d1343 + d4321 * (mua)) / d4343;

    pa = p1 + p21 * mua;
    pb = p3 + p43 * mub;

    return true;
}

}

using namespace euler;

bool EulerFaceLocator::convertLocalToModel(const Vec3d& local, Vec3d& world) const
{
#if ((OPENSCENEGRAPH_MAJOR_VERSION <= 2) && (OPENSCENEGRAPH_MINOR_VERSION < 8))
    // OSG 2.7 bug workaround: bug fix in Locator submitted by GW
    const_cast<EulerFaceLocator*>(this)->_inverse.invert( _transform );
#endif
    if ( _coordinateSystemType == GEOCENTRIC )
    {
        //Convert the NDC coordinate into face space
        osg::Vec3d faceCoord = local * _transform;

        double lat_deg, lon_deg;
        faceCoordsToLatLon(faceCoord.x(), faceCoord.y(), _face,
                           lat_deg, lon_deg);

        //OE_NOTICE << "LatLon=" << latLon <<  std::endl;

        // convert to geocentric:
        _ellipsoidModel->convertLatLongHeightToXYZ(
            DegreesToRadians(lat_deg),
            DegreesToRadians(lon_deg),
            local.z(),
            world.x(), world.y(), world.z());

        return true;
    }
    return true;
}

bool EulerFaceLocator::convertModelToLocal(const Vec3d& world, Vec3d& local) const
{
#if ((OPENSCENEGRAPH_MAJOR_VERSION <= 2) && (OPENSCENEGRAPH_MINOR_VERSION < 8))
    // OSG 2.7 bug workaround: bug fix in Locator submitted by GW
    const_cast<EulerFaceLocator*>(this)->_inverse.invert( _transform );
#endif

    switch(_coordinateSystemType)
    {
    case(GEOCENTRIC):
        {
            double longitude, latitude, height;

            _ellipsoidModel->convertXYZToLatLongHeight
                (world.x(), world.y(), world.z(), latitude, longitude, height );

            int face=-1;
            double x, y;

            double lat_deg = RadiansToDegrees(latitude);
            double lon_deg = RadiansToDegrees(longitude);

            bool success = latLonToFaceCoords(lat_deg, lon_deg, x, y, face,
                                              _face);

            if (!success)
            {
                OE_NOTICE << LC << "Couldn't convert to face coords\n";
            }
            if (face != static_cast<int>(_face))
            {
                OE_NOTICE << LC
                    << "Face should be " << _face << " but is " << face
                    << ", lat = " << lat_deg
                    << ", lon = " << lon_deg
                    << "\n";
            }

            local = Vec3d(x, y, height) * _inverse;
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


EulerSpatialReference::EulerSpatialReference( void* handle )
    : SpatialReference(handle, "OSGEARTH", "euler-cube",
                       "Euler Cube")
{
    //nop
}

void
EulerSpatialReference::_init()
{
    SpatialReference::_init();

    _is_user_defined = true;
    _is_cube = true;
    _is_contiguous = false;
    _is_geographic = false;
    _name = "Euler Cube";
}

GeoLocator*
EulerSpatialReference::createLocator(double xmin, double ymin,
                                   double xmax, double ymax,
                                    bool plate_carre) const
{
    int face;
    cubeToFace(xmin, ymin, xmax, ymax, face);

    GeoLocator* result = new EulerFaceLocator( face );

    osg::Matrixd transform;
    transform.set(
        xmax-xmin, 0.0,       0.0, 0.0,
        0.0,       ymax-ymin, 0.0, 0.0,
        0.0,       0.0,       1.0, 0.0,
        xmin,      ymin,      0.0, 1.0);
    result->setTransform( transform );

    return result;
}

bool
EulerSpatialReference::preTransform(double& x, double& y, void* context) const
{
    // Convert the incoming points from cube => face => lat/long.
    int face;
    if ( !cubeToFace(x, y, face) )
    {
        OE_WARN << LC << "Failed to convert (" << x << "," << y << ") into face coordinates." << std::endl;
        return false;
    }

    double lat_deg, lon_deg;
    bool success = faceCoordsToLatLon( x, y, face, lat_deg, lon_deg );
    if (!success)
    {
        OE_WARN << LC << "Could not transform face coordinates to lat lon" << std::endl;
        return false;
    }
    x = lon_deg;
    y = lat_deg;
    return true;
}

bool
EulerSpatialReference::postTransform(double& x, double& y, void* context) const
{
    //Convert the incoming points from lat/lon back to face coordinates
    int face;
    double out_x, out_y;

    // convert from lat/long to x/y/face
    bool success = latLonToFaceCoords(y, x, out_x, out_y, face);
    if (!success)
    {
        OE_WARN << LC << "Could not transform face coordinates to lat lon" << std::endl;
        return false;
    }

    //TODO: what to do about boundary points?

    if ( !faceToCube(out_x, out_y, face))
    {
        OE_WARN << LC << "fromFace(" << out_x << "," << out_y << "," << face << ") failed" << std::endl;
        return false;
    }

    x = out_x;
    y = out_y;

    return true;
}

bool
EulerSpatialReference::transform(double x, double y,
                                 const SpatialReference* to_srs,
                                 double& out_x, double& out_y,
                                 void* context) const
{
    if ( !_initialized )
        const_cast<EulerSpatialReference*>(this)->init();
    if (!to_srs->isEquivalentTo(getGeographicSRS()))
        return SpatialReference::transform(x, y, to_srs, out_x, out_y, context);
    if (EulerSpatialReference::preTransform(x, y, context))
    {
        out_x = x;
        out_y = y;
        return true;
    }
    else
    {
        return false;
    }
}

bool EulerSpatialReference::transformPoints(const SpatialReference* to_srs, 
                                            double* x, double *y,
                                            unsigned int numPoints,
                                            void* context,
                                            bool ignore_errors) const
{
        if ( !_initialized )
            const_cast<EulerSpatialReference*>(this)->init();
        if (!to_srs->isEquivalentTo(getGeographicSRS()))
            return SpatialReference::transformPoints(to_srs, x, y, numPoints,
                                                     context, ignore_errors);
        bool success = true;
        for (unsigned int i = 0; i < numPoints; ++i)
        {
            bool result
                = EulerSpatialReference::preTransform(x[i], y[i], context);
            success = success && result;
        }
        return success;
}

bool
EulerSpatialReference::transformExtent(const SpatialReference* to_srs,
                                     double& in_out_xmin,
                                     double& in_out_ymin,
                                     double& in_out_xmax,
                                     double& in_out_ymax,
                                     void* context ) const
{
    // note: this method only works when the extent is isolated to one
    // face of the cube. If you want to transform an arbitrary extent,
    // you need to break it up into separate extents for each cube face.
    bool ok = true;

    double face_xmin = in_out_xmin, face_ymin = in_out_ymin;
    double face_xmax = in_out_xmax, face_ymax = in_out_ymax;

    int face;
    if (!cubeToFace(face_xmin, face_ymin, face_xmax, face_ymax, face))
    {
        OE_WARN << LC << "extent (" << in_out_xmin << ", " << in_out_ymin
                << ")=>(" << in_out_xmax << ", " << in_out_ymax <<
            ") crosses faces\n";
        return false;
    }
    // The polar and equatorial faces behave the same way, except if
    // an extent crosses the pole.

    double lonmin, lonmax, latmin, latmax;
    // if the extent crosses the center axes of the face, then,
    // due to the curvy nature of the projection, the maximim /
    // minimum will be at the crossing. So we may need to sample
    // up to 8 points.
    double lats[8], lons[8];
    int numSamples = 4;
    faceCoordsToLatLon(face_xmin, face_ymin, face, lats[0], lons[0]);
    faceCoordsToLatLon(face_xmax, face_ymin, face, lats[1], lons[1]);
    faceCoordsToLatLon(face_xmin, face_ymax, face, lats[2], lons[2]);
    faceCoordsToLatLon(face_xmax, face_ymax, face, lats[3], lons[3]);

    if ((face_xmin < 0 && face_xmax > 0))
    {
        faceCoordsToLatLon(0.0, face_ymin, face,
                           lats[numSamples], lons[numSamples]);
        faceCoordsToLatLon(0.0, face_ymax, face,
                           lats[numSamples + 1], lons[numSamples + 1]);
        numSamples += 2;

    }
    if ((face_ymin < 0 && face_ymax > 0))
    {
        faceCoordsToLatLon(face_xmin, 0.0, face,
                           lats[numSamples], lons[numSamples]);
        faceCoordsToLatLon(face_xmax, 0.0, face,
                           lats[numSamples + 1], lons[numSamples + 1]);
        numSamples += 2;

    }
    // keep extent longitudes consistent. If the max latitude lies on
    // the date line, its value should be 180, not -180. The poles are
    // especially interesting...
    if (face == 2 && face_xmax == 0.0)
    {
        lons[1] = 180.0;
        lons[3] = 180.0;
    }
    else if ((face == 4 && face_ymax > 0.0)
             || (face == 5 && face_ymax <= 0.0))
    {
        if (face_xmin == 0.0)
        {
            lons[0] = 180.0;
            lons[2] = 180.0;
        }
        else if (face_xmax == 0.0)
        {
            lons[1] = -180.0;
            lons[3] = -180.0;
        }
    }
    if ((face == 4 || face == 5) && face_ymax == 0.0)
    {
        if  (face_xmax == 0.0)
            lons[3] = -90;
        else if (face_xmin == 0.0)
            lons[2] = 90;
            
    }
    lonmin = *min_element(&lons[0], &lons[numSamples]);
    latmin = *min_element(&lats[0], &lats[numSamples]);
    lonmax = *max_element(&lons[0], &lons[numSamples]);
    latmax = *max_element(&lats[0], &lats[numSamples]);
    // Does the extent cross one of the poles?
    if ((face == 4 || face == 5) && numSamples == 8)
    {
        lonmin = -180.0;
        lonmax = 180.0;
        if (face == 4)
            latmax = 90.0;
        else
            latmin = -90.0;
    }
    // Check for Date Line crossing
    else if (face_xmin < 0 && face_xmax > 0
             && (face == 2 || (face == 4 && face_ymin >= 0)
                 || (face == 5 && face_ymax <= 0)))
    {
        std::swap(lonmin, lonmax);
    }
    if (to_srs->isGeographic())
    {
        in_out_xmin = lonmin;
        in_out_xmax = lonmax;
        in_out_ymin = latmin;
        in_out_ymax = latmax;
    }
    else
    {
        bool ok1 = transform(lonmin, latmin, to_srs, in_out_xmin, in_out_ymin,
                             context);
        bool ok2 = transform(lonmax, latmax, to_srs, in_out_xmax, in_out_ymax,
                             context);
        ok = ok1 && ok2;
    }
    return ok;
}

// This has been written to minimize the number of transcendental
// function calls -- as well as other expensive operations -- in the
// inner loop.
bool EulerSpatialReference::transformExtentPoints(
    const SpatialReference* to_srs,
    double in_xmin, double in_ymin,
    double in_xmax, double in_ymax,
    double* x, double *y,
    unsigned int numx, unsigned int numy,
    void* context, bool ignore_errors ) const
{
    if ( !_initialized )
        const_cast<EulerSpatialReference*>(this)->init();
    int face;

    // Punt if the extent covers more than one face (doesn't happen
    // normally).
    if (!(to_srs->isEquivalentTo(getGeographicSRS())
          && cubeToFace(in_xmin, in_ymin, in_xmax, in_ymax, face)))
        return SpatialReference::transformExtentPoints(
            to_srs, in_xmin, in_ymin, in_xmax, in_ymax, x, y,
            numx, numy, context, ignore_errors);
    const double dx = (in_xmax - in_xmin) / (numx - 1);
    const double dy = (in_ymax - in_ymin) / (numy - 1);
    unsigned pixel = 0;
    // Cache values for and tan(y)
    AutoBuffer<double, 256> tany(numy);
    // induction variables for column and row counters
    double fc = 0.0, fr = 0.0;
    for (unsigned r = 0; r < numy; ++r, ++fr)
        tany[r] = tan((in_ymin + fr * dy) * osg::PI_4);
    if (face < 4)
    {
        double lonBase = face * osg::PI_2;
        for (unsigned c = 0; c < numx; ++c, ++fc)
        {
            const double l = (in_xmin + fc * dx) * osg::PI_4;
            double lon = lonBase + l;
            lon = fmod(lon + osg::PI, 2.0 * osg::PI) - osg::PI;
            const double rlon = RadiansToDegrees(lon);
            const double cosl = cos(l);
            for (unsigned r = 0; r < numy; ++r)
            {
                const double lat = atan(cosl * tany[r]);
                x[pixel] = rlon;
                y[pixel] = RadiansToDegrees(lat);
                pixel++;
            }
        }
    }
    else
    {
        // The pole faces are the same, except for a change of sign in
        // the latitude and longitude calculations.
        const double sgn = face == 4 ? -1.0 : 1.0;
        for (unsigned c = 0; c < numx; ++c, ++fc)
        {
            const double l = (in_xmin + fc * dx) * osg::PI_4;
            const double tx = tan(l);
            const double tx2 = tx * tx;
            for (unsigned r = 0; r < numy; ++r)
            {
                const double ty = tany[r];
                const double lon = atan2(tx, sgn * ty);
                const double lat = (atan(sqrt(tx2 + ty * ty)) - osg::PI_2)
                    * sgn;
                x[pixel] = RadiansToDegrees(lon);
                y[pixel] = RadiansToDegrees(lat);
                pixel++;
            }
        }
    }
    const int numPixels = numx * numy;
    // Fixups for boundary lieing on date line
    if (face == 2 && in_xmax == 0.0)
    {
        for (int pixel = numx - 1; pixel < numPixels; pixel += numx)
            x[pixel] = 180;
    }
    else if ((face == 4 && in_ymax > 0.0)
             || (face == 5 && in_ymax <= 0.0))
    {
        double val;
        int startPix = -1;
        if (in_xmin == 0.0)
        {
            val = 180.0;
            startPix = 0;
        }
        else if (in_xmax == 0.0)
        {
            val = -180.0;
            startPix = numx - 1;
        }
        if (startPix > 0)
            for (int pixel = startPix; pixel < numPixels; pixel += numx)
                x[pixel] = val;
    }
    // pole case
    if ((face == 4 || face == 5) && in_ymax == 0.0)
    {
        if (in_xmax == 0.0)
            x[numPixels - 1] = -90.0;
        else if (in_xmin == 0.0)
            x[numx * (numy - 1)] = 90;
    }
    return true;
}

namespace
{
SpatialReference* createEulerSRS()
{
    // root the cube srs with a WGS84 intermediate ellipsoid.
    std::string init = "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs";

    SpatialReference* result = 0;
    GDAL_SCOPED_LOCK;
    void* handle = OSRNewSpatialReference(0);
    if ( OSRImportFromProj4( handle, init.c_str() ) == OGRERR_NONE )
	{
        result = new EulerSpatialReference( handle );
	}
	else
	{
        OE_WARN << "[osgEarth::SRS] Unable to create SRS: " << init << std::endl;
        OSRDestroySpatialReference( handle );
	}
    return result;
}
}

// Hack to get euler-cube into the spatial reference cache
class CacheInitializer
{
public:
    CacheInitializer()
    {
        EulerSpatialReference::getSpatialReferenceCache()["euler-cube"]
            = createEulerSRS();
    }
};

namespace
{
CacheInitializer s_cacheInitializer;
}

EulerProfile::EulerProfile()
    : Profile(createEulerSRS(),
              0.0, 0.0, 4.0, 4.0,
              -180.0, -90.0, 180.0, 90.0,
              0,
              1, 1)
{
}

int EulerProfile::getFace(const TileKey& key)
{
    int shiftVal = key.getLevelOfDetail() - 2;
    int faceX = key.getTileX() >> shiftVal;
    int faceY = key.getTileY() >> shiftVal;
    if (faceY == 1)
        return 4;
    else if (faceY == 3)
        return 5;
    else
        return faceX;
}

void EulerProfile::getIntersectingTiles(
    const GeoExtent& remoteExtent,
    std::vector<TileKey>& out_intersectingKeys ) const
{
    OE_FATAL << "EulerProfile::getIntersectingTiles not implemented yet!\n";
}

}
