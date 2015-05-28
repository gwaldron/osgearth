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

#include "QSC"

#include <algorithm>
#include <vector>

#include <osg/Math>

#include <osgEarth/Registry>
#include <ogr_api.h>
#include <ogr_spatialref.h>

#define LC "[seamless::QSC] "

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

namespace qsc
{
// Convert lat, lon to a unit vector on a sphere

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
// face. r and s correspond to x and y face coordinates.

Vec3d xyz2qrs(const Vec3d& xyz, int face)
{
    switch (face)
    {
    case 0:
        return Vec3d(xyz.x(), xyz.y(), xyz.z());
    case 1:
        return Vec3d(xyz.y(), -xyz.x(), xyz.z());
    case 2:
        return Vec3d(-xyz.x(), -xyz.y(), xyz.z());
    case 3:
        return Vec3d(-xyz.y(), xyz.x(), xyz.z());
    case 4:
        return Vec3d(xyz.z(), xyz.y(), -xyz.x());
    case 5:
        return Vec3d(-xyz.z(), xyz.y(), xyz.x());
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
    if (equivalent(qrs[1], 0.0, 1e-11) && equivalent(qrs[1], 0.0, 1e-11))
    {
        out_x = qrs[1];
        out_y = qrs[2];
        return true;
    }
    bool swap = false;
    if (fabs(qrs[1]) < fabs(qrs[2]))
    {
        std::swap(qrs[1], qrs[2]);
        swap = true;
    }
    double sOverR = qrs[2] / qrs[1];
    double x = sqrt((1 - qrs[0])/(1 - 1/sqrt(2 + sOverR * sOverR)));
    double y = x * (PiOver12Inv * atan(sOverR)
                    - asin(qrs[2] / sqrt(2.0 * (qrs[1] * qrs[1]
                                                + qrs[2] * qrs[2]))));
    if (qrs[1] < 0.0)
        x = -x;
    if (qrs[2] < 0.0)
        y = -y;
    if (swap)
    {
        out_x = y;
        out_y = x;
    }
    else
    {
        out_x = x;
        out_y = y;
    }
    return true;
}

Vec3d face2qrs(const Vec2d& face)
{
    // formulae assume that x is not less than y.
    bool swap = false;
    double x = face.x(), y = face.y();
    if (equivalent(x, 0.0, 1e-11) && equivalent(y, 0.0, 1e-11))
        return Vec3d(sqrt33, x, y);
    if (fabs(x) < fabs(y))
    {
        x = face.y();
        y = face.x();
        swap = true;
    }
    const double yOverX = y / x;
    const double quo
        = sin(PiOver12 * yOverX) / (cos(PiOver12 * yOverX) - 1 / sqrt2);
    const double quo2 = quo * quo;
    const double q = 1 - x * x * (1 - 1 / sqrt(2 + quo2));
    const double q2 = q * q;
    double r2 = (1 - q2) / (1 + quo2);
    double s2 = 1 - q2 - r2;
    double r = sqrt(r2);
    double s = sqrt(s2);
    Vec3d result;
    result[0] = q;
    if (x > 0)
        result[1] = r;
    else
        result[1] = -r;
    if (y > 0)
        result[2] = s;
    else
        result[2] = -s;
    if (swap)
        std::swap(result[1], result[2]);
    return result;
}

// Face 0 is centered at 0, 0 lat long or 0,0,0. faces 1-3 follow
// around the equator in direction of increasing longitude
// (east). Face 4 is the North Pole, Face 5 the South.

Vec3d face2ec(int faceNum, const Vec2d& faceCoord)
{
    Vec3d local = face2qrs(faceCoord);
    switch (faceNum)
    {
    case 0:
        return Vec3d(local.x(), local.y(), local.z());
        break;
    case 1:
        return Vec3d(-local.y(), local.x(), local.z());
        break;
    case 2:
        return Vec3d(-local.x(), -local.y(), local.z());
    case 3:
        return Vec3d(local.y(), -local.x(), local.z());
    case 4:
        return Vec3d(-local.z(),local.y(), local.x());
    case 5:
        return Vec3d(local.z(), local.y(), -local.x());
    default:
        return Vec3d(0,0,0);
    }
}

bool faceCoordsToLatLon(double x, double y, int face,
                        double& out_lat_deg, double& out_lon_deg)
{
    Vec3d geo = face2ec(face, Vec2d(x, y));
    const double lon = atan2(geo.y(),geo.x());
    const double lat = atan2(geo.z(),
                             sqrt(geo.x() * geo.x() + geo.y() * geo.y()));
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

}

using namespace qsc;

bool QscFaceLocator::convertLocalToModel(const Vec3d& local, Vec3d& world) const
{
#if ((OPENSCENEGRAPH_MAJOR_VERSION <= 2) && (OPENSCENEGRAPH_MINOR_VERSION < 8))
    // OSG 2.7 bug workaround: bug fix in Locator submitted by GW
    const_cast<QscFaceLocator*>(this)->_inverse.invert( _transform );
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

bool QscFaceLocator::convertModelToLocal(const Vec3d& world, Vec3d& local) const
{
#if ((OPENSCENEGRAPH_MAJOR_VERSION <= 2) && (OPENSCENEGRAPH_MINOR_VERSION < 8))
    // OSG 2.7 bug workaround: bug fix in Locator submitted by GW
    const_cast<QscFaceLocator*>(this)->_inverse.invert( _transform );
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


QscSpatialReference::QscSpatialReference( void* handle )
    : SpatialReference(handle, "OSGEARTH", "qsc-cube",
                       "Quadralateralized Sphere Cube")
{
    //nop
}

void
QscSpatialReference::_init()
{
    SpatialReference::_init();

    _is_user_defined = true;
    _is_cube = true;
    _is_contiguous = false;
    _is_geographic = false;
    _name = "Quadralateralized Sphere Cube";
}

GeoLocator*
QscSpatialReference::createLocator(double xmin, double ymin,
                                   double xmax, double ymax,
                                    bool plate_carre) const
{
    int face;
    cubeToFace(xmin, ymin, xmax, ymax, face);

    GeoLocator* result = new QscFaceLocator( face );

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
QscSpatialReference::preTransform(double& x, double& y, void* context) const
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
QscSpatialReference::postTransform(double& x, double& y, void* context) const
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
QscSpatialReference::transformExtent(const SpatialReference* to_srs,
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

namespace
{
SpatialReference* createQscSRS()
{
    // root the cube srs with a WGS84 intermediate ellipsoid.
    std::string init = "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs";

    SpatialReference* result = 0;
    GDAL_SCOPED_LOCK;
    void* handle = OSRNewSpatialReference(0);
    if ( OSRImportFromProj4( handle, init.c_str() ) == OGRERR_NONE )
	{
        result = new QscSpatialReference( handle );
	}
	else
	{
        OE_WARN << "[osgEarth::SRS] Unable to create SRS: " << init << std::endl;
        OSRDestroySpatialReference( handle );
	}
    return result;
}


}
QscProfile::QscProfile()
    : Profile(createQscSRS(),
              0.0, 0.0, 4.0, 3.0,
              -180.0, -90.0, 180.0, 90.0,
              0,
              4, 3)
{
}

int QscProfile::getFace(const osgEarth::TileKey* key)
{
    int faceX = key->getTileX() >> key->getLevelOfDetail();
    int faceY = key->getTileY() >> key->getLevelOfDetail();
    if (faceY == 0)
        return 5;
    else if (faceY == 2)
        return 4;
    else
        return faceX;
}

void QscProfile::getIntersectingTiles(
    const GeoExtent& remoteExtent,
    std::vector<TileKey>& out_intersectingKeys ) const
{
    OE_FATAL << "QscProfile::getIntersectingTiles not implemented yet!\n";
}
}
