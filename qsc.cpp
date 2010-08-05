#include <algorithm>

#include <osg/Math>
#include <osg/Vec3d>
#include <osg/Vec2d>

#include "qsc.hxx"

using namespace osg;
/** Given x,y for a cube face, return the corresponding q,r,s coords.
 */
Vec3d face2LocalCartesian(const Vec2d& face)
{
    // formulae assume that x is not less than y.
    bool swap = false;
    double x = face.x(), y = face.y();
    if (x == 0.0 && y == 0.0)
        return Vec3d(sqrt(3)/3, 0, 0);
    if (fabs(x) < fabs(y))
    {
        x = face.y();
        y = face.x();
        swap = true;
    }
    double quo = sin((PI / 12.0) * (y / x)) / (cos((PI / 12.0) * (y / x)) - 1 / sqrt(2));
    double q = 1 - x * x * (1 - 1 / sqrt(2 + quo * quo));
    double r2 = (1 - q * q) / (1 + quo * quo);
    double s2 = 1 - q * q - r2;
    Vec3d result;
    result.x() = q;
    if (x > 0)
        result.y() = sqrt(r2);
    else
        result.y() = -sqrt(r2);
    if (y > 0)
        result.z() = sqrt(s2);
    else
        result.z() = -sqrt(s2);
    if (swap)
        std::swap(result.y(), result.z());
    return result;
}

// Face 0 is centered at 0, 0 lat long or 0,0,0. faces 1-3 follow
// around the equator in direction of increasing longitude
// (east). Face 4 is the North Pole, Face 5 the South.

Vec3d face2ec(int faceNum, const Vec2d& faceCoord)
{
    Vec3d local = face2LocalCartesian(faceCoord);
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
        return Vec3d(-local.z(), -local.y(), -local.x());
    default:
        return Vec3d(0,0,0);
    }
}

void face2LatLon(int faceNum, const Vec2d& faceCoord, double& lat, double& lon)
{
    Vec3d geo = face2ec(faceNum, faceCoord);
    lon = atan2(geo.y(),geo.x());
    lat = atan2(geo.z(), sqrt(geo.x() * geo.x() + geo.y() * geo.y()));
}
