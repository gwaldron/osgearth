/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/Bounds>

using namespace osgEarth;

Bounds
osgEarth::intersectionOf(const Bounds& lhs, const Bounds& rhs)
{
    if (lhs.valid() && !rhs.valid()) return lhs;
    if (!lhs.valid() && rhs.valid()) return rhs;

    if (contains(lhs, rhs)) return rhs;
    if (contains(rhs, lhs)) return lhs;

    if (!lhs.intersects(rhs)) return Bounds();

    double xmin, xmax, ymin, ymax, zmin, zmax;

    xmin = (lhs.xMin() > rhs.xMin() && lhs.xMin() < rhs.xMax()) ? lhs.xMin() : rhs.xMin();
    xmax = (lhs.xMax() > rhs.xMin() && lhs.xMax() < rhs.xMax()) ? lhs.xMax() : rhs.xMax();
    ymin = (lhs.yMin() > rhs.yMin() && lhs.yMin() < rhs.yMax()) ? lhs.yMin() : rhs.yMin();
    ymax = (lhs.yMax() > rhs.yMin() && lhs.yMax() < rhs.yMax()) ? lhs.yMax() : rhs.yMax();
    zmin = (lhs.zMin() > rhs.zMin() && lhs.zMin() < rhs.zMax()) ? lhs.zMin() : rhs.zMin();
    zmax = (lhs.zMax() > rhs.zMin() && lhs.zMax() < rhs.zMax()) ? lhs.zMax() : rhs.zMax();

    return Bounds(xmin, ymin, zmin, xmax, ymax, zmax);
}

Bounds
osgEarth::unionOf(const Bounds& lhs, const Bounds& rhs)
{
    if (lhs.valid() && !rhs.valid()) return lhs;
    if (!lhs.valid() && rhs.valid()) return rhs;

    Bounds u;
    if (lhs.intersects(rhs))
    {
        u.xMin() = lhs.xMin() >= rhs.xMin() && lhs.xMin() <= rhs.xMax() ? lhs.xMin() : rhs.xMin();
        u.xMax() = lhs.xMax() >= rhs.xMin() && lhs.xMax() <= rhs.xMax() ? lhs.xMax() : rhs.xMax();
        u.yMin() = lhs.yMin() >= rhs.yMin() && lhs.yMin() <= rhs.yMax() ? lhs.yMin() : rhs.yMin();
        u.yMax() = lhs.yMax() >= rhs.yMin() && lhs.yMax() <= rhs.yMax() ? lhs.yMax() : rhs.yMax();
        u.zMin() = lhs.zMin() >= rhs.zMin() && lhs.zMin() <= rhs.zMax() ? lhs.zMin() : rhs.zMin();
        u.zMax() = lhs.zMax() >= rhs.zMin() && lhs.zMax() <= rhs.zMax() ? lhs.zMax() : rhs.zMax();
    }
    return u;
}

bool
osgEarth::contains(const Bounds& lhs, const Bounds& rhs)
{
    return 
        lhs.valid() && rhs.valid() &&
        lhs.xMin() <= rhs.xMin() && lhs.xMax() >= rhs.xMax() &&
        lhs.yMin() <= rhs.yMin() && lhs.yMax() >= rhs.yMax();
}
