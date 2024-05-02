/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
