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

//------------------------------------------------------------------------

Bounds
osgEarth::intersection(const Bounds& lhs, const Bounds& rhs)
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

#if 0
void
Bounds::expandBy( double x, double y )
{
    osg::BoundingBoxImpl<osg::Vec3d>::expandBy( x, y, 0 );
}

void
Bounds::expandBy( double x, double y, double z )
{
    osg::BoundingBoxImpl<osg::Vec3d>::expandBy( x, y, z );
}

void
Bounds::expandBy( const Bounds& rhs )
{
    osg::BoundingBoxImpl<osg::Vec3d>::expandBy( rhs );
}

Bounds 
Bounds::unionWith(const Bounds& rhs) const
{
    if ( valid() && !rhs.valid() ) return *this;
    if ( !valid() && rhs.valid() ) return rhs;

    Bounds u;
    if ( intersects(rhs) ) {
        u.xMin() = xMin() >= rhs.xMin() && xMin() <= rhs.xMax() ? xMin() : rhs.xMin();
        u.xMax() = xMax() >= rhs.xMin() && xMax() <= rhs.xMax() ? xMax() : rhs.xMax();
        u.yMin() = yMin() >= rhs.yMin() && yMin() <= rhs.yMax() ? yMin() : rhs.yMin();
        u.yMax() = yMax() >= rhs.yMin() && yMax() <= rhs.yMax() ? yMax() : rhs.yMax();
        u.zMin() = zMin() >= rhs.zMin() && zMin() <= rhs.zMax() ? zMin() : rhs.zMin();
        u.zMax() = zMax() >= rhs.zMin() && zMax() <= rhs.zMax() ? zMax() : rhs.zMax();
    }
    return u;
}

Bounds
Bounds::intersectionWith(const Bounds& rhs) const 
{
    if ( valid() && !rhs.valid() ) return *this;
    if ( !valid() && rhs.valid() ) return rhs;

    if ( this->contains(rhs) ) return rhs;
    if ( rhs.contains(*this) ) return *this;

    if ( !intersects(rhs) ) return Bounds();

    double xmin, xmax, ymin, ymax;

    xmin = ( xMin() > rhs.xMin() && xMin() < rhs.xMax() ) ? xMin() : rhs.xMin();
    xmax = ( xMax() > rhs.xMin() && xMax() < rhs.xMax() ) ? xMax() : rhs.xMax();
    ymin = ( yMin() > rhs.yMin() && yMin() < rhs.yMax() ) ? yMin() : rhs.yMin();
    ymax = ( yMax() > rhs.yMin() && yMax() < rhs.yMax() ) ? yMax() : rhs.yMax();

    return Bounds(xmin, ymin, xmax, ymax);
}

double
Bounds::width() const {
    return xMax()-xMin();
}

double
Bounds::height() const {
    return yMax()-yMin();
}

double
Bounds::depth() const {
    return zMax()-zMin();
}

osg::Vec2d
Bounds::center2d() const {
    osg::Vec3d c = center();
    return osg::Vec2d( c.x(), c.y() );
}

double
Bounds::radius2d() const {
    return (center2d() - osg::Vec2d(xMin(),yMin())).length();
}

double
Bounds::area2d() const {
    return isValid() ? width() * height() : -1.0;
}

std::string
Bounds::toString() const
{
    return Stringify() << "(" << xMin() << "," << yMin() << " => " << xMax() << "," << yMax() << ")";
}

void
Bounds::transform( const SpatialReference* from, const SpatialReference* to )
{
    from->transformExtentToMBR( to, _min.x(), _min.y(), _max.x(), _max.y() );
}

void
Bounds::set(double xmin, double ymin, double xmax, double ymax)
{
    osg::BoundingBoxd::set(xmin, ymin, -DBL_MAX, xmax, ymax, DBL_MAX);
}

void
Bounds::set(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax)
{
    osg::BoundingBoxd::set(xmin, ymin, zmin, xmax, ymax, zmax);
}
#endif
