/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
#include <osgEarth/Math>

using namespace osgEarth;

//------------------------------------------------------------------------

namespace
{
    // ignores Z.
    bool intersectRaysXY(const osg::Vec3d& p0, const osg::Vec3d& d0,
                         const osg::Vec3d& p1, const osg::Vec3d& d1,
                         osg::Vec3d& out_p,
                         double&     out_u,
                         double&     out_v)
    {
        static const double epsilon = 0.001;

        double det = d0.y()*d1.x() - d0.x()*d1.y();
        if ( osg::equivalent(det, 0.0, epsilon) )
            return false; // parallel

        out_u = (d1.x()*(p1.y()-p0.y())+d1.y()*(p0.x()-p1.x()))/det;
        out_v = (d0.x()*(p1.y()-p0.y())+d0.y()*(p0.x()-p1.x()))/det;
        out_p = p0 + d0*out_u;
        return true;
    }
}

bool
Line2d::intersect( const Line2d& rhs, osg::Vec4d& out ) const
{
    double u, v;
    osg::Vec3d temp;
    bool ok = intersectRaysXY(_a, (_b-_a), rhs._a, (rhs._b-rhs._a), temp, u, v);
    out.set( temp.x(), temp.y(), temp.z(), 1.0 );
    return ok;
}

bool
Line2d::intersect( const Line2d& rhs, osg::Vec3d& out ) const
{
    double u, v;
    return intersectRaysXY(_a, (_b-_a), rhs._a, (rhs._b-rhs._a), out, u, v);
}

bool
Line2d::intersect( const Segment2d& rhs, osg::Vec3d& out ) const
{
    double u, v;
    bool ok = intersectRaysXY(_a, (_b-_a), rhs._a, (rhs._b-rhs._a), out, u, v);
    return ok && v >= 0.0 && v <= 1.0;
}

bool
Line2d::intersect( const Ray2d& rhs, osg::Vec3d& out ) const
{
    double u, v;
    bool ok = intersectRaysXY(_a, (_b-_a), rhs._a, rhs._dv, out, u, v);
    return ok && v >= 0.0;
}

bool
Line2d::intersect( const Line2d& rhs, osg::Vec2d& out ) const
{
    osg::Vec3d out3;
    bool ok = intersect( rhs, out3 );
    out.set(out3.x(), out3.y());
    return ok;
}

bool
Line2d::intersect( const Segment2d& rhs, osg::Vec2d& out ) const
{
    osg::Vec3d out3;
    bool ok = intersect( rhs, out3 );
    out.set(out3.x(), out3.y());
    return ok;
}

bool
Line2d::intersect( const Ray2d& rhs, osg::Vec2d& out ) const
{
    osg::Vec3d out3;
    bool ok = intersect( rhs, out3 );
    out.set(out3.x(), out3.y());
    return ok;
}

bool
Line2d::isPointOnLeft( const osg::Vec3d& p ) const
{
    return ((_b-_a) ^ (p-_a)).z() >= 0.0;
}

bool
Line2d::isPointOnLeft( const osg::Vec2d& p ) const
{
    return ((_b-_a) ^ (osg::Vec3d(p.x(),p.y(),0)-_a)).z() >= 0.0;
}

//--------------------------------------------------------------------------

bool
Ray2d::intersect( const Line2d& rhs, osg::Vec3d& out ) const
{
    double u, v;
    bool ok = intersectRaysXY(_a, _dv, rhs._a, (rhs._b-rhs._a), out, u, v);
    return ok && u >= 0.0;
}

bool
Ray2d::intersect( const Segment2d& rhs, osg::Vec3d& out ) const
{
    double u, v;
    bool ok = intersectRaysXY(_a, _dv, rhs._a, (rhs._b-rhs._a), out, u, v);
    return ok && u >= 0.0 && v >= 0.0 && v <= 1.0;
}

bool
Ray2d::intersect( const Ray2d& rhs, osg::Vec3d& out ) const
{
    double u, v;
    bool ok = intersectRaysXY(_a, _dv, rhs._a, rhs._dv, out, u, v);
    return ok && u >= 0.0 && v >= 0.0;
}

bool
Ray2d::intersect( const Line2d& rhs, osg::Vec2d& out ) const
{
    osg::Vec3d out3;
    bool ok = intersect( rhs, out3 );
    out.set(out3.x(), out3.y());
    return ok;
}

bool
Ray2d::intersect( const Segment2d& rhs, osg::Vec2d& out ) const
{
    osg::Vec3d out3;
    bool ok = intersect( rhs, out3 );
    out.set(out3.x(), out3.y());
    return ok;
}

bool
Ray2d::intersect( const Ray2d& rhs, osg::Vec2d& out ) const
{
    osg::Vec3d out3;
    bool ok = intersect( rhs, out3 );
    out.set(out3.x(), out3.y());
    return ok;
}

bool
Ray2d::isPointOnLeft( const osg::Vec3d& p ) const
{
    return (_dv ^ (p-_a)).z() >= 0.0;
}

bool
Ray2d::isPointOnLeft( const osg::Vec2d& p ) const
{
    return (_dv ^ (osg::Vec3d(p.x(),p.y(),0)-_a)).z() >= 0.0;
}

double
Ray2d::angle(const Segment2d& rhs) const
{
    osg::Vec2d v0( _dv.x(), _dv.y() );
    v0.normalize();
    osg::Vec2d v1(rhs._b.x()-rhs._a.x(), rhs._b.y()-rhs._a.y()); 
    v1.normalize();
    return acos( v0 * v1 );
}

//--------------------------------------------------------------------------

bool
Segment2d::intersect( const Line2d& rhs, osg::Vec3d& out ) const
{
    double u, v;
    bool ok = intersectRaysXY(_a, (_b-_a), rhs._a, (rhs._b-rhs._a), out, u, v);
    return ok && u >= 0.0 && u <= 1.0;
}

bool
Segment2d::intersect( const Segment2d& rhs, osg::Vec3d& out ) const
{
    double u, v;
    bool ok = intersectRaysXY(_a, (_b-_a), rhs._a, (rhs._b-rhs._a), out, u, v);
    return ok && u >= 0.0 && u <= 1.0 && v >= 0.0 && v <= 1.0;
}

bool
Segment2d::intersect( const Ray2d& rhs, osg::Vec3d& out ) const
{
    double u, v;
    bool ok = intersectRaysXY(_a, (_b-_a), rhs._a, rhs._dv, out, u, v);
    return ok && u >= 0.0 && u <= 1.0 && v >= 0.0;
}

bool
Segment2d::intersect( const Line2d& rhs, osg::Vec2d& out ) const
{
    osg::Vec3d out3;
    bool ok = intersect( rhs, out3 );
    out.set(out3.x(), out3.y());
    return ok;
}

bool
Segment2d::intersect( const Segment2d& rhs, osg::Vec2d& out ) const
{
    osg::Vec3d out3;
    bool ok = intersect( rhs, out3 );
    out.set(out3.x(), out3.y());
    return ok;
}

bool
Segment2d::intersect( const Ray2d& rhs, osg::Vec2d& out ) const
{
    osg::Vec3d out3;
    bool ok = intersect( rhs, out3 );
    out.set(out3.x(), out3.y());
    return ok;
}

bool
Segment2d::isPointOnLeft( const osg::Vec3d& p ) const
{
    return ((_b-_a) ^ (p-_a)).z() >= 0.0;
}

bool
Segment2d::isPointOnLeft( const osg::Vec2d& p ) const
{
    return ((_b-_a) ^ (osg::Vec3d(p.x(),p.y(),0)-_a)).z() >= 0.0;
}

double
Segment2d::angle(const Segment2d& rhs) const
{
    osg::Vec2d v0(_b.x()-_a.x(), _b.y()-_a.y());
    v0.normalize();
    osg::Vec2d v1(rhs._b.x()-rhs._a.x(), rhs._b.y()-rhs._a.y());
    v1.normalize();
    return acos( v0 * v1 );
}

//--------------------------------------------------------------------------

Segment3d
Segment2d::unrotateTo3D(const osg::Quat& q) const
{
    osg::Quat qi = q.inverse();
    return Segment3d( qi*_a, qi*_b );
}

//--------------------------------------------------------------------------

bool
Triangle2d::contains(const osg::Vec3d& p) const
{
    if ( !Line2d(_a, _b).isPointOnLeft(p) ) return false;
    if ( !Line2d(_b, _c).isPointOnLeft(p) ) return false;
    if ( !Line2d(_c, _a).isPointOnLeft(p) ) return false;
    return true;
}
