/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarth/EllipsoidIntersector>

using namespace osgEarth;

EllipsoidIntersector::EllipsoidIntersector(const osg::EllipsoidModel* em)
{
    _ellipsoidToUnitSphere.makeScale(
        1.0 / em->getRadiusEquator(),
        1.0 / em->getRadiusEquator(),
        1.0 / em->getRadiusPolar());

    _unitSphereToEllipsoid.makeScale(
        em->getRadiusEquator(),
        em->getRadiusEquator(),
        em->getRadiusPolar());
}

bool EllipsoidIntersector::intersectLine(const osg::Vec3d& p0_world, const osg::Vec3d& p1_world, osg::Vec3d& out_world)
{
    double dist2 = 0.0;
    osg::Vec3d v;
    osg::Vec3d p0 = p0_world * _ellipsoidToUnitSphere;
    osg::Vec3d p1 = p1_world * _ellipsoidToUnitSphere;

    const double R = 1.0; // for unit sphere.

                          // http://paulbourke.net/geometry/circlesphere/index.html#linesphere

    osg::Vec3d d = p1 - p0;

    double A = d * d;
    double B = 2.0 * (d * p0);
    double C = (p0 * p0) - R*R;

    // now solve the quadratic A + B*t + C*t^2 = 0.
    double D = B*B - 4.0*A*C;
    if (D > 0)
    {
        // two roots (line passes through sphere twice)
        // find the closer of the two.
        double sqrtD = sqrt(D);
        double t0 = (-B + sqrtD) / (2.0*A);
        double t1 = (-B - sqrtD) / (2.0*A);

        //seg; pick closest:
        if (fabs(t0) < fabs(t1))
            v = d*t0;
        else
            v = d*t1;
    }
    else if (D == 0.0)
    {
        // one root (line is tangent to sphere?)
        double t = -B / (2.0*A);
        v = d*t;
    }

    dist2 = v.length2();

    if (dist2 > 0.0)
    {
        out_world = (p0 + v) * _unitSphereToEllipsoid;
        return true;
    }
    else
    {
        // either no intersection, or the distance was not the max.
        return false;
    }
}
