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

#ifndef OSGEARTH_ELLIPSOID_INTERSECTOR_H
#define OSGEARTH_ELLIPSOID_INTERSECTOR_H 1

#include <osg/Matrix>
#include <osg/CoordinateSystemNode>
#include <osgEarth/Common>

namespace osgEarth
{
    /**
    * Utility class to perform ellipsoid intersections
    */
    class OSGEARTH_EXPORT EllipsoidIntersector
    {
    public:
        //! Construct a new ellipsoid intersector
        EllipsoidIntersector(const osg::EllipsoidModel* em);

        //! Interects a line (world space) with an ellipsoid.
        //! @param p0 First point on the line
        //! @param p1 Second point on the line
        //! @param out_world Output world coordinates of closest intersection
        bool intersectLine(const osg::Vec3d& p0_world, const osg::Vec3d& p1_world, osg::Vec3d& out_world);

    private:
        osg::Matrix _ellipsoidToUnitSphere;
        osg::Matrix _unitSphereToEllipsoid;
        osg::Matrix _clipToWorld;
    };

} // namespace osgEarth

#endif // OSGEARTH_ELLIPSOID_INTERSECTOR_H
