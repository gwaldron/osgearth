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
#ifndef OSGEARTH_UTIL_VIEW_FITTER_H
#define OSGEARTH_UTIL_VIEW_FITTER_H

#include <osgEarthUtil/Common>
#include <osgEarth/GeoData>
#include <osgEarth/Viewpoint>
#include <osg/Camera>

namespace osgEarth { namespace Util
{
    using namespace osgEarth;

    /**
     * Creates Viewpoints that fit a camera's view frustum to encompass
     * a set of geospatial points as tightly as possible.
     */
    class OSGEARTHUTIL_EXPORT ViewFitter
    {
    public:
        //! Construct a ViewFitter with a Map SRS and a camera.
        ViewFitter(const SpatialReference* mapSRS, const osg::Camera* camera);

        //! Creates a Viewpoint that looks straight down on the map and
        //! encompasses the provided set of points.
        //! Returns true upon success, or false if there is missing data or if
        //! the camera is orthographic.
        bool createViewpoint(const std::vector<GeoPoint>& points, Viewpoint& out) const;

        //! Sets a buffer (in meters) to apply to the view fit. Applying a buffer will
        //! expand the view so that the points are at least "buffer" meters inside the
        //! edge of the fitted view.
        void setBuffer(double value_meters) { _buffer_m = value_meters; }
        double getBuffer() const { return _buffer_m; }

        //! Sets the reference VFOV when using an orthographic camera.
        void setReferenceVFOV(float vfov_degrees) {  _vfov = vfov_degrees; }

    protected:
        osg::ref_ptr<const osg::Camera> _camera;
        osg::ref_ptr<const SpatialReference> _mapSRS;
        float _vfov;
        double _buffer_m;
    };

} } // namespace osgEarth::Util

#endif // OSGEARTH_UTIL_VIEW_FITTER_H
