/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarthUtil/ViewFitter>

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
    // Projected the point Pview (in camera view space) onto the far clip plane of
    // a projection matrix.
    void projectToFarPlane(osg::Vec3d& Pview, const osg::Matrix& projMatrix, const osg::Matrix& projMatrixInv)
    {
        osg::Vec4d Pclip = osg::Vec4d(Pview.x(), Pview.y(), Pview.z(), 1.0)* projMatrix;
        Pclip.z() = Pclip.w();
        osg::Vec4d Ptemp = Pclip * projMatrixInv;
        Pview.set(Ptemp.x() / Ptemp.w(), Ptemp.y() / Ptemp.w(), Ptemp.z() / Ptemp.w());
    }
}

ViewFitter::ViewFitter(const SpatialReference* mapSRS, const osg::Camera* camera) :
_mapSRS(mapSRS),
_camera(camera)
{
    //nop
}

bool
ViewFitter::createViewpoint(const std::vector<GeoPoint>& points, Viewpoint& outVP) const
{
    if (points.empty() || _mapSRS.valid() == false || _camera.valid()== false)
        return false;

    osg::Matrix projMatrix = _camera->getProjectionMatrix();
    osg::Matrix viewMatrix = _camera->getViewMatrix();

    // Orthographic matrix is not yet supported.
    bool isOrtho = osg::equivalent(projMatrix(3,3), 1.0);
    if (isOrtho)
        return false;

    // Convert the point set to world space:
    std::vector<osg::Vec3d> world(points.size());

    for (int i = 0; i < points.size(); ++i)
    {
        GeoPoint p = points[i].transform(_mapSRS.get());
        p.toWorld(world[i]);
    }

    // Rewrite the projection matrix so the far plane is at the ellipsoid. 
    // We do this so we can project our control points onto a common plane.
    double fovy_deg, ar, znear, zfar;
    projMatrix.getPerspective(fovy_deg, ar, znear, zfar);
    znear = 1.0;
    double eyeDist;

    if (_mapSRS->isGeographic())
    {
        zfar = osg::maximum(_mapSRS->getEllipsoid()->getRadiusEquator(),
                            _mapSRS->getEllipsoid()->getRadiusPolar());
        eyeDist = zfar * 2.0;
    }
    else
    {
        osg::Vec3d eye, center, up2;
        viewMatrix.getLookAt(eye, center, up2);
        eyeDist = eye.length();
        zfar = eyeDist;
    }
    projMatrix.makePerspective(fovy_deg, ar, znear, zfar);

    // Calculate the "centroid" of our point set:
    osg::Vec3d lookFrom;
    for (int i = 0; i < world.size(); ++i)
        lookFrom += world[i];
    lookFrom /= world.size();

    // Set up a new view matrix to look down on that centroid:
    osg::Vec3d lookAt, up;

    if (_mapSRS->isGeographic())
    {
        lookFrom.normalize();
        lookFrom *= eyeDist;
        lookAt.set(0,0,0);
        up.set(0,0,1);
    }
    else
    {
        lookFrom.z() = eyeDist;
        lookAt.set(lookFrom.x(), lookFrom.y(), 0);
        up.set(0,1,0);
    }
    viewMatrix.makeLookAt(lookFrom, lookAt, up);

    // Transform our control points into view space, and then project each one
    // onto our common view plane (tangent to the ellispoid).
    osg::Matrix projMatrixInv;
    projMatrixInv.invert(projMatrix);

    double Mx = -DBL_MAX, My = -DBL_MAX;
    std::vector<osg::Vec3d> view(world.size());
    for (int i = 0; i < world.size(); ++i)
    {
        view[i] = world[i] * viewMatrix;
        projectToFarPlane(view[i], projMatrix, projMatrixInv);
        Mx = osg::maximum(Mx, osg::absolute(view[i].x()));
        My = osg::maximum(My, osg::absolute(view[i].y()));
    }

    // Calculate optimal new Z (distance from view plane)
    double half_fovy_rad = osg::DegreesToRadians(fovy_deg) * 0.5;
    double half_fovx_rad = half_fovy_rad * ar;
    double Zx = Mx / tan(half_fovx_rad);
    double Zy = My / tan(half_fovy_rad);
    double Zbest = std::max(Zx, Zy);

    // Calcluate the new viewpoint.
    osg::Vec3d FPworld = lookFrom;

    if (_mapSRS->isGeographic())
    {
        FPworld.normalize();
        FPworld *= zfar;
    }
    else
    {
        FPworld.z() = 0.0;
    }

    // Convert to a geopoint
    GeoPoint FP;
    FP.fromWorld(_mapSRS.get(), FPworld);
    outVP = Viewpoint();
    outVP.focalPoint() = FP;
    outVP.pitch() = -90;
    outVP.range() = Zbest;

    return true;
}