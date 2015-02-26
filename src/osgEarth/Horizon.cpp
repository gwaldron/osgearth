/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include <osgEarth/Horizon>
#include <osg/Transform>

using namespace osgEarth;

Horizon::Horizon()
{
    setEllipsoid(osg::EllipsoidModel());
}

Horizon::Horizon(const osg::EllipsoidModel& e)
{
    setEllipsoid( e );
}

Horizon::Horizon(const Horizon& rhs) :
_scale ( rhs._scale ),
_eye   ( rhs._eye ),
_eyeLen( rhs._eyeLen ),
_cv    ( rhs._cv ),
_vhMag2( rhs._vhMag2 )
{
    //nop
}

void
Horizon::setEllipsoid(const osg::EllipsoidModel& e)
{
    _scale.set(
        1.0 / e.getRadiusEquator(),
        1.0 / e.getRadiusEquator(),
        1.0 / e.getRadiusPolar() );
}

void
Horizon::setEye(const osg::Vec3d& eyeECEF)
{
    _eye    = eyeECEF;
    _eyeLen = eyeECEF.length();
    _cv     = osg::componentMultiply(eyeECEF, _scale);
    _vhMag2 = (_cv*_cv)-1.0;
}

bool
Horizon::occludes(const osg::Vec3d& targetECEF,
                  double            radius) const
{
    // ref: https://cesiumjs.org/2013/04/25/Horizon-culling/

    osg::Vec3d tc = targetECEF;

    if ( radius > 0.0 )
    {
        // shift the target point outward to account for its bounding radius.
        double targetLen2 = tc.length2();
        osg::Vec3d targetUnit = tc;
        targetUnit.normalize();
        tc += targetUnit * radius;
    }
    
    tc = osg::componentMultiply(tc, _scale);

    osg::Vec3d vt = tc - _cv;
    double vtMag2 = vt.length2();
    double vtDotVc = -(vt*_cv);

    bool behindHorizonPlane = (vtDotVc > _vhMag2);
    bool insideHorizonCone  = (vtDotVc*vtDotVc / vtMag2) > _vhMag2;

    return behindHorizonPlane && insideHorizonCone;
}

bool
Horizon::getPlane(osg::Plane& out_plane) const
{
    if ( _eyeLen == 0.0 )
        return false;

    // calculate scaled distance from center to viewer:
    double magVC = _cv.length();
    if ( magVC == 0.0 )
        return false;

    // calculate scaled distance from center to horizon plane:
    double magPC = 1.0/magVC;

    // convert back to real space:
    double dist   = _eyeLen * magPC/magVC;

    // normalize the eye vector:
    osg::Vec3d normal = _eye;
    normal /= _eyeLen; // normalize

    // compute a new clip plane:
    out_plane.set(normal, normal*dist);
    return true;
}

//........................................................................

HorizonCullCallback::HorizonCullCallback() :
_enabled( true )
{
    //nop
}

HorizonCullCallback::HorizonCullCallback(const Horizon& horizon) :
_horizon( horizon ),
_enabled( true )
{
    //nop
}

void
HorizonCullCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    bool visible = true;

    if ( _enabled && node && nv && nv->getVisitorType() == nv->CULL_VISITOR )
    {
        osg::Matrix local2world = osg::computeLocalToWorld(nv->getNodePath());

        // make a local copy to support multi-threaded cull
        Horizon horizon(_horizon);
        horizon.setEye( osg::Vec3d(nv->getEyePoint()) * local2world );

        const osg::BoundingSphere& bs = node->getBound();

        visible = !horizon.occludes( bs.center() * local2world, bs.radius() );
    }

    if ( visible )
    {
        traverse(node, nv);
    }
}

