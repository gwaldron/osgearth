/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2015 Pelican Mapping
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
#include <osgEarth/Registry>

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
_scale        ( rhs._scale ),
_scaleInv     ( rhs._scaleInv ),
_cv           ( rhs._cv ),
_vhMag2       ( rhs._vhMag2 ),
_scaleToMinHAE( rhs._scaleToMinHAE )
{
    //nop
}

void
Horizon::setEllipsoid(const osg::EllipsoidModel& e)
{
    _scaleInv.set( 
        e.getRadiusEquator(),
        e.getRadiusEquator(),
        e.getRadiusPolar() );

    _scale.set(
        1.0 / e.getRadiusEquator(),
        1.0 / e.getRadiusEquator(),
        1.0 / e.getRadiusPolar() );

    // Minimum allowable HAE for calculating horizon distance.
    const double minHAE = 100.0;

    //double maxRadius = std::max(e.getRadiusEquator(), e.getRadiusPolar());
    //double minHAEScaled = 1.0 + minHAE/maxRadius;
    //_minHAEScaled2 = minHAEScaled * minHAEScaled;

    _scaleToMinHAE = (_scale*minHAE) + osg::Vec3d(1,1,1);
}

void
Horizon::setEye(const osg::Vec3d& eyeECEF)
{
    _cv = osg::componentMultiply(eyeECEF, _scale);

    double cvMag2 = _cv*_cv;

    osg::Vec3d minCV = _cv;
    minCV.normalize();
    minCV = osg::componentMultiply(minCV, _scaleToMinHAE);
    double min_cvMag2 = minCV*minCV;

#if 0 // debugging
    osg::Vec3d msl = _cv;
    msl.normalize();
    msl = osg::componentMultiply(msl, _scale+osg::Vec3d(1,1,1));
    msl = osg::componentMultiply(msl, _scaleInv);
    double alt = eyeECEF.length() - msl.length();
#endif

    if ( cvMag2 >= min_cvMag2 )
    {
        _vhMag2 = cvMag2-1.0;
    }
    else
    {
        _cv = minCV;
        _vhMag2 = (_cv*_cv)-1.0;
    }
    
#if 0 // debugging
    osg::Vec3d vh = _cv;
    vh.normalize();
    vh = osg::componentMultiply(vh, (_scale*sqrt(_vhMag2))+osg::Vec3d(1,1,1));
    vh = _scaleInv * sqrt(_vhMag2);

    static int count=0;
    if (count++ %60 == 0) {
        OE_NOTICE << "cvmag2="<< cvMag2 << "; minMag2="<< min_cvMag2 << "; vhMag2=" << _vhMag2 << "; alt=" << alt << "; vh=" << vh.length() << "\n";
    }
#endif
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
    // calculate scaled distance from center to viewer:
    double magVC = _cv.length();
    if ( magVC == 0.0 )
        return false;

    // calculate scaled distance from center to horizon plane:
    double magPC = 1.0/magVC;

    osg::Vec3d normal = _cv;
    normal.normalize();

    osg::Vec3d pcWorld = osg::componentMultiply(normal*magPC, _scaleInv);
    double dist = pcWorld.length();

    // compute a new clip plane:
    out_plane.set(normal, -dist);
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
        horizon.setEye( osg::Vec3d(nv->getViewPoint()) * local2world );

        const osg::BoundingSphere& bs = node->getBound();

        visible = !horizon.occludes( bs.center() * local2world, bs.radius() );
    }

    if ( visible )
    {
        traverse(node, nv);
    }
}

