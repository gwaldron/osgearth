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
#include <osgUtil/CullVisitor>
#include <osg/Transform>
#include <osgEarth/Registry>

#define LC "[Horizon] "


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
_scale   ( rhs._scale ),
_scaleInv( rhs._scaleInv ),
_eye     ( rhs._eye ),
_eyeUnit ( rhs._eyeUnit ),
_VC      ( rhs._VC ),
_VCmag   ( rhs._VCmag ),
_VCmag2  ( rhs._VCmag2 ),
_VHmag2  ( rhs._VHmag2 ),
_coneCos ( rhs._coneCos ),
_coneTan ( rhs._coneTan )
{
    // nop
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

    // just so we don't have gargabe values
    setEye( osg::Vec3d(1e7, 0, 0) );
}

void
Horizon::setEye(const osg::Vec3d& eye)
{
    if ( eye != _eye )
    {
        _eye = eye;
        _eyeUnit = eye;
        _eyeUnit.normalize();

        _VC     = osg::componentMultiply( -_eye, _scale );  // viewer->center (scaled)
        _VCmag  = _VC.length();
        _VCmag2 = _VCmag*_VCmag;
        _VHmag2 = _VCmag2 - 1.0;  // viewer->horizon line (scaled)

        double VCmag = sqrt(_VCmag2);
        double VPmag = VCmag - 1.0/VCmag; // viewer->horizon plane dist (scaled)
        double VHmag = sqrtf( _VHmag2 );

        _coneCos = VPmag / VHmag; // cos of half-angle of horizon cone
        _coneTan = tan(acos(_coneCos));
    }
}

bool
Horizon::isVisible(const osg::Vec3d& target,
                   double            radius) const
{
    if ( radius >= _scaleInv.x() || radius >= _scaleInv.y() || radius >= _scaleInv.z() )
        return true;
    
    // First check the object against the horizon plane, a plane that intersects the 
    // ellipsoid, whose normal is the vector from the eyepoint to the center of the 
    // ellipsoid.
    // ref: https://cesiumjs.org/2013/04/25/Horizon-culling/

    // Viewer-to-target vector
    osg::Vec3d VT;

    // move the target closer to the horizon plane by "radius".
    VT = (target + _eyeUnit*radius) - _eye;

    // transform into unit space:
    VT = osg::componentMultiply( VT, _scale );

    // If the point is in front of the horizon plane, it's visible and we're done
    if ( VT*_VC <= _VHmag2 )
    {
        return true;
    }

    // The sphere is completely behind the horizon plane. So now intersect the 
    // sphere with the horizon cone, a cone eminating from the eyepoint along the 
    // eye->center vetor. If the sphere is entirely within the cone, it is occluded
    // by the spheroid (not ellipsoid, sorry)
    // ref: http://www.cbloom.com/3d/techdocs/culling.txt
    VT = target - _eye;

    double a = VT * -_eyeUnit;
    double b = a * _coneTan;
    double c = sqrt( VT*VT - a*a );
    double d = c - b;
    double e = d * _coneCos;

    if ( e > -radius )
    {
        // sphere is at least partially outside the cone (visible)
        return true;
    }

    // occluded.
    return false;
}


bool
Horizon::getPlane(osg::Plane& out_plane) const
{
    // calculate scaled distance from center to viewer:
    if ( _VCmag2 == 0.0 )
        return false;

    // calculate scaled distance from center to horizon plane:
    double PCmag = 1.0/_VCmag;

    osg::Vec3d pcWorld = osg::componentMultiply(_eyeUnit*PCmag, _scaleInv);
    double dist = pcWorld.length();

    // compute a new clip plane:
    out_plane.set(_eyeUnit, -dist);
    return true;
}

//........................................................................

HorizonCullCallback::HorizonCullCallback() :
_enabled   ( true ),
_centerOnly( false )
{
    //nop
}

void
HorizonCullCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    bool visible = true;

    if ( _enabled && node && nv && nv->getVisitorType() == nv->CULL_VISITOR )
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
        
        osg::NodePath np = nv->getNodePath();
        osg::Matrix local2world = osg::computeLocalToWorld(np);

        // make a local copy to support multi-threaded cull
        osg::Vec3d eye = osg::Vec3d(nv->getViewPoint()) * local2world;
        Horizon horizon(_horizon);
        horizon.setEye( eye );

        // pop the last node in the path (which is the node this callback is on)
        // to prevent double-transforming the bounding sphere's center point
        np.pop_back();
        local2world = osg::computeLocalToWorld(np);
        const osg::BoundingSphere& bs = node->getBound();

        double radius = _centerOnly ? 0.0 : bs.radius();

        visible = horizon.isVisible( bs.center()*local2world, radius );
    }

    if ( visible )
    {
        traverse(node, nv);
    }
}
