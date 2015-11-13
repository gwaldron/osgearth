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
#include <osg/ValueObject>

#define LC "[Horizon] "

#define OSGEARTH_HORIZON_UDC_NAME "osgEarth.Horizon"

using namespace osgEarth;

Horizon::Horizon()
{
    setName(OSGEARTH_HORIZON_UDC_NAME);
    setEllipsoid(osg::EllipsoidModel());
}

Horizon::Horizon(const osg::EllipsoidModel& e)
{
    setEllipsoid( e );
}

Horizon::Horizon(const Horizon& rhs, const osg::CopyOp& op) :
osg::Object( rhs, op ),
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

#ifdef OSGEARTH_HORIZON_SUPPORTS_NODEVISITOR

void
Horizon::put(osg::NodeVisitor& nv)
{
    osg::UserDataContainer* udc = nv.getOrCreateUserDataContainer();
    unsigned i = udc->getUserObjectIndex(OSGEARTH_HORIZON_UDC_NAME);
    if (i < udc->getNumUserObjects()) udc->setUserObject( i, this );
    else udc->addUserObject(this);
}

Horizon* Horizon::get(osg::NodeVisitor& nv)
{
    osg::UserDataContainer* udc = nv.getUserDataContainer();
    if ( !udc ) return 0L;
    unsigned i = udc->getUserObjectIndex(OSGEARTH_HORIZON_UDC_NAME);
    if ( i < udc->getNumUserObjects() ) return static_cast<Horizon*>(udc->getUserObject(i));
    else return 0L;
}
#endif

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

        //double VCmag = sqrt(_VCmag2);
        double VPmag = _VCmag - 1.0/_VCmag; // viewer->horizon plane dist (scaled)
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

    // If the target is above the eye, it's visible
    double VTdotVC = VT*_VC;
    if ( VTdotVC <= 0.0 )
    {        
        return true;
    }
    
    // If the eye is below the ellipsoid, but the target is below the eye
    // (since the above test failed) the target is occluded. 
    // NOTE: it might be better instead to check for a maximum distance from
    // the eyepoint instead. 
    if ( _VCmag < 0.0 )
    {
        return false;
    }

    // Now we know that the eye is above the ellipsoid, so there is a valid horizon plane. 
    // If the point is in front of that horizon plane, it's visible and we're done
    if ( VTdotVC <= _VHmag2 )
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

    double PCmag;
    if ( _VCmag > 0.0 )
    {
        // eyepoint is above ellipsoid? Calculate scaled distance from center to horizon plane
        PCmag = 1.0/_VCmag;    
    }
    else
    {
        // eyepoint is below the ellipsoid? plane passes through the eyepoint.
        PCmag = _VCmag;
    }

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


bool
HorizonCullCallback::isVisible(osg::Node* node, osg::NodeVisitor* nv)
{
    osg::NodePath np = nv->getNodePath();
    osg::Matrix local2world;
    Horizon* horizon = 0L;

#ifdef OSGEARTH_HORIZON_SUPPORTS_NODEVISITOR
    horizon = Horizon::get(*nv);
#endif

    if ( horizon )
    {
        // pop the last node in the path (which is the node this callback is on)
        // to prevent double-transforming the bounding sphere's center point
        np.pop_back();
        local2world = osg::computeLocalToWorld(np);

        const osg::BoundingSphere& bs = node->getBound();
        double radius = _centerOnly ? 0.0 : bs.radius();
        return horizon->isVisible( bs.center()*local2world, radius );
    }
    else
    {
        // make a local copy to support multi-threaded cull
        local2world  = osg::computeLocalToWorld(np);
        osg::Vec3d eye = osg::Vec3d(nv->getViewPoint()) * local2world;

        // pop the last node in the path (which is the node this callback is on)
        // to prevent double-transforming the bounding sphere's center point
        np.pop_back();
        local2world = osg::computeLocalToWorld(np);

        osg::ref_ptr<Horizon> horizonCopy = osg::clone(_horizon.get());
        horizonCopy->setEye( eye );

        const osg::BoundingSphere& bs = node->getBound();
        double radius = _centerOnly ? 0.0 : bs.radius();
        return horizonCopy->isVisible( bs.center()*local2world, radius );
    }
}

void
HorizonCullCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{    
    if ( _enabled )
    {
        if ( _proxy.valid() )
        {
            osg::ref_ptr<osg::Node> proxy;
            if ( _proxy.lock(proxy) )
            {
                if ( isVisible(proxy.get(), nv) )
                {
                    traverse(node, nv);
                    return;
                }
            }
        }

        if ( isVisible(node, nv) )
        {
            traverse(node, nv);
        }
    }

    else
    {
        traverse(node, nv);
    }
}
