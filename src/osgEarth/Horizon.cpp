/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/Horizon>
#include <osgEarth/CullingUtils>
#include <osgEarth/Utils>
#include <osgEarth/Math>

#include <osgUtil/CullVisitor>

#define LC "[Horizon] "

namespace
{
    const std::string OSGEARTH_HORIZON_UDC_NAME = "osgEarth.Horizon";
}

using namespace osgEarth;

Horizon::Horizon()
{
    setName(OSGEARTH_HORIZON_UDC_NAME);
    setEllipsoid(Ellipsoid());
}

Horizon::Horizon(const Ellipsoid& e)
{
    setName(OSGEARTH_HORIZON_UDC_NAME);
    setEllipsoid( e );
}

Horizon::Horizon(const SpatialReference* srs)
{
    setName(OSGEARTH_HORIZON_UDC_NAME);
    if ( srs && !srs->isProjected() )
    {
        setEllipsoid( srs->getEllipsoid() );
    }
}

Horizon::Horizon(const Horizon& rhs, const osg::CopyOp& op) :
    osg::Object(rhs, op),
    _em(rhs._em),
    _valid(rhs._valid),
    _scale(rhs._scale),
    _scaleInv(rhs._scaleInv),
    _eye(rhs._eye),
    _eyeUnit(rhs._eyeUnit),
    _VC(rhs._VC),
    _VCmag(rhs._VCmag),
    _VCmag2(rhs._VCmag2),
    _VHmag2(rhs._VHmag2),
    _coneCos(rhs._coneCos),
    _coneTan(rhs._coneTan),
    _minVCmag(rhs._minVCmag),
    _minHAE(rhs._minHAE)
{
    // nop
}

void
Horizon::setEllipsoid(const Ellipsoid& em)
{
    _em = em;

    _scaleInv.set(
        em.getRadiusEquator(),
        em.getRadiusEquator(),
        em.getRadiusPolar() );

    _scale.set(
        1.0 / em.getRadiusEquator(),
        1.0 / em.getRadiusEquator(),
        1.0 / em.getRadiusPolar() );

    _minHAE = 500.0;
    _minVCmag = 1.0 + (_scale*_minHAE).length();

    // just so we don't have garbage values
    setEye(osg::Vec3d(1e7, 0, 0), nullptr);

    _valid = true;
}

void
Horizon::setMinHAE(double value)
{
    _minHAE = value;
    _minVCmag = 1.0 + (_scale*_minHAE).length();
}

bool
Horizon::setEye(const osg::Vec3d& eye, const osg::RefMatrix* proj)
{
    if (eye == _eye)
        return false;

    _orthographic = proj ? ProjectionMatrix::isOrtho(*proj) : false;

    _eye = eye;
    _eyeUnit = eye;
    _eyeUnit.normalize();

    if (!_orthographic)
    {
        // vector from viewer to center of the earth
        _VC = osg::componentMultiply(-_eye, _scale);

        // distance from viewer to center of the earth, clamped to the min HAE (?)
        _VCmag = std::max(_VC.length(), _minVCmag);

        // distance squared from viewer to center of the earth
        _VCmag2 = _VCmag * _VCmag;

        // distance squared from the viewer to the nearest point on the horizon
        _VHmag2 = _VCmag2 - 1.0;

        // distance from the viewer to the horizon plane
        double VPmag = _VCmag - 1.0 / _VCmag;

        // distance from the viewer to the nearest surface point on the horizon
        double VHmag = sqrtf(_VHmag2);

        _coneCos = VPmag / VHmag; // cos of half-angle of horizon cone
        _coneTan = tan(acos(_coneCos));
    }

    return true;
}

double
Horizon::getRadius() const
{
    return osg::componentMultiply(_eyeUnit, _scaleInv).length();
}

bool
Horizon::isVisible(const osg::Vec3d& target, double radius) const
{
    if ( _valid == false || radius >= _scaleInv.x() || radius >= _scaleInv.y() || radius >= _scaleInv.z() )
        return true;   
    
    if (_orthographic)
    {
        auto CT = osg::componentMultiply(target, _scale);
        auto CTmag = CT.length();
        auto CTmag2 = CTmag * CTmag;
        CT.normalize();

        double cos_a = -_eyeUnit * CT;
        if (cos_a <= 0.0)
            return true;

        double x = CTmag * cos_a;
        double d = CTmag2 - x * x;

        return d >= 1.0;
    }

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
    if ( _valid == false || _VCmag2 == 0.0 )
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

double
Horizon::getDistanceToVisibleHorizon() const
{
    double eyeLen = _eye.length();

    osg::Vec3d geodetic;
    //double lat, lon, hasl;
    //_em.convertXYZToLatLongHeight(_eye.x(), _eye.y(), _eye.z(), lat, lon, hasl);

    geodetic = _em.geocentricToGeodetic(_eye);
    double hasl = geodetic.z();

    // limit it:
    hasl = osg::maximum(hasl, 100.0);

    double radius = eyeLen - hasl;
    return sqrt(2.0*radius*hasl + hasl*hasl);
}

//........................................................................


HorizonCullCallback::HorizonCullCallback() :
    _enabled(true),
    _centerOnly(false),
    _customEllipsoidSet(false)
{
    //nop
}

void
HorizonCullCallback::setEllipsoid(const Ellipsoid& em)
{
    _customEllipsoid.setSemiMajorAxis(em.getRadiusEquator());
    _customEllipsoid.setSemiMinorAxis(em.getRadiusPolar());
    _customEllipsoidSet = true;
}

bool
HorizonCullCallback::isVisible(osg::Node* node, osg::NodeVisitor* nv)
{
    if ( !node )
        return false;

    osg::ref_ptr<Horizon> horizon;
    ObjectStorage::get(nv, horizon);

    auto* cv = Culling::asCullVisitor(nv);

    if (_customEllipsoidSet)
    {
        osg::Vec3d eye;
        if (horizon.valid())
            eye = horizon->getEye();
        else
            eye = osg::Vec3d(0,0,0) * cv->getCurrentCamera()->getInverseViewMatrix();

        horizon = new Horizon(_customEllipsoid);
        horizon->setEye(eye, cv->getProjectionMatrix());
    }

    // If we fetched the Horizon from the nodevisitor...
    if ( horizon.valid() )
    {
        // pop the last node in the path (which is the node this callback is on)
        // to prevent double-transforming the bounding sphere's center point
        osg::NodePath np = nv->getNodePath();
        if (!np.empty() && np.back() == node)
            np.pop_back();

        osg::Matrix local2world = osg::computeLocalToWorld(np);

        const osg::BoundingSphere& bs = node->getBound();
        double radius = _centerOnly ? 0.0 : bs.radius();
        return horizon->isVisible( bs.center()*local2world, radius );
    }

    // If the user forgot to install a horizon at all...
    else
    {
        // No horizon data... just assume visibility
        //OE_WARN << LC << "No horizon info installed in callback\n";
        return true;
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
                    traverse(node, nv);
                return;
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
