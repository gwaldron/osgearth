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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include <osgEarth/CascadeDrapingDecorator>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/Horizon>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Shaders>
#include <osgEarth/TerrainResources>
#include <osgEarth/ShaderUtils>
#include <osgEarth/LineDrawable>
#include <osgEarth/GLUtils>

#include <osg/Texture2D>
#include <osg/Texture2DArray>
#include <osg/BlendFunc>
#include <osg/ShapeDrawable>
#include <osg/AutoTransform>
#include <osg/Depth>
#include <osgUtil/CullVisitor>
#include <osgUtil/LineSegmentIntersector>
#include <osgShadow/ConvexPolyhedron>

#include <stdlib.h> // getenv

#define LC "[CascadeDrapingDecorator] "

using namespace osgEarth;



// Whether to intersect the ellipsoid versus the horizon plane when
// building cascade bounding boxes. This will result in closer cascades
// but less resolution at distance
//#define USE_ELLIPSOID_INTERSECTIONS

// Enable this to generate an "--activity" readout of all cascade information
//#define DEBUG_CASCADES

// Enable this to do a terrain intersection before configuring the first cascade
#define USE_TERRAIN_INTERSECTION

// TODO ITEMS:
// - Address the dangling CameraLocal when a view disappears


namespace
{
    struct Line2d
    {
        bool intersectRaysXY(
            const osg::Vec3d& p0, const osg::Vec3d& d0,
            const osg::Vec3d& p1, const osg::Vec3d& d1,
            osg::Vec3d& out_p,
            double&     out_u,
            double&     out_v) const
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

        osg::Vec3d _a, _b;

        Line2d(const osg::Vec3d& p0, const osg::Vec3d& p1) : _a(p0), _b(p1) { }

        Line2d(const osg::Vec4d& p0, const osg::Vec4d& p1)
            : _a(p0.x()/p0.w(), p0.y()/p0.w(), p0.x()/p0.w()), _b(p1.x()/p1.w(), p1.y()/p1.w(), p1.z()/p1.w()) { }

        bool intersect(const Line2d& rhs, osg::Vec4d& out) const {
            double u, v;
            osg::Vec3d temp;
            bool ok = intersectRaysXY(_a, (_b-_a), rhs._a, (rhs._b-rhs._a), temp, u, v);
            out.set( temp.x(), temp.y(), temp.z(), 1.0 );
            return ok;
        }
        bool intersect(const Line2d& rhs, osg::Vec3d& out) const {
            double u, v;
            return intersectRaysXY(_a, (_b-_a), rhs._a, (rhs._b-rhs._a), out, u, v);
        }
    };

    void fitProjectionMatrix(osg::Matrix& proj, double nfratio)
    {
        // construct the polygon, which winds counter-clockwise from upper-right to lower-right.a
        osg::Vec4d t0(+1.0, 1.0, 0.0, 1.0);
        osg::Vec4d t1(-1.0, 1.0, 0.0, 1.0);
        osg::Vec4d t2(-nfratio, -1.0, 0.0, 1.0);
        osg::Vec4d t3(+nfratio, -1.0, 0.0, 1.0);

        // Next warp our polygon t0,t1,t2,t3 into a clip-space square
        // through a series of matrix operations.
        osg::Vec4d u, v;
        osg::Matrix M;

        // translate the center of the near plane to the origin
        u = (t2 + t3) * 0.5;
        osg::Matrix T1;
        T1.makeTranslate(-u.x(), -u.y(), 0.0);
        M = T1;

        // find the intersection of the side lines t0,t3 and t1,t2
        // and translate that point is at the origin:
        osg::Vec4d i;
        Line2d(t0, t3).intersect(Line2d(t1, t2), i);
        u = i*M;
        osg::Matrix T2;
        T2.makeTranslate(-u.x(), -u.y(), 0.0);
        M = T2*M;

        // scale the near corners to [-1,1] and [1,1] respectively:
        u = t3*M; // ...not t2.
        osg::Matrix S1;
        S1.makeScale(1 / u.x(), 1 / u.y(), 1.0);
        M = M*S1;

        // project onto the Y plane and translate the whole thing
        // back down to the origin at the same time.
        const osg::Matrix N(
            1, 0, 0, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, -1, 0, 0);
        M = M*N;

        // scale it back to unit size:
        u = t0*M;
        v = t3*M;
        osg::Matrix S3;
        S3.makeScale(1.0, 2.0 / (u.y() / u.w() - v.y() / v.w()), 1.0);
        M = M*S3;

        // finally, translate it to it lines up with the clip space boundaries.
        osg::Matrix T4;
        T4.makeTranslate(0.0, -1.0, 0.0);
        M = M*T4;

        // apply the result to the projection matrix.
        proj.postMult(M);
    }
}



CascadeDrapingDecorator::CascadeDrapingDecorator(const SpatialReference* srs, TerrainResources* resources) :
_unit(-1),
_multisamples(2u),
_maxCascades(4u),
_texSize(1024u),
_mipmapping(false),
_maxHorizonDistance(DBL_MAX),
_debug(false),
_srs(srs),
_resources(resources),
_constrainMaxYToFrustum(false),
_constrainRttBoxToDrapingSetBounds(true),
_useProjectionFitting(true),
_minNearFarRatio(0.25)
{
    if (::getenv("OSGEARTH_DRAPING_DEBUG"))
        _debug = true;

    const char* c = NULL;

    c = ::getenv("OSGEARTH_DRAPING_TEXTURE_SIZE");
    if (c)
        setTextureSize((unsigned)atoi(c));

    c = ::getenv("OSGEARTH_DRAPING_MAX_CASCADES");
    if (c)
        setMaxNumCascades((unsigned)atoi(c));

    c = ::getenv("OSGEARTH_DRAPING_MIPMAPPING");
    if (c)
        setUseMipMaps(atoi(c)? true : false);

    c = ::getenv("OSGEARTH_DRAPING_MULTISAMPLES");
    if (c)
        setNumMultiSamples((unsigned)atoi(c));

    c = ::getenv("OSGEARTH_DRAPING_MAX_HORIZON_DISTANCE");
    if (c)
        _maxHorizonDistance = (double)atoi(c);

    c = ::getenv("OSGEARTH_DRAPING_CONSTRAIN_TO_BOUNDS");
    if (c)
        _constrainRttBoxToDrapingSetBounds = atoi(c)?true:false;

    c = ::getenv("OSGEARTH_DRAPING_CONSTRAIN_TO_FRUSTUM");
    if (c)
        _constrainMaxYToFrustum = atoi(c)?true:false;

    c = ::getenv("OSGEARTH_DRAPING_USE_PROJECTION_FITTING");
    if (c)
        _useProjectionFitting = atoi(c)?true:false;
}

void
CascadeDrapingDecorator::setMaxNumCascades(unsigned value)
{
    _maxCascades = osg::clampBetween(value, 1u, 8u);
}

void
CascadeDrapingDecorator::setNumMultiSamples(unsigned value)
{
    _multisamples = osg::clampBetween(value, 0u, 4u);
}

void
CascadeDrapingDecorator::setTextureSize(unsigned value)
{
    _texSize = osg::clampBetween(value, 256u, 4096u);
}

void
CascadeDrapingDecorator::setUseMipMaps(bool value)
{
    _mipmapping = value;
}

void
CascadeDrapingDecorator::setUseProjectionFitting(bool value)
{
    _useProjectionFitting = value;
}

void
CascadeDrapingDecorator::setMinimumNearFarRatio(double value)
{
    _minNearFarRatio = value;
}

void
CascadeDrapingDecorator::traverse(osg::NodeVisitor& nv)
{
    bool traversedChildren = false;

    if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
        if (cv)
        {
            const osg::Camera* camera = cv->getCurrentCamera();

            // only proceed if there is geometry to drape.
            // TODO: is this correct? if there's nothing, should be clear out any
            // pre-existing projected texture or set a uniform or something?
            const osg::BoundingSphere& bound = _manager.get(camera).getBound();
            if (bound.valid())
            {
                // if we don't have a texture unit reserved, do so now.
                if (_unit < 0)
                {
                    reserveTextureImageUnit();
                }

                if (_unit >= 0)
                {
                    // access the draping configuration for this camera:
                    CameraLocal& local = _data.get(camera);

                    // traverse the RTT camera(s) and generate the projective texture
                    local.traverse(cv, *this, bound);

                    // then push the projected texture state and traverse the terrain.
                    cv->pushStateSet(local._terrainSS.get());
                    osg::Group::traverse(nv);
                    cv->popStateSet();

                    traversedChildren = true;
                
                    // debugging
                    if (camera->getName() == "dump")
                        local.dump(camera, *this);

                    local._projMatrixLastFrame = *cv->getProjectionMatrix();
                }
            }
        }
    }

    if (!traversedChildren)
    {
        osg::Group::traverse(nv);
    }
}

void
CascadeDrapingDecorator::reserveTextureImageUnit()
{
    if (_unit < 0)
    {
        static Threading::Mutex mutex;
        mutex.lock();

        osg::ref_ptr<TerrainResources> tr;
        if (_unit < 0 && _resources.lock(tr))
        {
            tr->reserveTextureImageUnit(_unit, "Draping");
        }

        mutex.unlock();
    }
}

osg::Node*
CascadeDrapingDecorator::getDump()
{
    osg::Node* n = _dump.release();
    _dump = 0L;
    return n;
}

//........................................................................

namespace
{
    /**
     * A camera that will traverse the per-thread DrapingCullSet instead of its own children.
     */
    class DrapingCamera : public osg::Camera
    {
    public:
        DrapingCamera(DrapingManager& dm, const osg::Camera* parentCamera) 
            : osg::Camera(), _parentCamera(parentCamera), _dm(dm)
        {
            setCullingActive( false );
            osg::StateSet* ss = getOrCreateStateSet();

            // do not sort geometry - draw it in traversal order without depth testing
            ss->setMode(GL_DEPTH_TEST, 0);
            ss->setRenderBinDetails(1, "TraversalOrderBin", osg::StateSet::OVERRIDE_PROTECTED_RENDERBIN_DETAILS);
        }

    public: // osg::Node

        void traverse(osg::NodeVisitor& nv)
        {
            DrapingCullSet& cullSet = _dm.get(_parentCamera);
            cullSet.accept( nv );

            // manhandle the render bin sorting, since OSG ignores the override
            // in the render bin details above
            osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
            if (cv)
            {
                applyTraversalOrderSorting(cv->getCurrentRenderBin());
            }
        }

        void applyTraversalOrderSorting(osgUtil::RenderBin* bin)
        {
            bin->setSortMode(osgUtil::RenderBin::TRAVERSAL_ORDER);

            for (osgUtil::RenderBin::RenderBinList::iterator i = bin->getRenderBinList().begin();
                i != bin->getRenderBinList().end();
                ++i)
            {
                osgUtil::RenderBin* child = i->second.get();
                applyTraversalOrderSorting(child);
            }
        }

    protected:
        virtual ~DrapingCamera() { }
        const osg::Camera* _parentCamera;
        DrapingManager& _dm;
    };


    //! Intersect a world-space ray with a plane.
    bool
    intersectRayWithPlane(const osg::Vec3d& L0,
                          const osg::Vec3d& L1,
                          const osg::Plane& plane,
                          osg::Vec3d& output)
    {
        osg::Vec3d L = L1 - L0;
        L.normalize();

        const osg::Plane::Vec3_type N = plane.getNormal();

        double dist = plane.distance(L0);

#if 0
        // Numerator always resolves to -dist, so this is commented out
        // as an optimization. Leave it here to show why.
        osg::Vec3d P0 = L0-N*dist;    // point on the plane
        double numer = (P0 - L0) * N; // always -dist
        if (numer == 0)
            return false;
#endif

        double numer = -dist;

        double denom = L * N;
        if (osg::equivalent(denom, 0.0))
            return false;

        double d = numer/denom;

        // enforce "ray" .. fail if intersection is behind L0
        if (d <= 0.0)
            return false;

        output = L0 + L*d;
        return true;
    }

    //! Intersect a world-space ray with the ellipsoid
    bool
    intersectRayWithEllipsoid(const osg::Vec3d& p0,
                              const osg::Vec3d& p1,
                              const osg::EllipsoidModel& ellipsoid,
                              osg::Vec3d& output)
    {
        // reference frame (converts world ellipsoid to unit sphere)
        osg::Vec3d unitSphereFrame(
            1.0/ellipsoid.getRadiusEquator(),
            1.0/ellipsoid.getRadiusEquator(),
            1.0/ellipsoid.getRadiusPolar());

        // get radius at p0:
        osg::Vec3d temp = osg::componentMultiply(p0, unitSphereFrame);
        temp.normalize();
        temp = osg::componentDivide(temp, unitSphereFrame);
        double R = temp.length();

        osg::Vec3d d = p1 - p0;

        double a = d * d;
        double b = 2.0 * (d*p0);
        double c = (p0*p0) - R*R;

        double t0, t1;

        double D = b*b - 4*a*c;
        if (D < 0.0)
            return false;

        double sD = sqrt(D);
        
        t0 = (-b+sD)/(2*a);
        t1 = (-b-sD)/(2*a);

        if (t0>t1)
            std::swap(t0,t1);

        if (t0 < 0.0)
            t0 = t1;

        if (t0 < 0.0)
            return false;

        osg::Vec3d v = d*t0;
        output = p0 + v;
        //maxDist = osg::maximum(maxDist, (output-p0).length());
        return true;
    }

    inline double mix(double a, double b, double t) 
    { 
        return a*(1.0 - t) + b*t;
    }
}

CascadeDrapingDecorator::CameraLocal::~CameraLocal()
{
    osg::ref_ptr<osg::Camera> camera;
    if (_token.lock(camera))
    {
        camera->removeObserver(this);
    }
}

void
CascadeDrapingDecorator::CameraLocal::objectDeleted(void* ptr)
{
    OE_DEBUG << "CameraLocal::objectDeleted" << std::endl;
    for (unsigned i = 0; i < 4; ++i)
    {
        _cascades[i]._rtt = 0L;
        _terrainSS = 0L;
    }
}

void
CascadeDrapingDecorator::CameraLocal::initialize(osg::Camera* camera, CascadeDrapingDecorator& decorator)
{
    // set up the auto-delete of orphaned cameras
    _token = camera;
    camera->getOrCreateObserverSet()->addObserver(this);

    unsigned textureWidth;
    unsigned textureHeight;

    unsigned multiSamples;
    osg::Texture::FilterMode minifyFilter, magnifyFilter;
    osg::Vec4 clearColor;
    bool mipmapping = decorator._mipmapping;

    // if the master cam is a picker, just limit to one cascade with no sampling.
    bool isPickCamera = camera->getName() == "osgEarth::RTTPicker";
    if (isPickCamera)
    {
        textureWidth = osg::minimum(512u, decorator._texSize);
        textureHeight = osg::minimum(512u, decorator._texSize);
        _maxCascades = 2u; // limit to one cascade when picking
        multiSamples = 0u; // no antialiasing allowed
        minifyFilter = osg::Texture::NEAREST; // no texture filtering allowed
        magnifyFilter = osg::Texture::NEAREST;
        clearColor.set(0,0,0,0);
        mipmapping = false;
    }
    else
    {
        textureWidth = decorator._texSize;
        textureHeight = decorator._texSize;
        _maxCascades = decorator._maxCascades;
        multiSamples = decorator._multisamples;
        minifyFilter = mipmapping? osg::Texture::LINEAR_MIPMAP_LINEAR : osg::Texture::LINEAR;
        magnifyFilter = osg::Texture::LINEAR;
        clearColor.set(1,1,1,0);
    }

    // Create the shared draping texture.
    osg::Texture2DArray* tex = new osg::Texture2DArray();
    tex->setTextureSize(textureWidth, textureHeight, osg::maximum(_maxCascades+1, 4u));
    tex->setInternalFormat(GL_RGBA);
    tex->setSourceFormat(GL_RGBA);
    tex->setSourceType(GL_UNSIGNED_BYTE);
    tex->setResizeNonPowerOfTwoHint(false);
    tex->setFilter(tex->MIN_FILTER, minifyFilter);
    tex->setFilter(tex->MAG_FILTER, magnifyFilter);
    tex->setWrap(tex->WRAP_S, tex->CLAMP_TO_EDGE);
    tex->setWrap(tex->WRAP_T, tex->CLAMP_TO_EDGE);
    tex->setMaxAnisotropy(4.0f);
    
    // set up the global RTT camera state:
    _rttSS = new osg::StateSet();
    osg::StateAttribute::OverrideValue forceOff = osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED | osg::StateAttribute::OVERRIDE;
    osg::StateAttribute::OverrideValue forceOn  = osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED | osg::StateAttribute::OVERRIDE;
    
    // blending:
    _rttSS->setAttributeAndModes(
        new osg::BlendFunc(GL_SRC_COLOR, GL_ONE_MINUS_SRC_COLOR, GL_ONE, GL_ONE_MINUS_SRC_ALPHA),
        forceOn);

    // Cannot do this because it will break picking -gw
    //VirtualProgram* rttVP = VirtualProgram::getOrCreate(_rttSS.get());
    //rttVP->setInheritShaders(false);

    for (unsigned i = 0; i < _maxCascades; ++i)
    {
        osg::ref_ptr<osg::Camera> rtt = new DrapingCamera(decorator._manager, camera);

        rtt->setClearColor(clearColor);

        if (decorator._debug && camera->getName() == "dump")
        {
            if (i==0 || i==4)
                rtt->setClearColor(osg::Vec4(1,0,0,0.15));
            else if (i==1 || i==5)
                rtt->setClearColor(osg::Vec4(0,1,0,0.15));
            else if (i==2 || i==6)
                rtt->setClearColor(osg::Vec4(0,0,1,0.15));
            else
                rtt->setClearColor(osg::Vec4(1,1,0,0.15));
        }

        rtt->setGraphicsContext(camera->getGraphicsContext());
        rtt->setReferenceFrame(rtt->ABSOLUTE_RF_INHERIT_VIEWPOINT);
        rtt->setViewport(0, 0, textureWidth, textureHeight);
        rtt->setRenderOrder(rtt->PRE_RENDER);
        rtt->setRenderTargetImplementation(rtt->FRAME_BUFFER_OBJECT);
        rtt->setComputeNearFarMode(rtt->DO_NOT_COMPUTE_NEAR_FAR);
        rtt->setImplicitBufferAttachmentMask(0, 0); // no implicit attachments!
        rtt->setDrawBuffer(GL_FRONT);
        rtt->setReadBuffer(GL_FRONT);

        rtt->attach(
            rtt->COLOR_BUFFER,        // target
            tex,                      // texture to populate
            0u,                       // mipmap level
            i,                        // texture array index
            mipmapping,               // mipmapping
            multiSamples);            // antialiasing multi-samples

        // Note: No depth buffer.

        // Set this so we can detect the RTT camera's parent for the DrapingCamera and
        // for things like auto-scaling, picking, etc.
        // GW: unnecessary unless we want to eventually share RTT cameras?
        rtt->setView(camera->getView());

        // no addChild() because the DrapingCamera will automatically traverse the current cull set

        _cascades[i]._rtt = rtt.get();
    }

    // Set up a stateSet for the terrain that will apply the projected texture.
    _terrainSS = new osg::StateSet();

    // bind the projected texture
    _terrainSS->setTextureAttributeAndModes(decorator._unit, tex, 1);
    _terrainSS->getOrCreateUniform("oe_Draping_tex", osg::Uniform::SAMPLER_2D)->set((int)decorator._unit);
    _terrainSS->setDefine("OE_DRAPING_MAX_CASCADES", Stringify() << decorator._maxCascades);
    
    // install the shader program to project a texture on the terrain
    VirtualProgram* drapingShader = VirtualProgram::getOrCreate(_terrainSS.get());
    drapingShader->setName("Draping");
    Shaders shaders;
    shaders.load(drapingShader, shaders.CascadeDraping);
}

void
CascadeDrapingDecorator::CameraLocal::clear()
{
    //todo - clear out unused RTT cameras/textures
}

void CascadeDrapingDecorator::Cascade::expandToInclude(const osg::Vec3d& pointView)
{
    _box.xMin() = osg::minimum(_box.xMin(), pointView.x());
    _box.yMin() = osg::minimum(_box.yMin(), pointView.y());
    _box.xMax() = osg::maximum(_box.xMax(), pointView.x());
    _box.yMax() = osg::maximum(_box.yMax(), pointView.y());
}

namespace
{
    void clipBox(osg::BoundingBoxd& box, const osg::BoundingBoxd& crop) //, double prevMaxY)
    {
        box.xMin() = box.xMin() !=  DBL_MAX ? osg::maximum(box.xMin(), crop.xMin()) : crop.xMin();
        box.xMax() = box.xMax() != -DBL_MAX ? osg::minimum(box.xMax(), crop.xMax()) : crop.xMax();
        box.yMin() = box.yMin() !=  DBL_MAX ? osg::maximum(box.yMin(), crop.yMin()) : crop.yMin();
        box.yMax() = box.yMax() != -DBL_MAX ? osg::minimum(box.yMax(), crop.yMax()) : crop.yMax();
    }
}

void CascadeDrapingDecorator::Cascade::computeProjection(const osg::Matrix& rttView,
                                                  const osg::Matrix& iCamMVP,
                                                  const osg::EllipsoidModel& ellipsoid,
                                                  const osg::Plane& plane,
                                                  double dp,
                                                  const osg::BoundingBoxd& rttBox)
{
    osg::EllipsoidModel e;

    // intersect the view frustum's edge vectors with the horizon plane
    osg::Vec3d LL, LR, UL, UR;
    bool LL_ok=false, LR_ok=false, UL_ok=false, UR_ok=false;
   
    osg::Vec3d N, F;

    N = osg::Vec3d(-1,_maxClipY,-1)*iCamMVP, F = osg::Vec3d(-1,_maxClipY,+1)*iCamMVP;
#ifdef USE_ELLIPSOID_INTERSECTIONS
    UL_ok = intersectRayWithEllipsoid(N, F, ellipsoid, UL);
#endif
    if (!UL_ok) UL_ok = intersectRayWithPlane(N, F, plane, UL);

    N = osg::Vec3d(+1,_maxClipY,-1)*iCamMVP, F = osg::Vec3d(+1,_maxClipY,+1)*iCamMVP;
#ifdef USE_ELLIPSOID_INTERSECTIONS
    UR_ok = intersectRayWithEllipsoid(N, F, ellipsoid, UR);
#endif
    if (!UR_ok) UR_ok = intersectRayWithPlane(N, F, plane, UR);
    
    N = osg::Vec3d(-1,_minClipY,-1)*iCamMVP, F = osg::Vec3d(-1,_minClipY,+1)*iCamMVP;
#ifdef USE_ELLIPSOID_INTERSECTIONS
    LL_ok = intersectRayWithEllipsoid(N, F, ellipsoid, LL);
#endif
    if (!LL_ok) LL_ok = intersectRayWithPlane(N, F, plane, LL);

    N = osg::Vec3d(+1,_minClipY,-1)*iCamMVP, F = osg::Vec3d(+1,_minClipY,+1)*iCamMVP;
#ifdef USE_ELLIPSOID_INTERSECTIONS
    LR_ok = intersectRayWithEllipsoid(N, F, ellipsoid, LR);
#endif
    if (!LR_ok) LR_ok = intersectRayWithPlane(N, F, plane, LR);

    // next, transform each frustum point into RTT view space (looking down),
    // and expand the orthographic extents to include all valid points    
    _box.set(DBL_MAX, DBL_MAX, DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX);

    // if all frustum edges intersect the horizon plane, calculate the region
    // bounded by the intersected area. Do this by projecting each point into
    // the RTT's view plane.
    if (UL_ok && UR_ok && LL_ok && LR_ok)
    {
        expandToInclude(UL*rttView);
        expandToInclude(UR*rttView);
        expandToInclude(LL*rttView);
        expandToInclude(LR*rttView);
    }

    // if only the LL and LR edges intersect, the view is pitched up over the horizon. 
    // Clamp the Y extent to 0 to prevent RTT-ing behind the camera.
    else if (LL_ok && LR_ok)
    {
        _box.yMin() = 0.0;
    }

    // If looking forward, clamp the minimum Y to zero so we don't
    // clip geometry in front of us
    if (_minClipY == -1.0 && _box.yMin() >= 0.0)
    {
        _box.yMin() = 0.0;

        // bump to near clip plane
        osg::Vec3d NP(0, -1, -1);
        osg::Vec3d RT = NP * iCamMVP * rttView;
        _box.yMin() = osg::maximum(0.0, RT.y());
    }

    // constrain by original RTT box (e.g., bounds of geometry)
    clipBox(_box, rttBox);

    // construct the projection matrix for RTT
    makeProj(dp);
}

void CascadeDrapingDecorator::Cascade::makeProj(double rttFar)
{
    _rttProj.makeOrtho(_box.xMin(), _box.xMax(), _box.yMin(), _box.yMax(), -rttFar*4, rttFar);
}

// Computes the "coverage" of the RTT region in normalized [0..1] clip space.
// If the width and height are 1.0, that means the RTT region will fit exactly 
// within the camera's viewport. For example, a heightNDC of 3.0 means that the
// RTT region is 3x "longer" than the viewport can accommodate; therefore we will need
// multiple cascades.
void CascadeDrapingDecorator::Cascade::computeClipCoverage(const osg::Matrix& rttView, const osg::Matrix& camMVP)
{
    // inverse of the RTT MVP matrix:
    osg::Matrix rttMVP = rttView * _rttProj;
    osg::Matrix rttMVPInv;
    rttMVPInv.invert(rttMVP);

    // matrix to transform from RTT clip to camera clip:
    osg::Matrix rttClipToCamClip = rttMVPInv * camMVP;

    osg::Vec3d winLL = osg::Vec3d(-1, _minClipY, +1) * rttClipToCamClip;
    osg::Vec3d winLR = osg::Vec3d(+1, _minClipY, +1) * rttClipToCamClip;
    osg::Vec3d winUL = osg::Vec3d(-1, _maxClipY, +1) * rttClipToCamClip;
    osg::Vec3d winUR = osg::Vec3d(+1, _maxClipY, +1) * rttClipToCamClip;

    // NEW:
    _minClipY = osg::minimum(winLL.y(), winLR.y());
    _maxClipY = osg::maximum(winUL.y(), winUR.y());

    // width and height [0..1]
    _widthNDC = std::max(winUR.x() - winUL.x(), winLR.x() - winLL.x())*0.5 + 0.5;
    _heightNDC = (std::max(winUL.y(), winUR.y()) - std::min(winLL.y(), winLR.y()))*0.5 + 0.5;
}

#define MAXABS4(A,B,C,D) \
    osg::maximum(fabs(A), osg::maximum(fabs(B), osg::maximum(fabs(C),fabs(D))))

void CascadeDrapingDecorator::CameraLocal::constrainRttBoxToFrustum(const osg::Matrix& iCamMVP, 
                                                             const osg::Matrix& rttView, 
                                                             const osg::EllipsoidModel& ellipsoid,
                                                             bool constrainY,
                                                             osg::BoundingBoxd& rttBox)
{
    // transform camera clip space into RTT view space (the maxExt space)
    osg::Matrix camProjToRttView = iCamMVP * rttView;

    if (constrainY)
    {
        // contrain both dimensions - in practice, this might not be necessary
        // because the far clip plane is always (?) beyond the horizon plane due to 
        // bounding-sphere culling. Also, doing this may cause the cascade sizes to
        // "jump" which might be an undesirable visual effect. -gw
        osg::Vec3d farLL = osg::Vec3d(-1,-1,+1)*camProjToRttView;
        osg::Vec3d farLR = osg::Vec3d(+1,-1,+1)*camProjToRttView;
        osg::Vec3d farUL = osg::Vec3d(-1,+1,+1)*camProjToRttView;
        osg::Vec3d farUR = osg::Vec3d(+1,+1,+1)*camProjToRttView;
        double x = osg::minimum(rttBox.xMax(), MAXABS4(farLL.x(), farLR.x(), farUL.x(), farUR.x()));
        double y = osg::minimum(rttBox.yMax(), MAXABS4(farLL.y(), farLR.y(), farUL.y(), farUR.y()));
        rttBox.set(-x, -y, 0, x, y, 0);
    }
    else
    {
        osg::Vec3d farLL = osg::Vec3d(-1,-1,+1)*camProjToRttView;
        osg::Vec3d farUR = osg::Vec3d(+1,+1,+1)*camProjToRttView;
        double x = osg::minimum(rttBox.xMax(), osg::maximum(fabs(farLL.x()), fabs(farUR.x())));
        rttBox.xMin() = -x, rttBox.xMax() = x;
    }
}

void CascadeDrapingDecorator::CameraLocal::constrainRttBoxToBounds(const osg::Matrix& rttView, 
                                                            const osg::BoundingSphere& bound, 
                                                            osg::BoundingBoxd& rttBox)
{
    // center point in RTT view space:
    osg::Vec3d c = bound.center() * rttView;

    // radius (no scale? what is the earth is scaled?)
    double r = bound.radius();
    
    rttBox.yMin() = osg::maximum(rttBox.yMin(), c.y()-r);
    rttBox.yMax() = osg::minimum(rttBox.yMax(), c.y()+r);
    rttBox.xMin() = osg::maximum(rttBox.xMin(), c.x()-r);
    rttBox.xMax() = osg::minimum(rttBox.xMax(), c.x()+r);
}

bool
CascadeDrapingDecorator::CameraLocal::intersectTerrain(CascadeDrapingDecorator& terrain, const osg::Vec3d& startWorld, const osg::Vec3d& endWorld, osg::Vec3d& outputWorld)
{
    osgUtil::LineSegmentIntersector* lsi = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::MODEL, startWorld, endWorld);
    lsi->setIntersectionLimit(lsi->LIMIT_NEAREST);
    osgUtil::IntersectionVisitor iv(lsi);
    terrain.accept(iv);
    bool hit = lsi->containsIntersections();
    if (hit)
    {
        outputWorld = lsi->getFirstIntersection().getWorldIntersectPoint();
    }
    return hit;
}

void
CascadeDrapingDecorator::CameraLocal::traverse(osgUtil::CullVisitor* cv, CascadeDrapingDecorator& decorator, const osg::BoundingSphere& bounds)
{    
    osg::Camera* camera = cv->getCurrentCamera();

    // first time through, intiailize the RTT cameras.
    if (_rttSS.valid() == false)
    {
        initialize(camera, decorator);
    }

    // establish the camera view vectors:
    const osg::Matrix& camView = camera->getViewMatrix();
    osg::Vec3d camEye, camCenter, camUp;
    camView.getLookAt(camEye, camCenter, camUp);
    osg::Vec3d camLook = camCenter-camEye;
    camLook.normalize();

    // camera modelview matrix (world -> view)
    const osg::Matrix& camMV = *cv->getModelViewMatrix();

    // camera projection matrix (from previous frame)
    const osg::Matrix& camProj = _projMatrixLastFrame;

    // camera world -> clip
    osg::Matrixd camMVP = camMV * camProj;

    // camera clip -> world
    osg::Matrixd iCamMVP;
    iCamMVP.invert(camMVP);

    // horizon plane (world space) - the plane passing through the
    // ellipsoid's visible horizon in all directions from the camera.
    osg::Plane horizonPlane;

    // distance to the visible horizon (in any direction)
    double dh;

    // Shortest distance from the camera to the horizon plane (straight down)
    double dp;
    
    // Maximum theorectical extent of the draping region; i.e. the distance
    // from the camera to the visible horizon projected onto the horizon plane.
    // This the largest possible extent we will need for RTT.
    osg::BoundingBoxd rttBox;
    
    osg::EllipsoidModel fakeEM;
    const osg::EllipsoidModel* ellipsoid = decorator._srs->getEllipsoid();

    if (decorator._srs->isGeographic())
    {
        Horizon* horizon = Horizon::get(*cv);
        horizon->getPlane(horizonPlane);
        dh = horizon->getDistanceToVisibleHorizon();
        dp = horizonPlane.distance(camEye);
    }
    else
    {
        // in projected mode, the horizon is at an infinite distance, so
        // we need to simulate it.
        dp = osg::maximum(camEye.z(), 100.0);
        dh = sqrt(2.0*6356752.3142*dp + dp*dp);
        horizonPlane.set(osg::Vec3d(0,0,1), dp);
    }

#if 0
    // intersect the terrain and build a custom horizon plane.
    osg::Vec3d camFar = osg::Vec3d(0, 0, 1) * iCamMVP;
    osg::Vec3d isect;
    if (intersectTerrain(decorator, camEye, camFar, isect))
    {
        //double diff = isect.length() - dp;
        //if (diff > 0.0)
        {
            fakeEM.setRadiusEquator(isect.length());
            fakeEM.setRadiusPolar(isect.length());
            osg::ref_ptr<Horizon> horizon = new Horizon(fakeEM);
            horizon->setEye(camEye);
            horizon->getPlane(horizonPlane);
            dh = horizon->getDistanceToVisibleHorizon();
            dp = horizonPlane.distance(camEye);
            ellipsoid = &fakeEM;
        }
    }
#endif

    // project visible horizon distance into the horizon plane:
    double m = sqrt(dh*dh - dp*dp);
    m = osg::minimum(m, decorator._maxHorizonDistance);
    rttBox.set(-m, -m, 0, m, m, 0);

    // Create a view matrix that looks straight down at the horizon plane form the eyepoint.
    // This will be our view matrix for all RTT draping cameras.
    osg::Matrix rttView;
    osg::Vec3d rttLook = -ellipsoid->computeLocalUpVector(camEye.x(), camEye.y(), camEye.z());
    rttLook.normalize();
    osg::Vec3d camLeft = camUp ^ camLook;
    osg::Vec3d rttUp = rttLook ^ camLeft;
    rttView.makeLookAt(camEye, camEye + rttLook, rttUp);

    bool colinear = (rttLook * camLook) > 0.99999;

    osg::Matrix iRttView;
    iRttView.invert(rttView);

    // So far, rttBox is a theoretical max. Now we can constrain it based on
    // the actual camera frustum and (optionally) far clip plane. 
    // Constraining the the far clip gives a tigher bounds, but can result in 
    // "resolution jumps" as the camera moves around and the far clip plane
    // jumps around.
    constrainRttBoxToFrustum(iCamMVP, rttView, *ellipsoid, decorator._constrainMaxYToFrustum, rttBox);

    // further constrain the max extent based on the bounds of the draped geometry
    if (decorator._constrainRttBoxToDrapingSetBounds)
    {
        constrainRttBoxToBounds(rttView, bounds, rttBox);
    }

    // camera view -> world
    osg::Matrix iCamMV;
    iCamMV.invert(camMV);

    // Start by computing the full extent of the RTT region. We will use the results
    // of this math to decide how many cascades we need to use. (We are using 
    // cascade[0] here as a temporary workspace, but will re-use it later for the actual
    // highest resolution cascade).
    _cascades[0]._minClipY = -1.0;
    _cascades[0]._maxClipY = 1.0;
    _cascades[0].computeProjection(rttView, iCamMVP, *ellipsoid, horizonPlane, dp, rttBox);

    // Next compute the extent, in pixels, of the full RTT. If it's larger than our
    // texture cascade size, we may need multiple cascases.
    _cascades[0].computeClipCoverage(rttView, camMVP);

    // Update the main RTT box with the new computation:
    rttBox = _cascades[0]._box;

    // Compute the nunber and size of the draping cascades.
    {
        // Interesect the terrain along the view vector.
        // Use this information to gauge the relative elevation near the camera
        // so we can increase the resolution of the near cascade if necessary.
        optional<double> firstYMax(rttBox.yMax());

#ifdef USE_TERRAIN_INTERSECTIONS
        if (rttBox.yMin() >= 0.0)
        {
            osg::Vec3d camFar = osg::Vec3d(0,0,1) * iCamMVP;
            osg::Vec3d isect;
            if (intersectTerrain(decorator, camEye, camFar, isect))
            {
                isect = isect * rttView;        // terrain look-at point in rttView space
                double hat = -isect.z();        // height above terrain
                double ymax = isect.y() + hat;
                firstYMax = ymax;
            }
        }
#endif

#ifdef DEBUG_CASCADES
        std::stringstream buf;
#endif

        // Height (Y-span) of the RTT box in meters
        double rttHeight = rttBox.yMax() - rttBox.yMin();

        // Height (Y-span) of the RTT box [0..1]
        double rttHeightNDC = osg::maximum(_cascades[0]._heightNDC, 1.0);

        // Number of texels it will take to represent the RTT frustum:
        double totalTexels = rttHeightNDC * decorator._texSize;

        // Texels still needed
        double remainingTexels = totalTexels;
        
#ifdef DEBUG_CASCADES
        buf << "\nTexels: " << totalTexels
            << "\nHeight: " << rttHeight << "m"
            << "\nHeightNDC: " << rttHeightNDC
            << "\nRttbox: " << rttBox.yMin() << " -> " << rttBox.yMax();
#endif

        // resolution, roughly
        double metersPerTexel = rttHeight / totalTexels;

        // Start with one cascade and keep adding them until we don't need any more.
        for(_numCascades = 0u; 
            remainingTexels > 0.0 && _numCascades < _maxCascades;
            _numCascades++)
        {
            Cascade& cascade = _cascades[_numCascades];
            
#ifdef DEBUG_CASCADES
            buf << "\nCascade " << _numCascades << ": ";
#endif

            // Number of texels this cascade will consume:
            double texels = osg::minimum((double)decorator._texSize, remainingTexels);

            // near extent in rtt view space:
            double ymin = _numCascades > 0? _cascades[_numCascades-1]._box.yMax() : rttBox.yMin();

            // far extent in rtt view space:
            double ymax = ymin + (texels * metersPerTexel);

            // If we calculated a special YMax for the first cascade, apply it now:
            if (_numCascades == 0 && firstYMax.isSet() && firstYMax.get() < ymax)
            {
                ymax = firstYMax.get();
            }

            // Set the Y extents of the RTT projection:
            cascade._box.yMin() = ymin;
            cascade._box.yMax() = ymax;
            
            // If this the final cascade, force the Y to the maximum:
            if (texels >= remainingTexels ||            // out of texels?
                ymax >= rttBox.yMax()     ||            // exceeded RTT range?
                _numCascades+1 == _maxCascades)         // exceeded cascade count?
            {
                cascade._box.yMax() = rttBox.yMax();
            }

            // Track the number of texels we still need:
            remainingTexels -= texels;

            // Calculate the X extents of the cascade (widest necessary to accommodate).            
            osg::Vec3d point(0.0, cascade._box.yMax(), -dp); // Point in RTT view space at yMax on the horizon plane
            osg::Vec3d camClip = point * iRttView * camMVP;  // Xform into camera's clip space

            osg::Vec3d P;
            intersectRayWithPlane(camEye, osg::Vec3d(+1,camClip.y(),+1)*iCamMVP, horizonPlane, P);
            cascade._box.xMax() = (P * rttView).x();
            cascade._box.xMin() = -cascade._box.xMax();

            // Finally assemble the projection matrix.
            cascade.makeProj(dp);

            if (decorator._useProjectionFitting && !colinear)
            {
                // now intersect with near plane:
                point.set(0.0, cascade._box.yMin(), -dp);
                camClip = point * iRttView * camMVP;
                intersectRayWithPlane(camEye, osg::Vec3d(+1,camClip.y(),-1)*iCamMVP, horizonPlane, P);
                double nfRatio = (P*rttView).x() / cascade._box.xMax();
                if (_numCascades == 0)
                    nfRatio = osg::maximum(nfRatio, decorator._minNearFarRatio);
                fitProjectionMatrix(cascade._rttProj, nfRatio);
            }

            
#ifdef DEBUG_CASCADES
            buf << "\n  Texels: " << texels
                << "\n  Range: " << cascade._box.yMin() << " -> " << cascade._box.yMax()
                << "\n  Coverage: " << 100*(texels/totalTexels) << "%"
                << "\n  MetersPerTexel: " << metersPerTexel;            
#endif
        }
        
#ifdef DEBUG_CASCADES
        if (camera->getName() == "dump")
            osgEarth::Registry::instance()->startActivity("Cascades", buf.str());
#endif
    }


    // xform from clip [-1..1] to texture [0..1] space
    static const osg::Matrix clipToTex =
        osg::Matrix::translate(1.0, 1.0, 1.0) *
        osg::Matrix::scale(0.5, 0.5, 0.5);
    
    // For each active cascade, configure its RTT camera and build the
    // texture projection matrix
    ArrayUniform texMat("oe_Draping_texMatrix", osg::Uniform::FLOAT_MAT4, _terrainSS.get(), decorator._maxCascades);
    unsigned i;

    for (i = 0; i < _numCascades; ++i)
    {
        Cascade& cascade = _cascades[i];
        osg::Camera* rtt = cascade._rtt.get();

        // configure the RTT camera's matrices:
        rtt->setViewMatrix(rttView);
        rtt->setProjectionMatrix(cascade._rttProj);

        // Create the texture matrix that will transform the RTT frame into texture [0..1] space.
        // Doing this on the CPU avoids precision errors on the GPU.
        texMat.setElement(i, iCamMV * rttView * cascade._rttProj * clipToTex);
    }

    if (i < _maxCascades)
    {
        // install a "marker" matrix that tells the shader we're past the final cascade.
        static osg::Matrix marker(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
        texMat.setElement(i, marker);
    }

    // traverse and write to the texture.
    if (_numCascades > 0u)
    {
        cv->pushStateSet(_rttSS.get());
        for (unsigned i = 0; i < _numCascades; ++i)
        {
            Cascade& c = _cascades[i];
            c._rtt->accept(*cv);
        }
        cv->popStateSet(); // _rttSS
    }
}

void
CascadeDrapingDecorator::CameraLocal::dump(const osg::Camera* cam, CascadeDrapingDecorator& decorator)
{
    static const char* fn = "CascadeDrapingDecoratorDump.osgb";
 
    decorator._dump = new osg::Group();

    // Main camera:
    {
        osgShadow::ConvexPolyhedron ph;
        ph.setToUnitFrustum();
        osg::Matrix mvp = cam->getViewMatrix() * cam->getProjectionMatrix();
        osg::Matrix imvp; imvp.invert(mvp);
        ph.transform( imvp, mvp );
        ph.dumpGeometry(0,0,0,fn);
        osg::ref_ptr<osg::Node> camNode = osgDB::readRefNodeFile(fn);
        camNode->setName("camera");
        decorator._dump->addChild(camNode.get());
    }

    // RTT cameras
    for (unsigned i=0; i<_numCascades; ++i)
    {
        const osg::Camera* rtt = _cascades[i]._rtt.get();
        osgShadow::ConvexPolyhedron ph;
        ph.setToUnitFrustum();
        osg::Matrix mvp = rtt->getViewMatrix() * rtt->getProjectionMatrix();
        osg::Matrix imvp; imvp.invert(mvp);
        ph.transform( imvp, mvp );
        ph.dumpGeometry(0, 0, 0, fn, osg::Vec4(1, 1, 0, 1), osg::Vec4(1, 1, 0, 0.25));
        osg::ref_ptr<osg::Node> camNode = osgDB::readRefNodeFile(fn);
        camNode->setName("rtt");
        decorator._dump->addChild(camNode.get());
    }
}
