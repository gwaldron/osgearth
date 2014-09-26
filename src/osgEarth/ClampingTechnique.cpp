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
#include <osgEarth/ClampingTechnique>
#include <osgEarth/Capabilities>
#include <osgEarth/CullingUtils>
#include <osgEarth/Registry>
#include <osgEarth/VirtualProgram>
#include <osgEarth/MapNode>
#include <osgEarth/Utils>

#include <osg/Depth>
#include <osg/PolygonMode>
#include <osg/Texture2D>
#include <osg/Uniform>
#include <osg/ValueObject>
#include <osg/Timer>

#include <osgDB/WriteFile>

#define LC "[ClampingTechnique] "

//#define OE_TEST if (_dumpRequested) OE_INFO << std::setprecision(9)
#define OE_TEST OE_NULL

//#define USE_RENDER_BIN 1
#undef USE_RENDER_BIN

//#define DUMP_RTT_IMAGE 1
//#undef DUMP_RTT_IMAGE

//#define TIME_RTT_CAMERA 1

using namespace osgEarth;

//---------------------------------------------------------------------------

namespace
{
    osg::Group* s_providerImpl(MapNode* mapNode)
    {
        return mapNode ? mapNode->getOverlayDecorator()->getGroup<ClampingTechnique>() : 0L;
    }

#ifdef TIME_RTT_CAMERA
    static osg::Timer_t t0, t1;
    struct RttIn : public osg::Camera::DrawCallback {
        void operator()(osg::RenderInfo& r) const {
            t0 = osg::Timer::instance()->tick();
        }
    };
    struct RttOut : public osg::Camera::DrawCallback {
        void operator()(osg::RenderInfo& r) const {
            t1 = osg::Timer::instance()->tick();
            OE_NOTICE << "RTT = " << osg::Timer::instance()->delta_m(t0, t1) << "ms" << std::endl;
        }
    };
#endif
}

ClampingTechnique::TechniqueProvider ClampingTechnique::Provider = s_providerImpl;

//--------------------------------------------------------------------------


// SUPPORT_Z is a placeholder - we need to come up with another method for
// clamping verts relative to Z without needing the current Model Matrix
// (as we would now). Leave this #undef's until further notice.
#define SUPPORT_Z 1
#undef  SUPPORT_Z

namespace
{
    const char clampingVertexShader[] =

        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

         // uniforms from this ClampingTechnique:
         "uniform sampler2D oe_clamp_depthTex; \n"
         "uniform mat4 oe_clamp_cameraView2depthClip; \n"
#ifdef SUPPORT_Z
         "uniform mat4 oe_clamp_depthClip2depthView; \n"
         "uniform mat4 oe_clamp_depthView2cameraView; \n"
#else
         "uniform mat4 oe_clamp_depthClip2cameraView; \n"
#endif

         "uniform float oe_clamp_horizonDistance; \n"
         "varying float oe_clamp_alphaFactor; \n"

         "void oe_clamp_vertex(inout vec4 VertexVIEW) \n"
         "{ \n"
         //   start by mocing the vertex into view space.
         "    vec4 v_view_orig = VertexVIEW; \n"

         //   if the distance to the vertex is beyond the visible horizon,
         //   "hide" the vertex by setting its alpha component to 0.0.
         //   if this is the case, there's no point in continuing -- so we 
         //   would normally branch here, but since this happens on the fly 
         //   the shader engine will run both branches regardless. So keep going.
         "    float vert_distance = length(v_view_orig.xyz/v_view_orig.w); \n"
         "    oe_clamp_alphaFactor = clamp(oe_clamp_horizonDistance - vert_distance, 0.0, 1.0 ); \n"

         //   transform the vertex into the depth texture's clip coordinates.
         "    vec4 v_depthClip = oe_clamp_cameraView2depthClip * v_view_orig; \n"

         //   sample the depth map.
         "    float d = texture2DProj( oe_clamp_depthTex, v_depthClip ).r; \n"

         //   blank it out if it's at the far plane (no terrain visible)
         "    if ( d > 0.999999 ) { oe_clamp_alphaFactor = 0.0; } \n"

         //   now transform into depth-view space so we can apply the height-above-ground:
         "    vec4 p_depthClip = vec4(v_depthClip.x, v_depthClip.y, d, 1.0); \n"

#ifdef SUPPORT_Z
         "    vec4 p_depthView = oe_clamp_depthClip2depthView * p_depthClip; \n"

              // next, apply the vert's Z value for that ground offset.
              // TODO: This calculation is not right!
              //       I think perhaps the model matrix is not earth-aligned.
         "    p_depthView.z += gl_Vertex.z*gl_Vertex.w/p_depthView.w; \n"

              // then transform the vert back into camera view space.
         "    VertexVIEW = oe_clamp_depthView2cameraView * p_depthView; \n"
#else
              // transform the depth-clip point back into camera view coords.
         "    VertexVIEW = oe_clamp_depthClip2cameraView * p_depthClip; \n"
#endif
         "} \n";


    const char clampingFragmentShader[] =

        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "varying float oe_clamp_alphaFactor; \n"

        "void oe_clamp_fragment(inout vec4 color)\n"
        "{ \n"
             // adjust the alpha component to "hide" geometry beyond the visible horizon.
        "    color.a *= oe_clamp_alphaFactor; \n"
        "}\n";

}

//---------------------------------------------------------------------------

namespace
{
    // Additional per-view data stored by the clamping technique.
    struct LocalPerViewData : public osg::Referenced
    {
        osg::ref_ptr<osg::Texture2D> _rttTexture;
        osg::ref_ptr<osg::StateSet>  _groupStateSet;
        osg::ref_ptr<osg::Uniform>   _camViewToDepthClipUniform;
        osg::ref_ptr<osg::Uniform>   _depthClipToCamViewUniform;

        osg::ref_ptr<osg::Uniform>   _depthClipToDepthViewUniform;
        osg::ref_ptr<osg::Uniform>   _depthViewToCamViewUniform;

        osg::ref_ptr<osg::Uniform>   _horizonDistanceUniform;

        unsigned _renderLeafCount;

#ifdef DUMP_RTT_IMAGE
        osg::ref_ptr<osg::Image> _rttDebugImage;
#endif
    };

#ifdef DUMP_RTT_IMAGE
    struct DumpTex : public osg::Camera::DrawCallback
    {
        osg::ref_ptr<osg::Image> _tex;
        DumpTex(osg::Image* tex) : _tex(tex) { }
        void operator () (osg::RenderInfo& renderInfo) const
        {
            static int s_cc = 0;
            if ( s_cc++ % 60 == 0 ) {
                osgDB::writeImageFile(*_tex.get(), "rttimage.osgb");
            }
        }
    };
#endif
}

#ifdef USE_RENDER_BIN

//---------------------------------------------------------------------------
// Custom bin for clamping.

namespace
{
    struct ClampingRenderBin : public osgUtil::RenderBin
    {
        struct PerViewData // : public osg::Referenced
        {
            osg::observer_ptr<LocalPerViewData> _techData;
        };

        // shared across ALL render bin instances.
        typedef Threading::PerObjectMap<osg::Camera*, PerViewData> PerViewDataMap;
        PerViewDataMap* _pvd; 

        // support cloning (from RenderBin):
        virtual osg::Object* cloneType() const { return new ClampingRenderBin(); }
        virtual osg::Object* clone(const osg::CopyOp& copyop) const { return new ClampingRenderBin(*this,copyop); } // note only implements a clone of type.
        virtual bool isSameKindAs(const osg::Object* obj) const { return dynamic_cast<const ClampingRenderBin*>(obj)!=0L; }
        virtual const char* libraryName() const { return "osgEarth"; }
        virtual const char* className() const { return "ClampingRenderBin"; }


        // constructs the prototype for this render bin.
        ClampingRenderBin() : osgUtil::RenderBin()
        {
            this->setName( OSGEARTH_CLAMPING_BIN );
            _pvd = new PerViewDataMap();
        }

        ClampingRenderBin( const ClampingRenderBin& rhs, const osg::CopyOp& op )
            : osgUtil::RenderBin( rhs, op ), _pvd( rhs._pvd )
        {
            // zero out the stateset...dont' want to share that!
            _stateset = 0L;
        }

        // override.
        void sortImplementation()
        {
            copyLeavesFromStateGraphListToRenderLeafList();
        }

        // override.
        void drawImplementation(osg::RenderInfo& renderInfo, osgUtil::RenderLeaf*& previous)
        {
            // find and initialize the state set for this camera.
            if ( !_stateset.valid() )
            {
                osg::Camera* camera = renderInfo.getCurrentCamera();
                PerViewData& data = _pvd->get(camera);
                if ( data._techData.valid() )
                {
                    _stateset = data._techData->_groupStateSet.get();
                    LocalPerViewData* local = static_cast<LocalPerViewData*>(data._techData.get());
                    local->_renderLeafCount = _renderLeafList.size();
                }
            }

            osgUtil::RenderBin::drawImplementation( renderInfo, previous );
        }
    };
}

/** the static registration. */
extern "C" void osgEarth_clamping_bin_registration(void) {}
static osgEarthRegisterRenderBinProxy<ClampingRenderBin> s_regbin(OSGEARTH_CLAMPING_BIN);

#endif // USE_RENDER_BIN

//---------------------------------------------------------------------------

ClampingTechnique::ClampingTechnique() :
_textureSize( 1024 )
{
    // disable if GLSL is not supported
    _supported = Registry::capabilities().supportsGLSL();

    // use the maximum available unit.
    _textureUnit = Registry::capabilities().getMaxGPUTextureUnits() - 1;
}


bool
ClampingTechnique::hasData(OverlayDecorator::TechRTTParams& params) const
{
#ifdef USE_RENDER_BIN
    //TODO: reconsider
    return true;
#else
    return params._group->getNumChildren() > 0;
#endif
}


void
ClampingTechnique::reestablish(TerrainEngineNode* engine)
{
    // nop.
}

void
ClampingTechnique::setUpCamera(OverlayDecorator::TechRTTParams& params)
{
    // To store technique-specific per-view info:
    LocalPerViewData* local = new LocalPerViewData();
    params._techniqueData = local;

    // create the projected texture:
    local->_rttTexture = new osg::Texture2D();
    local->_rttTexture->setTextureSize( *_textureSize, *_textureSize );
    local->_rttTexture->setInternalFormat( GL_DEPTH_COMPONENT );
    local->_rttTexture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::NEAREST );
    local->_rttTexture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );

    // this is important. geometry that is outside the depth texture will clamp to the
    // closest edge value in the texture -- this is good when you are rendering a 
    // primitive that has one or more of its verts off-screen.
    local->_rttTexture->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
    local->_rttTexture->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );
    //local->_rttTexture->setBorderColor( osg::Vec4(0,0,0,1) );

    // set up the RTT camera:
    params._rttCamera = new osg::Camera();
    params._rttCamera->setReferenceFrame( osg::Camera::ABSOLUTE_RF_INHERIT_VIEWPOINT );
    params._rttCamera->setClearDepth( 1.0 );
    params._rttCamera->setClearMask( GL_DEPTH_BUFFER_BIT );
    params._rttCamera->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
    params._rttCamera->setViewport( 0, 0, *_textureSize, *_textureSize );
    params._rttCamera->setRenderOrder( osg::Camera::PRE_RENDER );
    params._rttCamera->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT );
    params._rttCamera->setImplicitBufferAttachmentMask(0, 0);
    params._rttCamera->attach( osg::Camera::DEPTH_BUFFER, local->_rttTexture.get() );

#ifdef DUMP_RTT_IMAGE
    local->_rttDebugImage = new osg::Image();
    local->_rttDebugImage->allocateImage(4096, 4096, 1, GL_RGB, GL_UNSIGNED_BYTE);
    memset( (void*)local->_rttDebugImage->getDataPointer(), 0xff, local->_rttDebugImage->getTotalSizeInBytes() );
    params._rttCamera->attach( osg::Camera::COLOR_BUFFER, local->_rttDebugImage.get() );
    params._rttCamera->setFinalDrawCallback( new DumpTex(local->_rttDebugImage.get()) );
#endif

#ifdef TIME_RTT_CAMERA
    params._rttCamera->setInitialDrawCallback( new RttIn() );
    params._rttCamera->setFinalDrawCallback( new RttOut() );
#endif

    // set up a StateSet for the RTT camera.
    osg::StateSet* rttStateSet = params._rttCamera->getOrCreateStateSet();

    rttStateSet->setMode(
        GL_BLEND, 
        osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

    // prevents wireframe mode in the depth camera.
    rttStateSet->setAttributeAndModes(
        new osg::PolygonMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL ),
        osg::StateAttribute::ON | osg::StateAttribute::PROTECTED );
    
    // attach the terrain to the camera.
    // todo: should probably protect this with a mutex.....
    params._rttCamera->addChild( _engine ); // the terrain itself.

    // assemble the overlay graph stateset.
    local->_groupStateSet = new osg::StateSet();

    // Required for now, otherwise GPU-clamped geometry will jitter sometimes.
    // TODO: figure out why and fix it. This is a workaround for now.
    local->_groupStateSet->setDataVariance( osg::Object::DYNAMIC );

    local->_groupStateSet->setTextureAttributeAndModes( 
        _textureUnit, 
        local->_rttTexture.get(), 
        osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );

    // set up depth test/write parameters for the overlay geometry:
    local->_groupStateSet->setAttributeAndModes(
        new osg::Depth( osg::Depth::LEQUAL, 0.0, 1.0, true ),
        osg::StateAttribute::ON );

    local->_groupStateSet->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );

    // uniform for the horizon distance (== max clamping distance)
    local->_horizonDistanceUniform = local->_groupStateSet->getOrCreateUniform(
        "oe_clamp_horizonDistance",
        osg::Uniform::FLOAT );

    // sampler for depth map texture:
    local->_groupStateSet->getOrCreateUniform(
        "oe_clamp_depthTex", 
        osg::Uniform::SAMPLER_2D )->set( _textureUnit );

    // matrix that transforms a vert from EYE coords to the depth camera's CLIP coord.
    local->_camViewToDepthClipUniform = local->_groupStateSet->getOrCreateUniform( 
        "oe_clamp_cameraView2depthClip", 
        osg::Uniform::FLOAT_MAT4 );

#ifdef SUPPORT_Z

    // matrix that transforms a vert from depth clip coords to depth view coords.
    local->_depthClipToDepthViewUniform = local->_groupStateSet->getOrCreateUniform(
        "oe_clamp_depthClip2depthView",
        osg::Uniform::FLOAT_MAT4 );

    // matrix that transforms a vert from depth view coords to camera view coords.
    local->_depthViewToCamViewUniform = local->_groupStateSet->getOrCreateUniform(
        "oe_clamp_depthView2cameraView",
        osg::Uniform::FLOAT_MAT4 );

#else

    // matrix that transforms a vert from depth-cam CLIP coords to EYE coords.
    local->_depthClipToCamViewUniform = local->_groupStateSet->getOrCreateUniform( 
        "oe_clamp_depthClip2cameraView", 
        osg::Uniform::FLOAT_MAT4 );

#endif

    // make the shader that will do clamping and depth offsetting.
    VirtualProgram* vp = VirtualProgram::getOrCreate(local->_groupStateSet.get());
    vp->setName( "ClampingTechnique" );
    vp->setFunction( "oe_clamp_vertex",   clampingVertexShader,   ShaderComp::LOCATION_VERTEX_VIEW );
    vp->setFunction( "oe_clamp_fragment", clampingFragmentShader, ShaderComp::LOCATION_FRAGMENT_COLORING );
}


void
ClampingTechnique::preCullTerrain(OverlayDecorator::TechRTTParams& params,
                                 osgUtil::CullVisitor*             cv )
{
    if ( !params._rttCamera.valid() && hasData(params) )
    {
        setUpCamera( params );

#ifdef USE_RENDER_BIN

        // store our camera's stateset in the perview data.
        ClampingRenderBin* bin = dynamic_cast<ClampingRenderBin*>( osgUtil::RenderBin::getRenderBinPrototype(OSGEARTH_CLAMPING_BIN) );
        if ( bin )
        {
            ClampingRenderBin::PerViewData& data = bin->_pvd->get( cv->getCurrentCamera() );
            LocalPerViewData* local = static_cast<LocalPerViewData*>(params._techniqueData.get());
            data._techData = local;
        }
        else
        {
            OE_WARN << LC << "Odd, no prototype found for the clamping bin." << std::endl;
        }

#endif
    }

#ifdef TIME_RTT_CAMERA
#endif
}


void
ClampingTechnique::cullOverlayGroup(OverlayDecorator::TechRTTParams& params,
                                    osgUtil::CullVisitor*            cv )
{
    if ( params._rttCamera.valid() && hasData(params) )
    {
        // update the RTT camera.
        params._rttCamera->setViewMatrix      ( params._rttViewMatrix );
        params._rttCamera->setProjectionMatrix( params._rttProjMatrix );

        LocalPerViewData& local = *static_cast<LocalPerViewData*>(params._techniqueData.get());

#if 0
        osg::Vec3d eye, lookat, up;
        params._rttViewMatrix.getLookAt(eye, lookat, up);
        OE_WARN << "rtt eye=" << eye.x() << ", " << eye.y() << ", " << eye.z() << std::endl;

        double left, right, bottom, top, n, f;
        params._rttProjMatrix.getOrtho(left, right, bottom, top, n, f);
        OE_WARN << "rtt prj=" << left << ", " << right << ", " << bottom << ", " << top << ", " << n << ", " << f << std::endl << std::endl;
#endif

        // create the depth texture (render the terrain to tex)
        params._rttCamera->accept( *cv );


        // construct a matrix that transforms from camera view coords to depth texture
        // clip coords directly. This will avoid precision loss in the 32-bit shader.
        static osg::Matrix s_scaleBiasMat = 
            osg::Matrix::translate(1.0,1.0,1.0) * 
            osg::Matrix::scale    (0.5,0.5,0.5);

        static osg::Matrix s_invScaleBiasMat = osg::Matrix::inverse(
            osg::Matrix::translate(1.0,1.0,1.0) * 
            osg::Matrix::scale    (0.5,0.5,0.5) );

        osg::Matrix vm;
        vm.invert( *cv->getModelViewMatrix() );
        osg::Matrix cameraViewToDepthView =
            vm *
            params._rttViewMatrix;

        osg::Matrix depthViewToDepthClip = 
            params._rttProjMatrix *
            s_scaleBiasMat;

        osg::Matrix cameraViewToDepthClip =
            cameraViewToDepthView *
            depthViewToDepthClip;
        local._camViewToDepthClipUniform->set( cameraViewToDepthClip );

        local._horizonDistanceUniform->set( float(*params._horizonDistance) );

        //OE_NOTICE << "HD = " << std::setprecision(8) << float(*params._horizonDistance) << std::endl;

#if SUPPORT_Z
        osg::Matrix depthClipToDepthView;
        depthClipToDepthView.invert( depthViewToDepthClip );
        local._depthClipToDepthViewUniform->set( depthClipToDepthView );

        osg::Matrix depthViewToCameraView;
        depthViewToCameraView.invert( cameraViewToDepthView );
        local._depthViewToCamViewUniform->set( depthViewToCameraView );
#else

        osg::Matrix depthClipToCameraView;
        depthClipToCameraView.invert( cameraViewToDepthClip );
        local._depthClipToCamViewUniform->set( depthClipToCameraView );
#endif

        if ( params._group->getNumChildren() > 0 )
        {
            // traverse the overlay nodes, applying the clamping shader.
            cv->pushStateSet( local._groupStateSet.get() );

            // Since the vertex shader is moving the verts to clamp them to the terrain,
            // OSG will not be able to properly cull the geometry. (Specifically: OSG may
            // cull geometry which is invisible when NOT clamped, but becomes visible after
            // GPU clamping.) We work around that by using a Proxy cull visitor that will 
            // use the RTT camera's matrixes for frustum culling (instead of the main camera's).
            ProxyCullVisitor pcv( cv, params._rttProjMatrix, params._rttViewMatrix );

            // cull the clampable geometry.
            params._group->accept( pcv );
            //params._group->accept( *cv ); // old way - direct traversal

            // done; pop the clamping shaders.
            cv->popStateSet();
        }
    }
}


void
ClampingTechnique::setTextureSize( int texSize )
{
    if ( texSize != _textureSize.value() )
    {
        _textureSize = texSize;
    }
}


void
ClampingTechnique::onInstall( TerrainEngineNode* engine )
{
    // save a pointer to the terrain engine.
    _engine = engine;

    if ( !_textureSize.isSet() )
    {
        unsigned maxSize = Registry::capabilities().getMaxFastTextureSize();
        _textureSize.init( osg::minimum( 4096u, maxSize ) );

        OE_INFO << LC << "Using texture size = " << *_textureSize << std::endl;
    }
}


void
ClampingTechnique::onUninstall( TerrainEngineNode* engine )
{
    _engine = 0L;
}
