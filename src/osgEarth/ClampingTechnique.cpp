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
#include <osgEarth/ClampingTechnique>
#include <osgEarth/Capabilities>
#include <osgEarth/CullingUtils>
#include <osgEarth/Registry>
#include <osgEarth/Shaders>

#include <osg/Depth>
#include <osg/PolygonMode>
#include <osg/Texture2D>

#define LC "[ClampingTechnique] "

//#define OE_TEST if (_dumpRequested) OE_INFO << std::setprecision(9)
#define OE_TEST OE_NULL

//#define DUMP_RTT_IMAGE 1
//#undef DUMP_RTT_IMAGE

//#define TIME_RTT_CAMERA 1

using namespace osgEarth;

//---------------------------------------------------------------------------

#ifdef TIME_RTT_CAMERA
namespace
{
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
}
#endif

//---------------------------------------------------------------------------

namespace
{
    // Additional per-view data stored by the clamping technique.
    struct LocalPerViewData : public osg::Object
    {
        osg::ref_ptr<osg::Texture2D> _rttTexture;
        osg::ref_ptr<osg::StateSet>  _groupStateSet;
        osg::ref_ptr<osg::Uniform>   _camViewToDepthClipUniform;
        osg::ref_ptr<osg::Uniform>   _depthClipToCamViewUniform;

        osg::ref_ptr<osg::Uniform>   _depthClipToDepthViewUniform;
        osg::ref_ptr<osg::Uniform>   _depthViewToCamViewUniform;

        osg::ref_ptr<osg::Uniform>   _horizonDistance2Uniform;

        unsigned _renderLeafCount;

        META_Object(osgEarth,LocalPerViewData);
        LocalPerViewData() { }
        LocalPerViewData(const LocalPerViewData& rhs, const osg::CopyOp& co) { }
        
        void resizeGLObjectBuffers(unsigned maxSize) {
            if (_rttTexture.valid())
                _rttTexture->resizeGLObjectBuffers(maxSize);
            if (_groupStateSet.valid())
                _groupStateSet->resizeGLObjectBuffers(maxSize);
        }
        void releaseGLObjects(osg::State* state) const {
            if (_rttTexture.valid())
                _rttTexture->releaseGLObjects(state);
            if (_groupStateSet.valid())
                _groupStateSet->releaseGLObjects(state);
        }


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

//---------------------------------------------------------------------------

ClampingTechnique::ClampingTechnique() :
_textureSize( 1024 ),
_engine(0L)
{
    // disable if GLSL is not supported
    _supported = Registry::capabilities().supportsGLSL();

    // use the maximum available unit.
    _textureUnit = Registry::capabilities().getMaxGPUTextureUnits() - 1;
}


bool
ClampingTechnique::hasData(OverlayDecorator::TechRTTParams& params) const
{
    return getBound(params).valid();
}


void
ClampingTechnique::reestablish(TerrainEngineNode* engine)
{
    // nop.
}

void
ClampingTechnique::setUpCamera(OverlayDecorator::TechRTTParams& params)
{
    OE_INFO << LC << "Using texture size = " << _textureSize.get() << std::endl;

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

    // set up the RTT camera for rendering a depth map of the terrain:
    params._rttCamera = new osg::Camera();
    params._rttCamera->setName("GPU Clamping");
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
    
    // matrix that transforms the RTT's view to the main camera's view (for morphing support)
    params._rttToPrimaryMatrixUniform = rttStateSet->getOrCreateUniform(
        "oe_shadowToPrimaryMatrix",
        osg::Uniform::FLOAT_MAT4);

    // We use this define to tell the terrain it's a depth-only camera
    rttStateSet->setDefine("OE_IS_DEPTH_CAMERA");

    // install a VP on the stateset that cancels out any higher-up VP code.
    // This will prevent things like VPs on the main camera (e.g., log depth buffer)
    // from interfering with the depth camera
    VirtualProgram* rttVP = VirtualProgram::getOrCreate(rttStateSet);
    rttVP->setName("GPU Clamping RTT");
    rttVP->setInheritShaders(false);
    
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

    // set a define so the shaders know we are running GPU clamping.
    local->_groupStateSet->setDefine("OE_GPU_CLAMPING");

    // uniform for the horizon distance (== max clamping distance)
    local->_horizonDistance2Uniform = local->_groupStateSet->getOrCreateUniform(
        "oe_clamp_horizonDistance2",
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

    // default value for altitude offset; can be overriden by geometry.
    local->_groupStateSet->addUniform( new osg::Uniform(Clamping::AltitudeOffsetUniformName, 0.0f) );

    // make the shader that will do clamping and depth offsetting.
    VirtualProgram* vp = VirtualProgram::getOrCreate(local->_groupStateSet.get());
    vp->setName( "GPUClamping" );

    // Bind clamping attribute location, and a default uniform indicating whether
    // they are available (default is false).
    vp->addBindAttribLocation( Clamping::AnchorAttrName, Clamping::AnchorAttrLocation );
    local->_groupStateSet->addUniform( new osg::Uniform(Clamping::HasAttrsUniformName, false) );

    // Bind clamping heights location.
    vp->addBindAttribLocation( Clamping::HeightsAttrName, Clamping::HeightsAttrLocation );

    osgEarth::Shaders pkg;
    pkg.load(vp, pkg.GPUClampingVertex);
    pkg.load(vp, pkg.GPUClampingFragment);
}


void
ClampingTechnique::preCullTerrain(OverlayDecorator::TechRTTParams& params,
                                 osgUtil::CullVisitor*             cv )
{
    if ( !params._rttCamera.valid() && hasData(params) )
    {
        setUpCamera( params );
    }
}

       
const osg::BoundingSphere&
ClampingTechnique::getBound(OverlayDecorator::TechRTTParams& params) const
{
    return _clampingManager.get(params._mainCamera).getBound();
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
        
        // set the primary-camera-to-rtt-camera transformation matrix,
        // which lets you perform vertex shader operations from the perspective
        // of the primary camera (morphing, etc.) so that things match up
        // between the two cameras.
        osg::Matrix viewMatrixInverse = osg::Matrix::inverse(params._rttViewMatrix);
        params._rttToPrimaryMatrixUniform->set(viewMatrixInverse * (*cv->getModelViewMatrix()));

        // create the depth texture (render the terrain to tex)
        params._rttCamera->accept( *cv );
        

        LocalPerViewData& local = *static_cast<LocalPerViewData*>(params._techniqueData.get());

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

        float hd = (float)(*params._horizonDistance);
        local._horizonDistance2Uniform->set( hd*hd );

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

        // traverse the overlay nodes, applying the clamping shader.
        cv->pushStateSet(local._groupStateSet.get());

        // Since the vertex shader is moving the verts to clamp them to the terrain,
        // OSG will not be able to properly cull the geometry. (Specifically: OSG may
        // cull geometry which is invisible when NOT clamped, but becomes visible after
        // GPU clamping.) We work around that by using a Proxy cull visitor that will 
        // use the RTT camera's matrixes for frustum culling (instead of the main camera's).
        ProxyCullVisitor pcv(cv, params._rttProjMatrix, params._rttViewMatrix);

        // cull the clampable geometry.
        getClampingManager().get(cv->getCurrentCamera()).accept(pcv);

        // done; pop the clamping shaders.
        cv->popStateSet();
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
    }
}


void
ClampingTechnique::onUninstall( TerrainEngineNode* engine )
{
    _engine = 0L;
}
