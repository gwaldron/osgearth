/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#include <osgEarth/Registry>
#include <osgEarth/VirtualProgram>

#include <osg/Depth>
#include <osg/PolygonMode>
#include <osg/Texture2D>
#include <osg/Uniform>

#define LC "[ClampingTechnique] "

//#define OE_TEST if (_dumpRequested) OE_INFO << std::setprecision(9)
#define OE_TEST OE_NULL

using namespace osgEarth;

//---------------------------------------------------------------------------

namespace
{
    // Projection clamping callback that simply records the clamped matrix for
    // later use
    struct CPM : public osg::CullSettings::ClampProjectionMatrixCallback
    {
        void setup( osgUtil::CullVisitor* cv ) {
            _clampedProj.makeIdentity();
            _cv = cv;
        }
        bool clampProjectionMatrixImplementation(osg::Matrixf& projection, double& znear, double& zfar) const {
            return _cv->clampProjectionMatrixImplementation(projection, znear, zfar);
            OE_WARN << "NO!" << std::endl;
        }
        bool clampProjectionMatrixImplementation(osg::Matrixd& projection, double& znear, double& zfar) const {
            bool r = _cv->clampProjectionMatrixImplementation(projection, znear, zfar);
            if ( r ) _clampedProj = projection;
            return r;
        }
        
        osgUtil::CullVisitor* _cv;
        mutable osg::Matrixd  _clampedProj;
    };


    // Additional per-view data stored by the clamping technique.
    struct LocalPerViewData : public osg::Referenced
    {
        osg::ref_ptr<osg::Texture2D> _rttTexture;
        osg::ref_ptr<osg::StateSet>  _groupStateSet;
        osg::ref_ptr<osg::Uniform>   _camViewToDepthClipUniform;
        osg::ref_ptr<osg::Uniform>   _depthClipToCamViewUniform;
        osg::ref_ptr<CPM>            _cpm;
    };
}

//---------------------------------------------------------------------------

ClampingTechnique::ClampingTechnique() :
_textureSize     ( 4096 )
{
    // use the maximum available unit.
    _textureUnit = Registry::capabilities().getMaxGPUTextureUnits() - 1;
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

    // set up a callback to extract the overlay projection matrix.
    local->_cpm = new CPM();

    // create the projected texture:
    local->_rttTexture = new osg::Texture2D();
    local->_rttTexture->setTextureSize( *_textureSize, *_textureSize );
    local->_rttTexture->setInternalFormat( GL_DEPTH_COMPONENT );
    local->_rttTexture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
    local->_rttTexture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    local->_rttTexture->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_BORDER );
    local->_rttTexture->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_BORDER );
    local->_rttTexture->setBorderColor( osg::Vec4(1,1,1,1) );

    // set up the RTT camera:
    params._rttCamera = new osg::Camera();
    params._rttCamera->setReferenceFrame( osg::Camera::ABSOLUTE_RF_INHERIT_VIEWPOINT );
    params._rttCamera->setClearColor( osg::Vec4f(0,0,0,0) );
    params._rttCamera->setClearMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    params._rttCamera->setComputeNearFarMode( osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES );
    params._rttCamera->setViewport( 0, 0, *_textureSize, *_textureSize );
    params._rttCamera->setRenderOrder( osg::Camera::PRE_RENDER );
    params._rttCamera->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT );
    params._rttCamera->attach( osg::Camera::DEPTH_BUFFER, local->_rttTexture.get() );
    params._rttCamera->setClampProjectionMatrixCallback( local->_cpm.get() );

    // set up a StateSet for the RTT camera.
    osg::StateSet* rttStateSet = params._rttCamera->getOrCreateStateSet();

    // lighting is off. We don't want draped items to be lit.
    //rttStateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );

    rttStateSet->setMode(
        GL_BLEND, 
        osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

    // prevents wireframe mode in the depth camera.
    rttStateSet->setAttributeAndModes(
        new osg::PolygonMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL ),
        osg::StateAttribute::ON | osg::StateAttribute::PROTECTED );

#if 0 //OOPS this kills things like a vertical scale shader!!
    // installs a dirt-simple program for rendering the depth texture that
    // skips all the normal terrain rendering stuff
    osg::Program* depthProg = new osg::Program();
    depthProg->addShader(new osg::Shader(
        osg::Shader::VERTEX, 
        "void main() { gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; }\n"));
    depthProg->addShader(new osg::Shader(
        osg::Shader::FRAGMENT, 
        "void main() { gl_FragColor = vec4(1,1,1,1); }\n"));
    rttStateSet->setAttributeAndModes(
        depthProg,
        osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED );
#endif
    
    // attach the terrain to the camera.
    // todo: should probably protect this with a mutex.....
    params._rttCamera->addChild( _engine ); //params._terrainParent->getChild(0) ); // the terrain itself.

    // assemble the overlay graph stateset.
    local->_groupStateSet = new osg::StateSet();

    local->_groupStateSet->setTextureAttributeAndModes( 
        _textureUnit, 
        local->_rttTexture.get(), 
        osg::StateAttribute::ON );

    // set up depth test/write parameters for the overlay geometry:
    local->_groupStateSet->setAttributeAndModes(
        new osg::Depth( osg::Depth::LEQUAL, 0.0, 1.0, false ),
        osg::StateAttribute::ON );

    local->_groupStateSet->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );

    // sampler for depth map texture:
    local->_groupStateSet->getOrCreateUniform(
        "oe_clamp_depthtex", 
        osg::Uniform::SAMPLER_2D )->set( _textureUnit );

    // matrix that transforms a vert from EYE coords to the depth camera's CLIP coord.
    local->_camViewToDepthClipUniform = local->_groupStateSet->getOrCreateUniform( 
        "oe_clamp_eye2depthclipmat", 
        osg::Uniform::FLOAT_MAT4 );

    // matrix that transforms a vert from depth-cam CLIP coords to EYE coords.
    local->_depthClipToCamViewUniform = local->_groupStateSet->getOrCreateUniform( 
        "oe_clamp_depthclip2eyemat", 
        osg::Uniform::FLOAT_MAT4 );

    // make the shader that will do clamping and depth offsetting.
    VirtualProgram* vp = new VirtualProgram();
    vp->setName( "ClampingTechnique program" );
    local->_groupStateSet->setAttributeAndModes( vp, osg::StateAttribute::ON );

    // vertex shader - subgraph
    std::string vertexSource = Stringify()
        << "#version " << GLSL_VERSION_STR << "\n"
#ifdef OSG_GLES2_AVAILABLE
        << "precision mediump float;\n"
#endif
        // uniforms from this ClampingTechnique:
        << "uniform sampler2D oe_clamp_depthtex; \n"
        << "uniform mat4 oe_clamp_eye2depthclipmat; \n"
        << "uniform mat4 oe_clamp_depthclip2eyemat; \n"

        // uniforms from ClampableNode:
        << "uniform vec2 oe_clamp_bias; \n"
        << "uniform vec2 oe_clamp_range; \n"

        << "varying vec4 oe_clamp_simvert; \n"
        << "varying float oe_clamp_simvertrange; \n"

        << "void oe_clamp_vertex(void) \n"
        << "{ \n"
        // transform the vertex into the depth texture's clip coordinates.
        << "    vec4 v_eye_orig = gl_ModelViewMatrix * gl_Vertex; \n"
        << "    vec4 tc = oe_clamp_eye2depthclipmat * v_eye_orig; \n"

        // sample the depth map.
        << "    float d = texture2DProj( oe_clamp_depthtex, tc ).r; \n"

        // make a fake point in depth clip space and transform it back into eye coords.
        << "    vec4 p = vec4(tc.x, tc.y, d, 1.0); \n"
        << "    vec4 v_eye_clamped = oe_clamp_depthclip2eyemat * p; \n"

        // if the clamping distance is too big, bag it.
        << "    vec3 v_eye_orig3    = v_eye_orig.xyz/v_eye_orig.w;\n"
        << "    vec3 v_eye_clamped3 = v_eye_clamped.xyz/v_eye_clamped.w; \n"
        << "    float clamp_distance = length(v_eye_orig3 - v_eye_clamped3); \n"

        << "    const float maxClampDistance = 10000.0; \n"
        
        << "    if ( clamp_distance > maxClampDistance ) \n"
        << "    { \n"
        << "        gl_Position = gl_ProjectionMatrix * v_eye_orig; \n"
        // still have to populate these to nullify the depth offset code.
        << "        oe_clamp_simvert = gl_Position; \n"
        << "        oe_clamp_simvertrange = 1.0; \n"
        << "    } \n"
        << "    else \n"
        << "    { \n"

        // now simulate a "closer" vertex for depth offsetting.

        // remap depth offset based on camera distance to vertex. The farther you are away,
        // the more of an offset you need.

        << "        float range = length(v_eye_clamped3); \n"

        << "        float ratio = (clamp(range, oe_clamp_range[0], oe_clamp_range[1])-oe_clamp_range[0])/(oe_clamp_range[1]-oe_clamp_range[0]);\n"
        << "        float bias = oe_clamp_bias[0] + ratio * (oe_clamp_bias[1]-oe_clamp_bias[0]);\n"

        << "        vec3 adj_vec = normalize(v_eye_clamped3); \n"
        << "        vec3 v_eye_offset3 = v_eye_clamped3 - (adj_vec * bias); \n"

        << "        vec4 v_sim_eye = vec4( v_eye_offset3 * v_eye_clamped.w, v_eye_clamped.w ); \n"
        << "        oe_clamp_simvert = gl_ProjectionMatrix * v_sim_eye;\n"
        << "        oe_clamp_simvertrange = range - bias; \n"
        << "        gl_Position = gl_ProjectionMatrix * v_eye_clamped; \n"
        << "    } \n"
        << "} \n";

    vp->setFunction( "oe_clamp_vertex", vertexSource, ShaderComp::LOCATION_VERTEX_POST_LIGHTING );


    // fragment shader - depth offset apply
    std::string frag =
        "varying vec4 oe_clamp_simvert; \n"
        "varying float oe_clamp_simvertrange; \n"
        "void oe_clamp_fragment(inout vec4 color)\n"
        "{ \n"
        "    float sim_depth = 0.5 * (1.0+(oe_clamp_simvert.z/oe_clamp_simvert.w));\n"

        // if the offset pushed the Z behind the eye, the projection mapping will
        // result in a z>1. We need to bring these values back down to the 
        // near clip plan (z=0). We need to check simRange too before doing this
        // so we don't draw fragments that are legitimently beyond the far clip plane.
        "    if ( sim_depth > 1.0 && oe_clamp_simvertrange < 0.0 ) { sim_depth = 0.0; } \n"
        "    gl_FragDepth = max(0.0, sim_depth); \n"
        "}\n";

    vp->setFunction( "oe_clamp_fragment", frag, ShaderComp::LOCATION_FRAGMENT_PRE_LIGHTING );
}


void
ClampingTechnique::preCullTerrain(OverlayDecorator::TechRTTParams& params,
                                 osgUtil::CullVisitor*             cv )
{
    if ( !params._rttCamera.valid() && params._group->getNumChildren() > 0 )
    {
        setUpCamera( params );
    }
}


void
ClampingTechnique::cullOverlayGroup(OverlayDecorator::TechRTTParams& params,
                                    osgUtil::CullVisitor*            cv )
{
    if ( params._rttCamera.valid() )
    {
        // update the RTT camera.
        params._rttCamera->setViewMatrix      ( params._rttViewMatrix );
        params._rttCamera->setProjectionMatrix( params._rttProjMatrix );

        LocalPerViewData& local = *static_cast<LocalPerViewData*>(params._techniqueData.get());

        // prime our CPM with the current cull visitor:
        local._cpm->setup( cv );

        // create the depth texture (render the terrain to tex)
        params._rttCamera->accept( *cv );

        // construct a matrix that transforms from camera view coords to depth texture
        // clip coords directly. This will avoid precision loss in the 32-bit shader.
        static osg::Matrix s_scaleBiasMat = 
            osg::Matrix::translate(1.0,1.0,1.0) * 
            osg::Matrix::scale(0.5,0.5,0.5);

        osg::Matrix camViewToRttClip = 
            cv->getCurrentCamera()->getInverseViewMatrix() * 
            params._rttViewMatrix                          * 
            local._cpm->_clampedProj                       *
            s_scaleBiasMat;

        local._camViewToDepthClipUniform->set( camViewToRttClip );
        local._depthClipToCamViewUniform->set( osg::Matrix::inverse(camViewToRttClip) );
        
        // traverse the overlay nodes, applying the clamping shader.
        cv->pushStateSet( local._groupStateSet.get() );
        params._group->accept( *cv );
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

        OE_INFO << LC << "Using texture size = " << *_textureSize << std::endl;
    }
}


void
ClampingTechnique::onUninstall( TerrainEngineNode* engine )
{
    _engine = 0L;
}
