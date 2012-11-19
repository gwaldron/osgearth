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
#include <osgEarth/DrapingTechnique>
#include <osgEarth/Capabilities>
#include <osgEarth/Registry>
#include <osgEarth/VirtualProgram>

#include <osg/BlendFunc>
#include <osg/TexGen>
#include <osg/Texture2D>
#include <osg/Uniform>

#define LC "[DrapingTechnique] "

//#define OE_TEST if (_dumpRequested) OE_INFO << std::setprecision(9)
#define OE_TEST OE_NULL

using namespace osgEarth;

//---------------------------------------------------------------------------

namespace
{
    // Additional per-view data stored by the draping technique.
    struct LocalPerViewData : public osg::Referenced
    {
        osg::ref_ptr<osg::Uniform> _texGenUniform;  // when shady
        osg::ref_ptr<osg::TexGen>  _texGen;         // when not shady
    };
}

//---------------------------------------------------------------------------

DrapingTechnique::DrapingTechnique() :
_textureUnit     ( 1 ),
_textureSize     ( 1024 ),
_useShaders      ( false ),
_mipmapping      ( false ),
_rttBlending     ( true ),
_attachStencil   ( true )
{
    // nop
}


bool
DrapingTechnique::hasData(OverlayDecorator::TechRTTParams& params) const
{
    return params._group->getNumChildren() > 0;
}


void
DrapingTechnique::reestablish(TerrainEngineNode* engine)
{
    if ( !_textureUnit.isSet() )
    {
        // apply the user-request texture unit, if applicable:
        if ( _explicitTextureUnit.isSet() )
        {
            if ( !_textureUnit.isSet() || *_textureUnit != *_explicitTextureUnit )
            {
                _textureUnit = *_explicitTextureUnit;
            }
        }

        // otherwise, automatically allocate a texture unit if necessary:
        else if ( !_textureUnit.isSet() )
        {
            int texUnit;
            if ( engine->getTextureCompositor()->reserveTextureImageUnit( texUnit ) )
            {
                _textureUnit = texUnit;
                OE_INFO << LC << "Reserved texture image unit " << *_textureUnit << std::endl;
            }
            else
            {
                OE_WARN << LC << "Uh oh, no texture image units available." << std::endl;
            }
        }
    }
}


void
DrapingTechnique::setUpCamera(OverlayDecorator::TechRTTParams& params)
{
    // create the projected texture:
    osg::Texture2D* projTexture = new osg::Texture2D();
    projTexture->setTextureSize( *_textureSize, *_textureSize );
    projTexture->setInternalFormat( GL_RGBA );
    projTexture->setSourceFormat( GL_RGBA );
    projTexture->setSourceType( GL_UNSIGNED_BYTE );
    projTexture->setFilter( osg::Texture::MIN_FILTER, _mipmapping? osg::Texture::LINEAR_MIPMAP_LINEAR: osg::Texture::LINEAR );
    projTexture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    projTexture->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_BORDER );
    projTexture->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_BORDER );
    //projTexture->setWrap( osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_EDGE );
    projTexture->setBorderColor( osg::Vec4(0,0,0,0) );

    // set up the RTT camera:
    params._rttCamera = new osg::Camera();
    params._rttCamera->setClearColor( osg::Vec4f(0,0,0,0) );

    // this ref frame causes the RTT to inherit its viewpoint from above (in order to properly
    // process PagedLOD's etc. -- it doesn't affect the perspective of the RTT camera though)
    params._rttCamera->setReferenceFrame( osg::Camera::ABSOLUTE_RF_INHERIT_VIEWPOINT );
    params._rttCamera->setViewport( 0, 0, *_textureSize, *_textureSize );
    params._rttCamera->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
    params._rttCamera->setRenderOrder( osg::Camera::PRE_RENDER );
    params._rttCamera->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT );
    params._rttCamera->attach( osg::Camera::COLOR_BUFFER, projTexture, 0, 0, _mipmapping );

    if ( _attachStencil )
    {
        // try a depth-packed buffer. failing that, try a normal one.. if the FBO doesn't support
        // that (which is doesn't on some GPUs like Intel), it will automatically fall back on 
        // a PBUFFER_RTT impl
        if ( Registry::capabilities().supportsDepthPackedStencilBuffer() )
        {
#ifdef OSG_GLES2_AVAILABLE 
            params._rttCamera->attach( osg::Camera::PACKED_DEPTH_STENCIL_BUFFER, GL_DEPTH24_STENCIL8_EXT );
#else
            params._rttCamera->attach( osg::Camera::PACKED_DEPTH_STENCIL_BUFFER, GL_DEPTH_STENCIL_EXT );
#endif
        }
        else
        {
            params._rttCamera->attach( osg::Camera::STENCIL_BUFFER, GL_STENCIL_INDEX );
        }

        params._rttCamera->setClearStencil( 0 );
        params._rttCamera->setClearMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );
    }
    else
    {
        params._rttCamera->setClearMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    }

    // set up a StateSet for the RTT camera.
    osg::StateSet* rttStateSet = params._rttCamera->getOrCreateStateSet();

    // lighting is off. We don't want draped items to be lit.
    rttStateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );

    // install a new default shader program that replaces anything from above.
    if ( _useShaders )
    {
        VirtualProgram* vp = new VirtualProgram();
        vp->setName( "DrapingTechnique RTT" );
        vp->installDefaultColoringAndLightingShaders();
        vp->setInheritShaders( false );
        rttStateSet->setAttributeAndModes( vp, osg::StateAttribute::ON );
    }
    
    // active blending within the RTT camera's FBO
    if ( _rttBlending )
    {
        //Setup a separate blend function for the alpha components and the RGB components.  
        //Because the destination alpha is initialized to 0 instead of 1
        osg::BlendFunc* blendFunc = 0;        
        if (Registry::capabilities().supportsGLSL(1.4f))
        {
            //Blend Func Separate is only available on OpenGL 1.4 and above
            blendFunc = new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
        }
        else
        {
            blendFunc = new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }

        rttStateSet->setAttributeAndModes(blendFunc, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    }
    else
    {
        rttStateSet->setMode(GL_BLEND, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
    }

    // attach the overlay group to the camera. 
    // TODO: we should probably lock this since other cull traversals might be accessing the group
    //       while we are changing its children.
    params._rttCamera->addChild( params._group );

    // overlay geometry is rendered with no depth testing, and in the order it's found in the
    // scene graph... until further notice.
    rttStateSet->setMode(GL_DEPTH_TEST, 0);
    rttStateSet->setBinName( "TraversalOrderBin" );

    // add to the terrain stateset, i.e. the stateset that the OverlayDecorator will
    // apply to the terrain before cull-traversing it. This will activate the projective
    // texturing on the terrain.
    params._terrainStateSet->setTextureAttributeAndModes( *_textureUnit, projTexture, osg::StateAttribute::ON );

    // fire up the local per-view data:
    LocalPerViewData* local = new LocalPerViewData();
    params._techniqueData = local;
    
    if ( _useShaders )
    {            
        // GPU path

        VirtualProgram* vp = new VirtualProgram();
        vp->setName( "DrapingTechnique terrain shaders");
        params._terrainStateSet->setAttributeAndModes( vp, osg::StateAttribute::ON );

        // sampler for projected texture:
        params._terrainStateSet->getOrCreateUniform( "oe_overlay_ProjTex", osg::Uniform::SAMPLER_2D )->set( *_textureUnit );

        // the texture projection matrix uniform.
        local->_texGenUniform = params._terrainStateSet->getOrCreateUniform( "oe_overlay_TexGenMatrix", osg::Uniform::FLOAT_MAT4 );

        // vertex shader - subgraph
        std::string vertexSource = Stringify()
            << "#version " << GLSL_VERSION_STR << "\n"
#ifdef OSG_GLES2_AVAILABLE
            << "precision mediump float;\n"
#endif
            << "uniform mat4 oe_overlay_TexGenMatrix; \n"
            << "uniform mat4 osg_ViewMatrixInverse; \n"
            << "varying vec4 osg_TexCoord[" << Registry::capabilities().getMaxGPUTextureCoordSets() << "]; \n"

            << "void oe_overlay_vertex(void) \n"
            << "{ \n"
            << "    osg_TexCoord["<< *_textureUnit << "] = oe_overlay_TexGenMatrix * osg_ViewMatrixInverse * gl_ModelViewMatrix * gl_Vertex; \n"
            << "} \n";

        vp->setFunction( "oe_overlay_vertex", vertexSource, ShaderComp::LOCATION_VERTEX_POST_LIGHTING );

        // fragment shader - subgraph
        std::string fragmentSource = Stringify()
            << "#version " << GLSL_VERSION_STR << "\n"
#ifdef OSG_GLES2_AVAILABLE
            << "precision mediump float;\n"
#endif
            << "uniform sampler2D oe_overlay_ProjTex; \n"
            << "varying vec4 osg_TexCoord[" << Registry::capabilities().getMaxGPUTextureCoordSets() << "]; \n"
            << "void oe_overlay_fragment( inout vec4 color ) \n"
            << "{ \n"
            << "    vec2 texCoord = osg_TexCoord["<< *_textureUnit << "].xy / osg_TexCoord["<< *_textureUnit << "].q; \n"
            << "    vec4 texel = texture2D(oe_overlay_ProjTex, texCoord); \n"  
            << "    color = vec4( mix( color.rgb, texel.rgb, texel.a ), color.a); \n"
            << "} \n";

        vp->setFunction( "oe_overlay_fragment", fragmentSource, ShaderComp::LOCATION_FRAGMENT_POST_LIGHTING );
    }
    else
    {
        // FFP path
        local->_texGen = new osg::TexGen();
        local->_texGen->setMode( osg::TexGen::EYE_LINEAR );
        params._terrainStateSet->setTextureAttributeAndModes( *_textureUnit, local->_texGen.get(), 1 );

        osg::TexEnv* env = new osg::TexEnv();
        env->setMode( osg::TexEnv::DECAL );
        params._terrainStateSet->setTextureAttributeAndModes( *_textureUnit, env, 1 );
    }
}


void
DrapingTechnique::preCullTerrain(OverlayDecorator::TechRTTParams& params,
                                 osgUtil::CullVisitor*             cv )
{
    if ( !params._rttCamera.valid() && params._group->getNumChildren() > 0 && _textureUnit.isSet() )
    {
        setUpCamera( params );
    }

    if ( params._rttCamera.valid() )
    {
        LocalPerViewData& local = *static_cast<LocalPerViewData*>(params._techniqueData.get());
        if ( local._texGen.valid() )
        {
            // FFP path only
            cv->getCurrentRenderBin()->getStage()->addPositionedTextureAttribute(
                *_textureUnit, cv->getModelViewMatrix(), local._texGen.get() );
        }
    }
}


void
DrapingTechnique::cullOverlayGroup(OverlayDecorator::TechRTTParams& params,
                                   osgUtil::CullVisitor*            cv )
{
    if ( params._rttCamera.valid() )
    {
        static osg::Matrix s_scaleBiasMat = 
            osg::Matrix::translate(1.0,1.0,1.0) * 
            osg::Matrix::scale(0.5,0.5,0.5);

        params._rttCamera->setViewMatrix      ( params._rttViewMatrix );
        params._rttCamera->setProjectionMatrix( params._rttProjMatrix );

        osg::Matrix VPT = params._rttViewMatrix * params._rttProjMatrix * s_scaleBiasMat;

        LocalPerViewData& local = *static_cast<LocalPerViewData*>(params._techniqueData.get());

        if ( local._texGenUniform.valid() )
            local._texGenUniform->set( VPT );
        else
            local._texGen->setPlanesFromMatrix( VPT );

        // traverse the overlay group (via the RTT camera).
        params._rttCamera->accept( *cv );
    }
}


void
DrapingTechnique::setTextureSize( int texSize )
{
    if ( texSize != _textureSize.value() )
    {
        _textureSize = texSize;
    }
}

void
DrapingTechnique::setTextureUnit( int texUnit )
{
    if ( !_explicitTextureUnit.isSet() || texUnit != _explicitTextureUnit.value() )
    {
        _explicitTextureUnit = texUnit;
    }
}

void
DrapingTechnique::setMipMapping( bool value )
{
    if ( value != _mipmapping )
    {
        _mipmapping = value;

        if ( _mipmapping )
            OE_INFO << LC << "Overlay mipmapping " << (value?"enabled":"disabled") << std::endl;
    }
}

void
DrapingTechnique::setOverlayBlending( bool value )
{
    if ( value != _rttBlending )
    {
        _rttBlending = value;
        
        if ( _rttBlending )
            OE_INFO << LC << "Overlay blending " << (value?"enabled":"disabled")<< std::endl;
    }
}

bool
DrapingTechnique::getAttachStencil() const
{
    return _attachStencil;
}

void
DrapingTechnique::setAttachStencil( bool value )
{
    _attachStencil = value;
}

void
DrapingTechnique::onInstall( TerrainEngineNode* engine )
{
    // see whether we want shader support:
    // TODO: this is not stricty correct; you might still want to use shader overlays
    // in multipass mode, AND you might want FFP overlays in multitexture-FFP mode.
    _useShaders = 
        Registry::capabilities().supportsGLSL() && 
        engine->getTextureCompositor()->usesShaderComposition();

    if ( !_textureSize.isSet() )
    {
        unsigned maxSize = Registry::capabilities().getMaxFastTextureSize();
        _textureSize.init( osg::minimum( 4096u, maxSize ) );

        OE_INFO << LC << "Using texture size = " << *_textureSize << std::endl;
    }
}

void
DrapingTechnique::onUninstall( TerrainEngineNode* engine )
{
    if ( !_explicitTextureUnit.isSet() && _textureUnit.isSet() )
    {
        engine->getTextureCompositor()->releaseTextureImageUnit( *_textureUnit );
        _textureUnit.unset();
    }
}
