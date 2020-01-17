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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarth/Capabilities>
#include <osgEarth/Version>
#include <osg/FragmentProgram>
#include <osg/GL2Extensions>
#include <osg/Version>
#include <osgViewer/Version>
#include <gdal_priv.h>

using namespace osgEarth;

#define LC "[Capabilities] "

#ifndef GL_CONTEXT_PROFILE_MASK
#define GL_CONTEXT_PROFILE_MASK           0x9126
#endif
#ifndef GL_CONTEXT_CORE_PROFILE_BIT
#define GL_CONTEXT_CORE_PROFILE_BIT       0x00000001
#endif

// ---------------------------------------------------------------------------
// A custom P-Buffer graphics context that we will use to query for OpenGL 
// extension and hardware support. (Adapted from osgconv in OpenSceneGraph)

struct MyGraphicsContext
{
    MyGraphicsContext()
    {

    	osg::GraphicsContext::ScreenIdentifier si;
	    si.readDISPLAY();
	    si.setUndefinedScreenDetailsToDefaultScreen();

        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;  
    	traits->hostName = si.hostName;
	    traits->displayNum = si.displayNum;
	    traits->screenNum = si.screenNum;
        traits->x = 0;
        traits->y = 0;
        traits->width = 1;
        traits->height = 1;
        traits->windowDecoration = false;
        traits->doubleBuffer = false;
        traits->sharedContext = 0;
        traits->pbuffer = false;
        traits->glContextVersion = osg::DisplaySettings::instance()->getGLContextVersion();
        traits->glContextProfileMask = osg::DisplaySettings::instance()->getGLContextProfileMask();

        // Intel graphics adapters dont' support pbuffers, and some of their drivers crash when
        // you try to create them. So by default we will only use the unmapped/pbuffer method
        // upon special request.
        if ( getenv( "OSGEARTH_USE_PBUFFER_TEST" ) )
        {
            traits->pbuffer = true;
            OE_INFO << LC << "Activating pbuffer test for graphics capabilities" << std::endl;
            _gc = osg::GraphicsContext::createGraphicsContext(traits.get());
            if ( !_gc.valid() )
                OE_WARN << LC << "Failed to create pbuffer" << std::endl;
        }

        if (!_gc.valid())
        {
            // fall back on a mapped window
            traits->pbuffer = false;
            _gc = osg::GraphicsContext::createGraphicsContext(traits.get());
        }

        if (_gc.valid()) 
        {
            _gc->realize();
            _gc->makeCurrent();

            if ( traits->pbuffer == false )
            {
                OE_DEBUG << LC << "Realized graphics window for OpenGL operations." << std::endl;
            }
            else
            {
                OE_DEBUG << LC << "Realized pbuffer for OpenGL operations." << std::endl;
            }
        }
        else
        {
            OE_WARN << LC << "Failed to create graphic window too." << std::endl;
        }
    }

    bool valid() const { return _gc.valid() && _gc->isRealized(); }

    osg::ref_ptr<osg::GraphicsContext> _gc;
};

// ---------------------------------------------------------------------------

#define SAYBOOL(X) (X?"yes":"no")

Capabilities::Capabilities() :
_maxFFPTextureUnits     ( 1 ),
_maxGPUTextureUnits     ( 1 ),
_maxGPUTextureCoordSets ( 1 ),
_maxGPUAttribs          ( 1 ),
_maxTextureSize         ( 256 ),
_maxFastTextureSize     ( 256 ),
_maxLights              ( 1 ),
_depthBits              ( 0 ),
_supportsGLSL           ( false ),
_GLSLversion            ( 1.0f ),
_supportsTextureArrays  ( false ),
_supportsMultiTexture   ( false ),
_supportsStencilWrap    ( true ),
_supportsTwoSidedStencil( false ),
_supportsTexture3D      ( false ),
_supportsTexture2DLod   ( false ),
_supportsMipmappedTextureUpdates( false ),
_supportsDepthPackedStencilBuffer( false ),
_supportsOcclusionQuery ( false ),
_supportsDrawInstanced  ( false ),
_supportsUniformBufferObjects( false ),
_supportsNonPowerOfTwoTextures( false ),
_maxUniformBlockSize    ( 0 ),
_preferDLforStaticGeom  ( true ),
_numProcessors          ( 1 ),
_supportsFragDepthWrite ( false ),
_supportsS3TC           ( false ),
_supportsPVRTC          ( false ),
_supportsARBTC          ( false ),
_supportsETC            ( false ),
_supportsRGTC           ( false ),
_supportsTextureBuffer  ( false ),
_maxTextureBufferSize   ( 0 ),
_isCoreProfile          ( true )
{
    // little hack to force the osgViewer library to link so we can create a graphics context
    osgViewerGetVersion();

    // check the environment in order to disable ATI workarounds
    bool enableATIworkarounds = true;
    if ( ::getenv( "OSGEARTH_DISABLE_ATI_WORKAROUNDS" ) != 0L )
        enableATIworkarounds = false;

    // logical CPUs (cores)
    _numProcessors = OpenThreads::GetNumberOfProcessors();

    // GLES compile?
#if (defined(OSG_GLES1_AVAILABLE) || defined(OSG_GLES2_AVAILABLE) || defined(OSG_GLES3_AVAILABLE))
    _isGLES = true;
#else
    _isGLES = false;
#endif

    // create a graphics context so we can query OpenGL support:
    osg::GraphicsContext* gc = NULL;
    unsigned int id = 0;
#ifndef __ANDROID__
    MyGraphicsContext mgc;
    if ( mgc.valid() )
    {
        gc = mgc._gc.get();
        id = gc->getState()->getContextID();
    }
#endif

#ifndef __ANDROID__
    if ( gc != NULL )
#endif
    {
        const osg::GL2Extensions* GL2 = osg::GL2Extensions::Get( id, true );

        OE_INFO << LC << "osgEarth Version: " << osgEarthGetVersion() << std::endl;

        OE_INFO << LC << "OSG Version:      " << osgGetVersion() << std::endl;

#ifdef GDAL_RELEASE_NAME
        OE_INFO << LC << "GDAL Version:     " << GDAL_RELEASE_NAME << std::endl;
#endif
        
        if ( ::getenv("OSGEARTH_NO_GLSL") )
        {
            _supportsGLSL = false;
            OE_INFO << LC << "Note: GLSL expressly disabled (OSGEARTH_NO_GLSL)" << std::endl;
        }
        else
        {
            _supportsGLSL = GL2->isGlslSupported;
        }

        OE_INFO << LC << "Detected hardware capabilities:" << std::endl;

        _vendor = std::string( reinterpret_cast<const char*>(glGetString(GL_VENDOR)) );
        OE_INFO << LC << "  Vendor = " << _vendor << std::endl;

        _renderer = std::string( reinterpret_cast<const char*>(glGetString(GL_RENDERER)) );
        OE_INFO << LC << "  Renderer = " << _renderer << std::endl;

        _version = std::string( reinterpret_cast<const char*>(glGetString(GL_VERSION)) );
        OE_INFO << LC << "  Version = " << _version << std::endl;

        // Detect core profile by investigating GL_CONTEXT_PROFILE_MASK
        if ( GL2->glVersion < 3.2f )
        {
            _isCoreProfile = false;
        }
        else
        {
            GLint profileMask = 0;
            glGetIntegerv(GL_CONTEXT_PROFILE_MASK, &profileMask);
            _isCoreProfile = ((profileMask & GL_CONTEXT_CORE_PROFILE_BIT) != 0);
        }
        OE_INFO << LC << "  Core Profile = " << SAYBOOL(_isCoreProfile) << std::endl;

#if !defined(OSG_GLES2_AVAILABLE) && !defined(OSG_GLES3_AVAILABLE)
        glGetIntegerv( GL_MAX_TEXTURE_UNITS, &_maxFFPTextureUnits );
        //OE_INFO << LC << "  Max FFP texture units = " << _maxFFPTextureUnits << std::endl;
#endif

        glGetIntegerv( GL_MAX_TEXTURE_IMAGE_UNITS_ARB, &_maxGPUTextureUnits );
        OE_INFO << LC << "  Max GPU texture units = " << _maxGPUTextureUnits << std::endl;

#if !defined(OSG_GLES2_AVAILABLE) && !defined(OSG_GLES3_AVAILABLE)
        glGetIntegerv( GL_MAX_TEXTURE_COORDS_ARB, &_maxGPUTextureCoordSets );
#else
        _maxGPUTextureCoordSets = _maxGPUTextureUnits;
#endif
        OE_INFO << LC << "  Max GPU texture coord indices = " << _maxGPUTextureCoordSets << std::endl;

        glGetIntegerv( GL_MAX_VERTEX_ATTRIBS, &_maxGPUAttribs );
        OE_INFO << LC << "  Max GPU attributes = " << _maxGPUAttribs << std::endl;

#if !(defined(OSG_GL3_AVAILABLE))
        glGetIntegerv( GL_DEPTH_BITS, &_depthBits );
        OE_INFO << LC << "  Depth buffer bits = " << _depthBits << std::endl;
#endif
        
        glGetIntegerv( GL_MAX_TEXTURE_SIZE, &_maxTextureSize );
#if !defined(OSG_GLES1_AVAILABLE) && !defined(OSG_GLES2_AVAILABLE) && !defined(OSG_GLES3_AVAILABLE)
        // Use the texture-proxy method to determine the maximum texture size 
        for( int s = _maxTextureSize; s > 2; s >>= 1 )
        {
            glTexImage2D( GL_PROXY_TEXTURE_2D, 0, GL_RGBA8, s, s, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0L );
            GLint width = 0;
            glGetTexLevelParameteriv( GL_PROXY_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width );
            if ( width == s )
            {
                _maxTextureSize = s;
                break;
            }
        }
#endif
        OE_INFO << LC << "  Max texture size = " << _maxTextureSize << std::endl;

#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
        glGetIntegerv( GL_MAX_LIGHTS, &_maxLights );
#else
        _maxLights = 1;
#endif
        OE_INFO << LC << "  GLSL = " << SAYBOOL(_supportsGLSL) << std::endl;

        if ( _supportsGLSL )
        {
			_GLSLversion = GL2->glslLanguageVersion;
            OE_INFO << LC << "  GLSL Version = " << getGLSLVersionInt() << std::endl;
        }

        _supportsTextureArrays = 
            _supportsGLSL &&
            osg::getGLVersionNumber() >= 2.0f && // hopefully this will detect Intel cards
            osg::isGLExtensionSupported( id, "GL_EXT_texture_array" );
        OE_INFO << LC << "  Texture arrays = " << SAYBOOL(_supportsTextureArrays) << std::endl;

        _supportsMultiTexture = 
            osg::getGLVersionNumber() >= 1.3f ||
            osg::isGLExtensionSupported( id, "GL_ARB_multitexture") ||
            osg::isGLExtensionSupported( id, "GL_EXT_multitexture" );
        //OE_INFO << LC << "  Multitexturing = " << SAYBOOL(_supportsMultiTexture) << std::endl;

        _supportsStencilWrap = osg::isGLExtensionSupported( id, "GL_EXT_stencil_wrap" );
        //OE_INFO << LC << "  Stencil wrapping = " << SAYBOOL(_supportsStencilWrap) << std::endl;

        _supportsTwoSidedStencil = osg::isGLExtensionSupported( id, "GL_EXT_stencil_two_side" );
        //OE_INFO << LC << "  2-sided stencils = " << SAYBOOL(_supportsTwoSidedStencil) << std::endl;

        _supportsDepthPackedStencilBuffer = osg::isGLExtensionSupported( id, "GL_EXT_packed_depth_stencil" ) || 
                                            osg::isGLExtensionSupported( id, "GL_OES_packed_depth_stencil" );
        //OE_INFO << LC << "  depth-packed stencil = " << SAYBOOL(_supportsDepthPackedStencilBuffer) << std::endl;

        _supportsOcclusionQuery = osg::isGLExtensionSupported( id, "GL_ARB_occlusion_query" );
        //OE_INFO << LC << "  occlusion query = " << SAYBOOL(_supportsOcclusionQuery) << std::endl;

        _supportsDrawInstanced = 
            _supportsGLSL &&
            osg::isGLExtensionOrVersionSupported( id, "GL_EXT_draw_instanced", 3.1f );
        OE_INFO << LC << "  draw instanced = " << SAYBOOL(_supportsDrawInstanced) << std::endl;

        glGetIntegerv( GL_MAX_UNIFORM_BLOCK_SIZE, &_maxUniformBlockSize );
        //OE_INFO << LC << "  max uniform block size = " << _maxUniformBlockSize << std::endl;

        _supportsUniformBufferObjects = 
            _supportsGLSL &&
            osg::isGLExtensionOrVersionSupported( id, "GL_ARB_uniform_buffer_object", 2.0f );
        //OE_INFO << LC << "  uniform buffer objects = " << SAYBOOL(_supportsUniformBufferObjects) << std::endl;

        if ( _supportsUniformBufferObjects && _maxUniformBlockSize == 0 )
        {
            OE_INFO << LC << "  ...but disabled, since UBO block size reports zero" << std::endl;
            _supportsUniformBufferObjects = false;
        }

#if !defined(OSG_GLES3_AVAILABLE)
        _supportsNonPowerOfTwoTextures =
            osg::isGLExtensionSupported( id, "GL_ARB_texture_non_power_of_two" );
#else
        _supportsNonPowerOfTwoTextures = true;
#endif
        //OE_INFO << LC << "  NPOT textures = " << SAYBOOL(_supportsNonPowerOfTwoTextures) << std::endl;


#if !defined(OSG_GLES3_AVAILABLE)
        _supportsTextureBuffer = 
            osg::isGLExtensionOrVersionSupported( id, "GL_ARB_texture_buffer_object", 3.0 ) ||
            osg::isGLExtensionOrVersionSupported( id, "GL_EXT_texture_buffer_object", 3.0 );
#else
        _supportsTextureBuffer = false;
#endif

        if ( _supportsTextureBuffer )
        {
            glGetIntegerv( GL_MAX_TEXTURE_BUFFER_SIZE, &_maxTextureBufferSize );
        }

        OE_INFO << LC << "  Texture buffers = " << SAYBOOL(_supportsTextureBuffer) << std::endl;
        if ( _supportsTextureBuffer )
        {
            OE_INFO << LC << "  Texture buffer max size = " << _maxTextureBufferSize << std::endl;
        }       

        bool supportsTransformFeedback =
            osg::isGLExtensionSupported( id, "GL_ARB_transform_feedback2" );
        //OE_INFO << LC << "  Transform feedback = " << SAYBOOL(supportsTransformFeedback) << "\n";


        // Writing to gl_FragDepth is not supported under GLES, is supported under gles3
#if (defined(OSG_GLES1_AVAILABLE) || defined(OSG_GLES2_AVAILABLE))
        _supportsFragDepthWrite = false;
#else
        _supportsFragDepthWrite = true;
#endif

        // NVIDIA:
        bool isNVIDIA = _vendor.find("NVIDIA") == 0;

        // NVIDIA has h/w acceleration of some kind for display lists, supposedly.
        // In any case they do benchmark much faster in osgEarth for static geom.
        // BUT unfortunately, they dont' seem to work too well with shaders. Colors
        // change randomly, etc. Might work OK for textured geometry but not for 
        // untextured. TODO: investigate.
        _preferDLforStaticGeom = false;
        if ( ::getenv("OSGEARTH_TRY_DISPLAY_LISTS") )
        {
            _preferDLforStaticGeom = true;
        }

        //OE_INFO << LC << "  prefer DL for static geom = " << SAYBOOL(_preferDLforStaticGeom) << std::endl;

        // ATI workarounds:
        bool isATI = _vendor.find("ATI ") == 0;

        _supportsMipmappedTextureUpdates = isATI && enableATIworkarounds ? false : true;

        _maxFastTextureSize = _maxTextureSize;

        //OE_INFO << LC << "  Max Fast Texture Size = " << _maxFastTextureSize << std::endl;

        // tetxure compression
        OE_INFO << LC << "  Compression = ";
        _supportsARBTC = osg::isGLExtensionSupported( id, "GL_ARB_texture_compression" );
        if (_supportsARBTC) OE_INFO_CONTINUE << "ARB ";

        _supportsS3TC = osg::isGLExtensionSupported( id, "GL_EXT_texture_compression_s3tc" );
        if ( _supportsS3TC ) OE_INFO_CONTINUE << "S3 ";

        _supportsPVRTC = osg::isGLExtensionSupported( id, "GL_IMG_texture_compression_pvrtc" );
        if ( _supportsPVRTC ) OE_INFO_CONTINUE << "PVR ";

        _supportsETC = osg::isGLExtensionSupported( id, "GL_OES_compressed_ETC1_RGB8_texture" );
        if ( _supportsETC ) OE_INFO_CONTINUE << "ETC1 ";

        _supportsRGTC = osg::isGLExtensionSupported( id, "GL_EXT_texture_compression_rgtc" );
        if ( _supportsRGTC ) OE_INFO_CONTINUE << "RG";

        OE_INFO_CONTINUE << std::endl;
    }
}

bool
Capabilities::supportsTextureCompression(const osg::Texture::InternalFormatMode& mode) const
{
    switch( mode )
    {
    case osg::Texture::USE_ARB_COMPRESSION:
        return _supportsARBTC;
        break;

    case osg::Texture::USE_S3TC_DXT1a_COMPRESSION:
    case osg::Texture::USE_S3TC_DXT1c_COMPRESSION:
    case osg::Texture::USE_S3TC_DXT1_COMPRESSION:
    case osg::Texture::USE_S3TC_DXT3_COMPRESSION:
    case osg::Texture::USE_S3TC_DXT5_COMPRESSION:
        return _supportsS3TC;
        break;

    case osg::Texture::USE_PVRTC_2BPP_COMPRESSION:
    case osg::Texture::USE_PVRTC_4BPP_COMPRESSION:
        return _supportsPVRTC;
        break;

    case osg::Texture::USE_ETC_COMPRESSION:
        return _supportsETC;
        break;

    case osg::Texture::USE_RGTC1_COMPRESSION:
    case osg::Texture::USE_RGTC2_COMPRESSION:
        return _supportsRGTC;
        break;

    default:
        return false;
    }

    return false;
}
