/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osg/FragmentProgram>
#include <osg/GraphicsContext>
#include <osg/GL>
#include <osg/GLExtensions>
#include <osg/GL2Extensions>
#include <osg/Texture>
#include <osgViewer/Version>

using namespace osgEarth;

#define LC "[Capabilities] "

// ---------------------------------------------------------------------------
// A custom P-Buffer graphics context that we will use to query for OpenGL 
// extension and hardware support. (Adapted from osgconv in OpenSceneGraph)

struct MyGraphicsContext
{
    MyGraphicsContext(bool quadBufferStereo)
    {
        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
        traits->x = 0;
        traits->y = 0;
        traits->width = 1;
        traits->height = 1;
        traits->windowDecoration = false;
        traits->doubleBuffer = false;
        traits->sharedContext = 0;
        traits->pbuffer = false;
		traits->quadBufferStereo = quadBufferStereo;

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
	osg::GraphicsContext * gc() { return _gc.get(); }
private:
    osg::ref_ptr<osg::GraphicsContext> _gc;
};

// ---------------------------------------------------------------------------

#define SAYBOOL(X) (X?"yes":"no")

Capabilities::Capabilities() :
_maxFFPTextureUnits     ( 1 ),
_maxGPUTextureUnits     ( 1 ),
_maxGPUTextureCoordSets ( 1 ),
_maxTextureSize         ( 256 ),
_maxLights              ( 1 ),
_supportsGLSL           ( false ),
_GLSLversion            ( 1.0f ),
_supportsTextureArrays  ( false ),
_supportsMultiTexture   ( false ),
_supportsStencilWrap    ( true ),
_supportsTwoSidedStencil( false ),
_supportsTexture2DLod   ( false ),
_supportsMipmappedTextureUpdates( false ),
_supportsQuadBufferStereo( false )
{
    // little hack to force the osgViewer library to link so we can create a graphics context
    osgViewerGetVersion();

    // check the environment in order to disable ATI workarounds
    bool enableATIworkarounds = true;
    if ( ::getenv( "OSGEARTH_DISABLE_ATI_WORKAROUNDS" ) != 0L )
        enableATIworkarounds = false;

	// first create a opengl context with quad buffer stereo enabled
	MyGraphicsContext * mgc = new MyGraphicsContext(true);
	_supportsQuadBufferStereo = mgc->valid();
	if(!_supportsQuadBufferStereo)
	{
		// delete the old context
		delete mgc;
		// second try to create a new graphics context without quad buffer stereo
		mgc = new MyGraphicsContext(false);
	}

    if ( mgc->valid() )
    {
        osg::GraphicsContext* gc = mgc->gc();
        unsigned int id = gc->getState()->getContextID();
        const osg::GL2Extensions* GL2 = osg::GL2Extensions::Get( id, true );

        OE_INFO << LC << "Detected hardware capabilities:" << std::endl;

        _vendor = std::string( reinterpret_cast<const char*>(glGetString(GL_VENDOR)) );
        OE_INFO << LC << "  Vendor = " << _vendor << std::endl;

        _renderer = std::string( reinterpret_cast<const char*>(glGetString(GL_RENDERER)) );
        OE_INFO << LC << "  Renderer = " << _renderer << std::endl;

        _version = std::string( reinterpret_cast<const char*>(glGetString(GL_VERSION)) );
        OE_INFO << LC << "  Version = " << _version << std::endl;

        glGetIntegerv( GL_MAX_TEXTURE_UNITS, &_maxFFPTextureUnits );
        OE_INFO << LC << "  Max FFP texture units = " << _maxFFPTextureUnits << std::endl;

        glGetIntegerv( GL_MAX_TEXTURE_IMAGE_UNITS_ARB, &_maxGPUTextureUnits );
        OE_INFO << LC << "  Max GPU texture units = " << _maxGPUTextureUnits << std::endl;

        glGetIntegerv( GL_MAX_TEXTURE_COORDS_ARB, &_maxGPUTextureCoordSets );
        OE_INFO << LC << "  Max GPU texture coordinate sets = " << _maxGPUTextureCoordSets << std::endl;

        // Use the texture-proxy method to determine the maximum texture size 
        glGetIntegerv( GL_MAX_TEXTURE_SIZE, &_maxTextureSize );
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
        OE_INFO << LC << "  Max texture size = " << _maxTextureSize << std::endl;

        glGetIntegerv( GL_MAX_LIGHTS, &_maxLights );
        OE_INFO << LC << "  Max lights = " << _maxLights << std::endl;

        _supportsGLSL = GL2->isGlslSupported();
        OE_INFO << LC << "  GLSL = " << SAYBOOL(_supportsGLSL) << std::endl;

        if ( _supportsGLSL )
        {
            _GLSLversion = GL2->getLanguageVersion();
            OE_INFO << LC << "  GLSL Version = " << _GLSLversion << std::endl;
        }

        _supportsTextureArrays = 
            _supportsGLSL &&
            osg::getGLVersionNumber() >= 2.0 && // hopefully this will detect Intel cards
            osg::isGLExtensionSupported( id, "GL_EXT_texture_array" );
        OE_INFO << LC << "  Texture arrays = " << SAYBOOL(_supportsTextureArrays) << std::endl;

        _supportsTexture3D = osg::isGLExtensionSupported( id, "GL_EXT_texture3D" );
        OE_INFO << LC << "  3D textures = " << SAYBOOL(_supportsTexture3D) << std::endl;

        _supportsMultiTexture = 
            osg::getGLVersionNumber() >= 1.3 ||
            osg::isGLExtensionSupported( id, "GL_ARB_multitexture") ||
            osg::isGLExtensionSupported( id, "GL_EXT_multitexture" );
        OE_INFO << LC << "  Multitexturing = " << SAYBOOL(_supportsMultiTexture) << std::endl;

        _supportsStencilWrap = osg::isGLExtensionSupported( id, "GL_EXT_stencil_wrap" );
        OE_INFO << LC << "  Stencil wrapping = " << SAYBOOL(_supportsStencilWrap) << std::endl;

        _supportsTwoSidedStencil = osg::isGLExtensionSupported( id, "GL_EXT_stencil_two_side" );
        OE_INFO << LC << "  2-sided stencils = " << SAYBOOL(_supportsTwoSidedStencil) << std::endl;

		OE_INFO << LC << "  Supports quad buffer stereo = " << SAYBOOL(_supportsQuadBufferStereo) << std::endl;

        //_supportsTexture2DLod = osg::isGLExtensionSupported( id, "GL_ARB_shader_texture_lod" );
        //OE_INFO << LC << "  texture2DLod = " << SAYBOOL(_supportsTexture2DLod) << std::endl;

        bool isATI = _vendor.length() >= 4 && ::strncmp(_vendor.c_str(), "ATI ", 4) == 0;

        _supportsMipmappedTextureUpdates = isATI && enableATIworkarounds ? false : true;
    }

    // delete the final object of the graphics context
	delete mgc;
}
