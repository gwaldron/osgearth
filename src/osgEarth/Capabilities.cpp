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
#include <osg/GraphicsContext>
#include <osg/GL>
#include <osg/GLExtensions>
#include <osg/GL2Extensions>
#include <osg/Texture>

using namespace osgEarth;

#define LC "[Capabilities] "

// ---------------------------------------------------------------------------
// A custom P-Buffer graphics context that we will use to query for OpenGL 
// extension and hardware support. (Adapted from osgconv in OpenSceneGraph)

struct MyGraphicsContext
{
    MyGraphicsContext()
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

        // Intel graphics adapters dont' support pbuffers, and some of their drivers crash when
        // you try to create them. So by default we will only use the unmapped/pbuffer method
        // upon special request.
        if ( getenv( "OSGEARTH_USE_PBUFFER_TEST" ) )
        {
            traits->pbuffer = true;
            OE_INFO << LC << "Activating pbuffer test for graphics capabilities" << std::endl;
            _gc = osg::GraphicsContext::createGraphicsContext(traits.get());
        }

        if (!_gc)
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
            OE_INFO << LC << "Failed to create graphic window too." << std::endl;
        }
    }

    bool valid() const { return _gc.valid() && _gc->isRealized(); }

    osg::ref_ptr<osg::GraphicsContext> _gc;
};

// ---------------------------------------------------------------------------

#define SAYBOOL(X) (X?"yes":"no")

Capabilities::Capabilities() :
_maxFFPTextureUnits     ( 1 ),
_maxTextureSize         ( 256 ),
_supportsGLSL           ( false ),
_supportsTextureArrays  ( false ),
_supportsMultiTexture   ( false ),
_supportsStencilWrap    ( true ),
_supportsTwoSidedStencil( false )
{
    // create a graphics context so we can query OpenGL support:
    MyGraphicsContext mgc;

    if ( mgc.valid() )
    {
        osg::GraphicsContext* gc = mgc._gc.get();
        unsigned int id = gc->getState()->getContextID();
        const osg::GL2Extensions* GL2 = osg::GL2Extensions::Get( id, true );

        OE_INFO << LC << "Detected hardware capabilities:" << std::endl;


        glGetIntegerv( GL_MAX_TEXTURE_UNITS, &_maxFFPTextureUnits );
        OE_INFO << LC << "  Max FFP texture units = " << _maxFFPTextureUnits << std::endl;

        // Use the texture-proxy method to determine the maximum texture size 
        for( int s = 16; s < 65536; s *= 2 )
        {
            glTexImage2D( GL_PROXY_TEXTURE_2D, 0, GL_RGBA8, s, s, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0L );
            GLint width = 0;
            glGetTexLevelParameteriv( GL_PROXY_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width );
            if ( width == 0 )
            {
                _maxTextureSize = s/2;
                break;
            }
        }
        OE_INFO << LC << "  Max texture size = " << _maxTextureSize << std::endl;

        _supportsGLSL = GL2->isGlslSupported();
        OE_INFO << LC << "  Supports GLSL = " << SAYBOOL(_supportsGLSL) << std::endl;

        _supportsTextureArrays = osg::isGLExtensionSupported( id, "GL_EXT_texture_array" );
        OE_INFO << LC << "  Supports texture arrays = " << SAYBOOL(_supportsTextureArrays) << std::endl;

        _supportsMultiTexture = 
            osg::getGLVersionNumber() >= 1.3 ||
            osg::isGLExtensionSupported( id, "GL_ARB_multitexture") ||
            osg::isGLExtensionSupported( id, "GL_EXT_multitexture" );
        OE_INFO << LC << "  Supports multitexturing = " << SAYBOOL(_supportsMultiTexture) << std::endl;

        _supportsStencilWrap = osg::isGLExtensionSupported( id, "GL_EXT_stencil_wrap" );
        OE_INFO << LC << "  Supports stencil wrapping = " << SAYBOOL(_supportsStencilWrap) << std::endl;

        _supportsTwoSidedStencil = osg::isGLExtensionSupported( id, "GL_EXT_stencil_two_side" );
        OE_INFO << LC << "  Supports 2-sided stencils = " << SAYBOOL(_supportsTwoSidedStencil) << std::endl;
    }
}

