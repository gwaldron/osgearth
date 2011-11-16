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

#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthSymbology/Color>
#include <osgEarth/ThreadingUtils>
#include <osgText/Text>

using namespace osgEarth;
using namespace osgEarth::Annotation;


osg::Drawable* 
AnnotationUtils::createTextDrawable(const std::string& text,
                                    const TextSymbol*  symbol,
                                    const osg::Vec3&   positionOffset,
                                    bool               installFadeShader)
                                    
{
    osgText::Text* t = new osgText::Text();
    osgText::String::Encoding text_encoding = osgText::String::ENCODING_UNDEFINED;
    if ( symbol && symbol->encoding().isSet() )
    {
        switch(symbol->encoding().value())
        {
        case TextSymbol::ENCODING_ASCII: text_encoding = osgText::String::ENCODING_ASCII; break;
        case TextSymbol::ENCODING_UTF8: text_encoding = osgText::String::ENCODING_UTF8; break;
        case TextSymbol::ENCODING_UTF16: text_encoding = osgText::String::ENCODING_UTF16; break;
        case TextSymbol::ENCODING_UTF32: text_encoding = osgText::String::ENCODING_UTF32; break;
        default: text_encoding = osgText::String::ENCODING_UNDEFINED; break;
        }
    }

    t->setText( text, text_encoding );

    if ( symbol && symbol->pixelOffset().isSet() )
    {
        t->setPosition( osg::Vec3(
            positionOffset.x() + symbol->pixelOffset()->x(),
            positionOffset.y() + symbol->pixelOffset()->y(),
            positionOffset.z() ) );
    }
    else
    {
        t->setPosition( positionOffset );
    }

    t->setAutoRotateToScreen( false );
    t->setCharacterSizeMode( osgText::Text::OBJECT_COORDS );
    t->setCharacterSize( symbol && symbol->size().isSet() ? *symbol->size() : 16.0f );
    t->setFont( osgText::readFontFile( symbol && symbol->font().isSet() ? *symbol->font() : "arial.ttf" ) );
    t->setColor( symbol && symbol->fill().isSet() ? symbol->fill()->color() : Color::White );

    if ( symbol )
    {
        // they're the same enum.
        osgText::Text::AlignmentType at = (osgText::Text::AlignmentType)symbol->alignment().value();
        t->setAlignment( at );
    }

    if ( symbol && symbol->halo().isSet() )
    {
        t->setBackdropColor( symbol->halo()->color() );
        t->setBackdropType( osgText::Text::OUTLINE );
    }

    // this disables the default rendering bin set by osgText::Font. Necessary if we're
    // going to do decluttering at a higher level
    osg::StateSet* stateSet = t->getOrCreateStateSet();

    stateSet->setRenderBinToInherit();
    
    if ( installFadeShader )
    {
        // a custom fragment shader for drawing faded, textured geometry:
        // btw, the "1.5" makes the text a little brighter and nicer-looking :)
        static char s_frag[] =
            "uniform float     fade; \n"
            "uniform sampler2D tex0; \n"
            "void main() { \n"
            //"    if ( fade < 1.0 ) { fade = 0.0; } \n"
            "    gl_FragColor = gl_Color * texture2D(tex0,gl_TexCoord[0].st).aaaa * vec4(1,1,1,fade*1.5); \n"
            "} \n";

        static osg::ref_ptr<osg::Program> s_program;
        static Threading::Mutex s_programMutex;
        if ( s_program == 0L )
        {
            Threading::ScopedMutexLock lock(s_programMutex);
            if ( s_program == 0L )
            {
                s_program = new osg::Program();
                s_program->addShader( new osg::Shader(osg::Shader::FRAGMENT, s_frag) );
            }
        }

        stateSet->setAttributeAndModes( s_program.get(), 1 );
    }

    return t;
}

osg::Geometry*
AnnotationUtils::createImageGeometry(osg::Image*       image,
                                     const osg::Vec2s& pixelOffset,
                                     bool              installFadeShader,
                                     unsigned          textureUnit )
{
    if ( !image )
        return 0L;

    osg::Texture2D* texture = new osg::Texture2D();
    texture->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR);
    texture->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR);
    texture->setResizeNonPowerOfTwoHint(false);
    texture->setImage( image );

    // set up the drawstate.
    osg::StateSet* dstate = new osg::StateSet;
    dstate->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);
    dstate->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    dstate->setMode(GL_BLEND, 1);
    dstate->setTextureAttributeAndModes(0, texture,osg::StateAttribute::ON);   

    // set up the geoset.
    osg::Geometry* geom = new osg::Geometry();
    geom->setStateSet(dstate);

    float x0 = (float)pixelOffset.x() - image->s()/2.0;
    float y0 = (float)pixelOffset.y() - image->t()/2.0;

    osg::Vec3Array* coords = new osg::Vec3Array(4);
    (*coords)[0].set( x0, y0, 0 );
    (*coords)[1].set( x0 + image->s(), y0, 0 );
    (*coords)[2].set( x0 + image->s(), y0 + image->t(), 0 );
    (*coords)[3].set( x0, y0 + image->t(), 0 );
    geom->setVertexArray(coords);

    osg::Vec2Array* tcoords = new osg::Vec2Array(4);
    (*tcoords)[0].set(0, 0);
    (*tcoords)[1].set(1, 0);
    (*tcoords)[2].set(1, 1);
    (*tcoords)[3].set(0, 1);
    geom->setTexCoordArray(textureUnit,tcoords);

    osg::Vec4Array* colors = new osg::Vec4Array(1);
    (*colors)[0].set(1.0f,1.0f,1.0,1.0f);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));
    
    if ( installFadeShader )
    {
        // a custom fragment shader for drawing faded, textured geometry:
        static char s_frag[] =
            "uniform float     fade; \n"
            "uniform sampler2D tex0; \n"
            "void main() { \n"
            "    gl_FragColor = gl_Color * texture2D(tex0,gl_TexCoord[0].st) * vec4(1,1,1,fade); \n"
            "} \n";

        static osg::ref_ptr<osg::Program> s_program;
        static Threading::Mutex s_programMutex;
        if ( s_program == 0L )
        {
            Threading::ScopedMutexLock lock(s_programMutex);
            if ( s_program == 0L )
            {
                s_program = new osg::Program();
                s_program->addShader( new osg::Shader(osg::Shader::FRAGMENT, s_frag) );
            }
        }

        dstate->setAttributeAndModes( s_program.get(), 1 );
    }

    return geom;
}

osg::Uniform*
AnnotationUtils::createFadeUniform()
{
    osg::Uniform* u = new osg::Uniform(osg::Uniform::FLOAT, "fade");
    u->set( 1.0f );
    return u;
}
