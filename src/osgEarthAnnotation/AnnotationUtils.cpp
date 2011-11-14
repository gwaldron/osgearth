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
#include <osgText/Text>

using namespace osgEarth::Annotation;


osg::Drawable* 
AnnotationUtils::createTextDrawable(const std::string& text,
                                    const TextSymbol*  symbol,
                                    const osg::Vec3&   positionOffset )
                                    
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
    t->getOrCreateStateSet()->setRenderBinToInherit();

    return t;
}

osg::Geometry*
AnnotationUtils::createImageGeometry(osg::Image*       image,
                                     const osg::Vec2s& pixelOffset)
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
    geom->setTexCoordArray(0,tcoords);

    osg::Vec4Array* colors = new osg::Vec4Array(1);
    (*colors)[0].set(1.0f,1.0f,1.0,1.0f);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));

    return geom;
}
