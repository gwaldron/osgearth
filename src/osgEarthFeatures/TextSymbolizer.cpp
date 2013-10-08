/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarthFeatures/TextSymbolizer>
#include <osgEarth/Registry>
#include <osgText/Text>


using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

TextSymbolizer::TextSymbolizer(const TextSymbol* symbol) :
_symbol( symbol )
{
    //nop
}

osgText::Text*
TextSymbolizer::create(Feature*             feature,
                       const FilterContext* context,
                       const std::string&   text     ) const
{    
    osgText::Text* t = new osgText::Text();

    osgText::String::Encoding textEncoding = osgText::String::ENCODING_UNDEFINED;

    if ( _symbol.valid() && _symbol->encoding().isSet() )
    {
        switch(_symbol->encoding().value())
        {
        case TextSymbol::ENCODING_ASCII: textEncoding = osgText::String::ENCODING_ASCII; break;
        case TextSymbol::ENCODING_UTF8: textEncoding = osgText::String::ENCODING_UTF8; break;
        case TextSymbol::ENCODING_UTF16: textEncoding = osgText::String::ENCODING_UTF16; break;
        case TextSymbol::ENCODING_UTF32: textEncoding = osgText::String::ENCODING_UTF32; break;
        default: textEncoding = osgText::String::ENCODING_UNDEFINED; break;
        }
    }

    if ( !text.empty() )
    {
        t->setText( text, textEncoding );
    }
    else if ( _symbol.valid() && _symbol->content().isSet() )
    {
        StringExpression expr = *_symbol->content();
        std::string newText = feature ? feature->eval(expr, context) : expr.eval();
        t->setText( newText, textEncoding );
    }

    if ( _symbol.valid() && _symbol->pixelOffset().isSet() )
    {
        t->setPosition( osg::Vec3(_symbol->pixelOffset()->x(), _symbol->pixelOffset()->y(), 0.0f) );
    }

    //TODO: resonsider defaults here
    t->setCharacterSizeMode( osgText::Text::OBJECT_COORDS );

    t->setCharacterSize( _symbol.valid() && _symbol->size().isSet() ? *_symbol->size() : 16.0f );

    t->setColor( _symbol.valid() && _symbol->fill().isSet() ? _symbol->fill()->color() : Color::White );

    osgText::Font* font = 0L;
    if ( _symbol.valid() && _symbol->font().isSet() )
        font = osgText::readFontFile( *_symbol->font() );
    if ( !font )
        font = Registry::instance()->getDefaultFont();
    if ( font )
        t->setFont( font );

    if ( _symbol.valid() )
    {
        // they're the same enum.
        osgText::Text::AlignmentType at = (osgText::Text::AlignmentType)_symbol->alignment().value();
        t->setAlignment( at );
    }

    if ( _symbol.valid() && _symbol->halo().isSet() )
    {
        t->setBackdropColor( _symbol->halo()->color() );
        t->setBackdropType( osgText::Text::OUTLINE );
    }
    else if ( !_symbol.valid() )
    {
        // if no symbol at all is provided, default to using a black halo.
        t->setBackdropColor( osg::Vec4(.3,.3,.3,1) );
        t->setBackdropType( osgText::Text::OUTLINE );
    }

    return t;
}

