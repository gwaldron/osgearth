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

#include <osgEarthAnnotation/LabelNode>
#include <osgEarthAnnotation/Decluttering>
#include <osgEarthSymbology/Color>
#include <osgText/Text>
#include <osg/Depth>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Symbology;


osg::Drawable* 
LabelUtils::createText(const osg::Vec3&   positionOffset,
                       const std::string& text,
                       const TextSymbol*  symbol )
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

//-------------------------------------------------------------------

LabelNode::LabelNode(MapNode*            mapNode,
                     const osg::Vec3d&   position,
                     const std::string&  text,
                     const TextSymbol*   symbol ) :

OrthoNode( mapNode->getMap()->getProfile()->getSRS(), position ),
_text( text )
{
    init( symbol );
}

LabelNode::LabelNode(const SpatialReference* mapSRS,
                     const osg::Vec3d&       position,
                     const std::string&      text,
                     const TextSymbol*       symbol ) :

//LocalizedNode( mapSRS, position, false ), //true ),
OrthoNode( mapSRS, position ),
_text( text )
{
    init( symbol );
}

void
LabelNode::init( const TextSymbol* symbol )
{
    // The following setup will result is a proper dynamic bounding box for the text.
    // If you just use osgText's rotate-to-screen and SCREEN_COORDS setup, you do not
    // get a proper bounds.
    osg::Drawable* t = LabelUtils::createText( osg::Vec3(0,0,0), _text, symbol );

    osg::StateSet* stateSet = t->getOrCreateStateSet();
    stateSet->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), 1 );

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( t );

    this->attach( geode );
}
