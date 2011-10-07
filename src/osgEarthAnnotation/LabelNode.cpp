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
#include <osgText/Text>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Symbology;


osg::Drawable* 
LabelUtils::createText(const osg::Vec3&   positionOffset,
                       const std::string& text,
                       const TextSymbol*  symbol )
{
    osgText::Text* t = new osgText::Text();
    t->setText( text );
    t->setPosition( positionOffset );
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
    return t;
}

//-------------------------------------------------------------------

LabelNode::LabelNode(MapNode*            mapNode,
                     const osg::Vec3d&   position,
                     const std::string&  text,
                     const TextSymbol*   symbol ) :

LocalizedNode( mapNode->getMap()->getProfile()->getSRS(), position, true ),
_text( text )
{
    init( symbol );
}

LabelNode::LabelNode(const SpatialReference* mapSRS,
                     const osg::Vec3d&       position,
                     const std::string&      text,
                     const TextSymbol*       symbol ) :

LocalizedNode( mapSRS, position, true ),
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

    // By default, osgText assigns a render bin; we need to negate that in order
    // to activate the decluttering.
    osg::StateSet* stateSet = t->getOrCreateStateSet();
    stateSet->setRenderBinDetails( osg::StateSet::DEFAULT_BIN, OSGEARTH_DECLUTTER_BIN );
    stateSet->setMode( GL_DEPTH_TEST, 0 );

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( t );

    getTransform()->addChild( geode );
    this->addChild( getTransform() );
}
