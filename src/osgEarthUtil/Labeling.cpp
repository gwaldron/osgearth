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
#include <osgEarthUtil/Labeling>
#include <osgEarth/HTTPClient>
#include <osgText/Text>
#include <osg/Depth>
#include <osgDB/ReadFile>
#include <osgWidget/ViewerEventHandlers>

using namespace osgEarth;
using namespace osgEarthUtil;
using namespace osgEarth::Symbology;

static const TextSymbol* s_defaultSymbol = new TextSymbol();


LabelMaker::LabelMaker( const TextSymbol* defaultSymbol ) :
_defaultSymbol( defaultSymbol )
{
    //NOP
}

osg::Drawable*
LabelMaker::create2dLabel( const std::string& text, const osg::Vec3d& pos, const TextSymbol* textSym )
{
    const TextSymbol* symbol = textSym ? textSym : _defaultSymbol.valid() ? _defaultSymbol.get() : s_defaultSymbol;

    osgText::Text* t = new osgText::Text();
    t->setText( text );

    if ( symbol->font().isSet() )
        t->setFont( symbol->font().value() );

    t->setAutoRotateToScreen( symbol->rotateToScreen().value() );

    t->setCharacterSizeMode( 
        symbol->sizeMode() == TextSymbol::SIZEMODE_SCREEN ? osgText::TextBase::SCREEN_COORDS :
        osgText::TextBase::OBJECT_COORDS );

    t->setCharacterSize( symbol->size().value() );
    t->setPosition( pos );
    t->setAlignment( osgText::TextBase::CENTER_CENTER );
    t->getOrCreateStateSet()->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS), osg::StateAttribute::ON );
    t->getOrCreateStateSet()->setMode( GL_DEPTH_TEST, 0 );
    t->getOrCreateStateSet()->setRenderBinDetails( 99999, "RenderBin" );

    if ( symbol->fill().isSet() )
        t->setColor( symbol->fill()->color() );

    if ( symbol->halo().isSet() )
    {
        t->setBackdropColor( symbol->halo()->color() );
        t->setBackdropType( osgText::Text::OUTLINE );
    }

    return t;
}

