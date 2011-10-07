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

#include <osgEarthAnnotation/PlaceNode>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthFeatures/BuildTextFilter>
#include <osgEarthFeatures/LabelSource>
#include <osgEarth/Utils>
#include <osgText/Text>
#include <osg/ShapeDrawable>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;


PlaceNode::PlaceNode(MapNode*           mapNode,
                     const osg::Vec3d&  position,
                     osg::Image*        image,
                     const std::string& text,
                     const Style&       style ) :

LocalizedNode( mapNode->getMap()->getProfile()->getSRS(), position, true ),
_image  ( image ),
_text   ( text ),
_style  ( style )
{
    init();
}

void
PlaceNode::init()
{
    // remove any old stuff to make way for the new stuff.
    this->removeChildren(0, this->getNumChildren());

    this->addChild( getTransform() );
}

void
PlaceNode::setIconImage( osg::Image* image )
{
    if ( image )
    {
        _image = image;
        if ( _icon.valid() )
            _icon->setImage( image );
    }
}

void
PlaceNode::setText( const std::string& text )
{
    if ( text != _text )
    {
        _text = text;
        if ( _label.valid() )
            _label->setText( text );
    }
}

void
PlaceNode::setStyle( const Style& style )
{
    _style = style;
    init();
}
