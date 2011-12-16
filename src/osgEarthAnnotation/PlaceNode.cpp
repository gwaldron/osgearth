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
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthFeatures/BuildTextFilter>
#include <osgEarthFeatures/LabelSource>
#include <osgEarth/Utils>
#include <osg/Depth>

#define LC "[PlaceNode] "

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;


PlaceNode::PlaceNode(MapNode*           mapNode,
                     const osg::Vec3d&  position,
                     osg::Image*        image,
                     const std::string& text,
                     const Style&       style ) :

OrthoNode( mapNode->getMap()->getProfile()->getSRS(), position ),
_image  ( image ),
_text   ( text ),
_style  ( style ),
_geode  ( 0L )
{
    init();
}

void
PlaceNode::init()
{
    _geode = new osg::Geode();

    osg::Drawable* text = 0L;

    if ( _image.get() )
    {
        // this offset anchors the image at the bottom
        osg::Vec2s offset( 0.0, _image->t()/2.0 );
        osg::Geometry* imageGeom = AnnotationUtils::createImageGeometry( _image.get(), offset );
        if ( imageGeom )
            _geode->addDrawable( imageGeom );

        text = AnnotationUtils::createTextDrawable(
            _text,
            _style.get<TextSymbol>(),
            osg::Vec3( _image->s()/2.0 + 2, _image->t()/2.0, 0 ) );
    }
    else
    {
        text = AnnotationUtils::createTextDrawable(
            _text,
            _style.get<TextSymbol>(),
            osg::Vec3( 0, 0, 0 ) );
    }

    if ( text )
        _geode->addDrawable( text );
    
    osg::StateSet* stateSet = _geode->getOrCreateStateSet();
    stateSet->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), 1 );

    this->attach( _geode );
}

void
PlaceNode::setIconImage( osg::Image* image )
{
    if ( !_dynamic )
    {
        OE_WARN << LC << "Illegal state: cannot change a LabelNode that is not dynamic" << std::endl;
        return;
    }
}

void
PlaceNode::setText( const std::string& text )
{
    if ( !_dynamic )
    {
        OE_WARN << LC << "Illegal state: cannot change a LabelNode that is not dynamic" << std::endl;
        return;
    }

    const osg::Geode::DrawableList& list = _geode->getDrawableList();
    for( osg::Geode::DrawableList::const_iterator i = list.begin(); i != list.end(); ++i )
    {
        osgText::Text* d = dynamic_cast<osgText::Text*>( i->get() );
        if ( d )
        {
            d->setText( text );
            break;
        }
    }
}

void
PlaceNode::setStyle( const Style& style )
{
    if ( !_dynamic )
    {
        OE_WARN << LC << "Illegal state: cannot change a LabelNode that is not dynamic" << std::endl;
        return;
    }
}

void
PlaceNode::setAnnotationData( AnnotationData* data )
{
    OrthoNode::setAnnotationData( data );

    // override this method so we can attach the anno data to the drawables.
    const osg::Geode::DrawableList& list = _geode->getDrawableList();
    for( osg::Geode::DrawableList::const_iterator i = list.begin(); i != list.end(); ++i )
    {
        i->get()->setUserData( data );
    }
}

void
PlaceNode::setDynamic( bool value )
{
    OrthoNode::setDynamic( value );

    const osg::Geode::DrawableList& list = _geode->getDrawableList();
    for( osg::Geode::DrawableList::const_iterator i = list.begin(); i != list.end(); ++i )
    {
        i->get()->setDataVariance( value ? osg::Object::DYNAMIC : osg::Object::STATIC );
    }
}
