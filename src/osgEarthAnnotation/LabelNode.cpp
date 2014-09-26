/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthSymbology/Color>
#include <osgEarth/Registry>
#include <osgEarth/ShaderGenerator>
#include <osgText/Text>
#include <osg/Depth>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

#define LC "[LabelNode] "

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Symbology;


//-------------------------------------------------------------------

LabelNode::LabelNode(MapNode*            mapNode,
                     const GeoPoint&     position,
                     const std::string&  text,
                     const Style&        style ) :

OrthoNode( mapNode, position ),
_text    ( text )
{
    init( style );
}

LabelNode::LabelNode(MapNode*            mapNode,
                     const GeoPoint&     position,
                     const std::string&  text,
                     const TextSymbol*   symbol ) :

OrthoNode( mapNode, position ),
_text    ( text )
{
    Style style;
    style.add( const_cast<TextSymbol*>(symbol) );
    init( style );
}

LabelNode::LabelNode(const std::string&  text,
                     const Style&        style ) :
OrthoNode(),
_text    ( text )
{
    init( style );
}

LabelNode::LabelNode(MapNode*            mapNode,
                     const GeoPoint&     position,
                     const Style&        style ) :
OrthoNode( mapNode, position )
{
    init( style );
}

LabelNode::LabelNode(MapNode*            mapNode,
                     const Style&        style ) :
OrthoNode( mapNode, GeoPoint::INVALID )
{
    init( style );
}

void
LabelNode::init( const Style& style )
{
    _geode = new osg::Geode();
    getAttachPoint()->addChild( _geode.get() );

    osg::StateSet* stateSet = _geode->getOrCreateStateSet();
    stateSet->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), 1 );

    setStyle( style );
}

void
LabelNode::setText( const std::string& text )
{
    if ( !_dynamic && getNumParents() > 0 )
    {
        OE_WARN << LC << "Illegal state: cannot change a LabelNode that is not dynamic" << std::endl;
        return;
    }

    osgText::Text* d = dynamic_cast<osgText::Text*>(_geode->getDrawable(0));
    if ( d )
    {
        d->setText( text );
        d->dirtyDisplayList();
        _text = text;
    }
}

void
LabelNode::setStyle( const Style& style )
{
    if ( !_dynamic && getNumParents() > 0 )
    {
        OE_WARN << LC << "Illegal state: cannot change a LabelNode that is not dynamic" << std::endl;
        return;
    }
    
    this->clearDecoration();

    _geode->removeDrawables( 0, _geode->getNumDrawables() );

    _style = style;

    const TextSymbol* symbol = _style.get<TextSymbol>();

    if ( _text.empty() )
        _text = symbol->content()->eval();

    osg::Drawable* t = AnnotationUtils::createTextDrawable( _text, symbol, osg::Vec3(0,0,0) );
    _geode->addDrawable(t);

    applyStyle( _style );

    setLightingIfNotSet( false );

    Registry::shaderGenerator().run(
        this,
        "osgEarth.LabelNode",
        Registry::stateSetCache() );
}

void
LabelNode::setAnnotationData( AnnotationData* data )
{
    OrthoNode::setAnnotationData( data );

    // override this method so we can attach the anno data to the drawables.
    for(unsigned i=0; i<_geode->getNumDrawables(); ++i)
    {
        _geode->getDrawable(i)->setUserData( data );
    }
}

void
LabelNode::setDynamic( bool dynamic )
{
    OrthoNode::setDynamic( dynamic );

    osgText::Text* d = dynamic_cast<osgText::Text*>(_geode->getDrawable(0));
    if ( d )
    {
        d->setDataVariance( dynamic ? osg::Object::DYNAMIC : osg::Object::STATIC );
    }    
}



//-------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( label, osgEarth::Annotation::LabelNode );


LabelNode::LabelNode(MapNode*               mapNode,
                     const Config&         conf,
                     const osgDB::Options* dbOptions ) :
OrthoNode( mapNode, GeoPoint::INVALID )
{
    optional<Style> style;

    conf.getObjIfSet( "style", style );
    conf.getIfSet   ( "text",  _text );

    init( *style );

    if ( conf.hasChild("position") )
        setPosition( GeoPoint(conf.child("position")) );
}

Config
LabelNode::getConfig() const
{
    Config conf( "label" );
    conf.add   ( "text",   _text );
    conf.addObj( "style",  _style );
    conf.addObj( "position", getPosition() );

    return conf;
}
