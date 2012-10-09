/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthSymbology/Color>
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
_text    ( text ),
_geode   ( 0L )
{
    init( style );
}

LabelNode::LabelNode(MapNode*            mapNode,
                     const GeoPoint&     position,
                     const std::string&  text,
                     const TextSymbol*   symbol ) :

OrthoNode( mapNode, position ),
_text    ( text ),
_geode   ( 0L )
{
    Style style;
    style.add( const_cast<TextSymbol*>(symbol) );
    init( style );
}

LabelNode::LabelNode(const std::string&  text,
                     const Style&        style ) :
OrthoNode(),
_text    ( text ),
_geode   ( 0L )
{
    init( style );
}

LabelNode::LabelNode(MapNode*            mapNode,
                     const GeoPoint&     position,
                     const Style&        style ) :
OrthoNode( mapNode, position ),
_geode   ( 0L )
{
    init( style );
}

void
LabelNode::init( const Style& style )
{
    _geode = new osg::Geode();
    getAttachPoint()->addChild( _geode );

    osg::StateSet* stateSet = _geode->getOrCreateStateSet();
    stateSet->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), 1 );
    stateSet->setRenderBinDetails(INT_MAX, "RenderBin");	// Force labels nodes to be drawn last

    AnnotationUtils::installAnnotationProgram( stateSet );

    setStyle( style );

    applyStyle( style );

    setLightingIfNotSet( false );
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

    _geode->removeDrawables( 0, _geode->getNumDrawables() );

    _style = style;

    const TextSymbol* symbol = _style.get<TextSymbol>();

    if ( _text.empty() )
        _text = symbol->content()->eval();

    osg::Drawable* t = AnnotationUtils::createTextDrawable( _text, symbol, osg::Vec3(0,0,0) );
    _geode->addDrawable(t);

    applyStyle( _style );

    setLightingIfNotSet( false );
}

void
LabelNode::setAnnotationData( AnnotationData* data )
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
