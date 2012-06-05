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

#include <osgEarthAnnotation/PlaceNode>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthFeatures/BuildTextFilter>
#include <osgEarthFeatures/LabelSource>
#include <osgEarth/Utils>
#include <osgEarth/Registry>
#include <osg/Depth>

#define LC "[PlaceNode] "

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;


PlaceNode::PlaceNode(MapNode*           mapNode,
                     const GeoPoint&    position,
                     osg::Image*        image,
                     const std::string& text,
                     const Style&       style ) :

OrthoNode( mapNode, position ),
_image  ( image ),
_text   ( text ),
_style  ( style ),
_geode  ( 0L )
{
    init();
}

PlaceNode::PlaceNode(MapNode*           mapNode,
                     const GeoPoint&    position,
                     const std::string& text,
                     const Style&       style ) :

OrthoNode( mapNode, position ),
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

    if ( !_image.valid() && _style.getSymbol<MarkerSymbol>() )
    {
        _image = _style.getSymbol<MarkerSymbol>()->getImage();
    }

    if ( _image.get() )
    {
        // this offset anchors the image at the bottom
        osg::Vec2s offset;
        MarkerSymbol* marker = _style.get<MarkerSymbol>();
        if ((marker == NULL) || !marker->alignment().isSet())
        {	
            // default to bottom center
            offset.set(0.0, (_image->t() / 2.0));
        }
        else
        {	// default to bottom center
            switch (marker->alignment().value())
            {
            case MarkerSymbol::ALIGN_LEFT_TOP:
                offset.set((_image->s() / 2.0), -(_image->t() / 2.0));
                break;
            case MarkerSymbol::ALIGN_LEFT_CENTER:
                offset.set((_image->s() / 2.0), 0.0);
                break;
            case MarkerSymbol::ALIGN_LEFT_BOTTOM:
                offset.set((_image->s() / 2.0), (_image->t() / 2.0));
                break;
            case MarkerSymbol::ALIGN_CENTER_TOP:
                offset.set(0.0, -(_image->t() / 2.0));
                break;
            case MarkerSymbol::ALIGN_CENTER_CENTER:
                offset.set(0.0, 0.0);
                break;
            case MarkerSymbol::ALIGN_CENTER_BOTTOM:
            default:
                offset.set(0.0, (_image->t() / 2.0));
                break;
            case MarkerSymbol::ALIGN_RIGHT_TOP:
                offset.set(-(_image->s() / 2.0), -(_image->t() / 2.0));
                break;
            case MarkerSymbol::ALIGN_RIGHT_CENTER:
                offset.set(-(_image->s() / 2.0), 0.0);
                break;
            case MarkerSymbol::ALIGN_RIGHT_BOTTOM:
                offset.set(-(_image->s() / 2.0), (_image->t() / 2.0));
                break;
            }
        }

        // Apply a rotation to the marker if requested:
        double heading = 0.0;
        if ( marker && marker->orientation().isSet() )
        {
            //Just get the heading
            heading = osg::DegreesToRadians(marker->orientation().value().x());
        }

        //We must actually rotate the geometry itself and not use a MatrixTransform b/c the 
        //decluttering doesn't respect Transforms above the drawable.
        osg::Geometry* imageGeom = AnnotationUtils::createImageGeometry( _image.get(), offset, 0, heading );
        if ( imageGeom )
            _geode->addDrawable( imageGeom );

        text = AnnotationUtils::createTextDrawable(
            _text,
            _style.get<TextSymbol>(),
            osg::Vec3( (offset.x() + (_image->s() / 2.0) + 2), offset.y(), 0 ) );
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

    getAttachPoint()->addChild( _geode );

    // for clamping
    applyStyle( _style );
}

#if 0 // TODO
void
PlaceNode::setIconImage( osg::Image* image )
{
    if ( !_dynamic )
    {
        OE_WARN << LC << "Illegal state: cannot change a LabelNode that is not dynamic" << std::endl;
        return;
    }
}
#endif

void
PlaceNode::setText( const std::string& text )
{
    if ( !_dynamic )
    {
        OE_WARN << LC << "Illegal state: cannot change a LabelNode that is not dynamic" << std::endl;
        return;
    }

    _text = text;

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

#if 0 // TODO
void
PlaceNode::setStyle( const Style& style )
{
    if ( !_dynamic )
    {
        OE_WARN << LC << "Illegal state: cannot change a LabelNode that is not dynamic" << std::endl;
        return;
    }
}
#endif

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



//-------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( place, osgEarth::Annotation::PlaceNode );


PlaceNode::PlaceNode(MapNode*      mapNode,
                     const Config& conf ) :
OrthoNode( mapNode, GeoPoint::INVALID )
{
    conf.getObjIfSet( "style",  _style );
    conf.getIfSet   ( "text",   _text );

    optional<URI> imageURI;
    conf.getIfSet( "icon", imageURI );
    if ( imageURI.isSet() ) {
        _image = imageURI->getImage();
        if ( _image.valid() )
            _image->setFileName( imageURI->base() );
    }

    init();

    if ( conf.hasChild("position") )
        setPosition( GeoPoint(conf.child("position")) );
}

Config
PlaceNode::getConfig() const
{
    Config conf( "place" );
    conf.add   ( "text",   _text );
    conf.addObj( "style",  _style );
    conf.addObj( "position", getPosition() );
    if ( _image.valid() ) {
        if ( !_image->getFileName().empty() )
            conf.add( "icon", _image->getFileName() );
        else if ( !_image->getName().empty() )
            conf.add( "icon", _image->getName() );
    }

    return conf;
}
