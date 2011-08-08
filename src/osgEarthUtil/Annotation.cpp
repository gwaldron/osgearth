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

#include <osgEarthUtil/Annotation>
#include <osgEarth/HTTPClient>
#include <osgEarth/Utils>
#include <osgEarthSymbology/GeometryFactory>
#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarthFeatures/BuildGeometryFilter>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Util::Annotation;
using namespace osgEarth::Util::Controls;

//------------------------------------------------------------------------

PlacemarkNode::PlacemarkNode( MapNode* mapNode ) :
_mapNode( mapNode )
{
    init();
}

PlacemarkNode::PlacemarkNode(MapNode*           mapNode, 
                             const std::string& iconURI, 
                             const std::string& text,
                             const Style&       style) :
_mapNode( mapNode ),
_iconURI( iconURI ),
_text   ( text ),
_style  ( style )
{
    init();
}

void
PlacemarkNode::setPosition( const osg::Vec3d& pos, const SpatialReference* srs )
{
    if ( _mapNode.valid() )
    {
        if ( !srs )
        {
            osg::Vec3d world;
            if ( _mapNode->getMap()->mapPointToWorldPoint(pos, world) )
            {
                this->setMatrix( osg::Matrix::translate(world) );
                static_cast<CullNodeByHorizon*>(this->getCullCallback())->_world = world;
            }
        }
        else
        {
            osg::Vec3d map, world;
            if ( _mapNode->getMap()->toMapPoint(pos, srs, map) )
            {
                if ( _mapNode->getMap()->mapPointToWorldPoint(map, world) )
                {
                    this->setMatrix( osg::Matrix::translate(world) );
                    static_cast<CullNodeByHorizon*>(this->getCullCallback())->_world = world;
                }
            }
        }
    }
}

void
PlacemarkNode::init()
{
    this->setCullCallback( new CullNodeByHorizon(
        osg::Vec3d(0,0,1),
        _mapNode->getMap()->getProfile()->getSRS()->getEllipsoid()) );

    _label = new LabelControl(_text);

    TextSymbol* s = _style.get<TextSymbol>();
    if ( s )
    {
        if ( s->font().isSet() )
            _label->setFont( osgText::readFontFile( *s->font() ) );
        if ( s->size().isSet() )
            _label->setFontSize( *s->size() );
        if ( s->fill().isSet() )
            _label->setForeColor( s->fill()->color() );
        if ( s->halo().isSet() )
            _label->setHaloColor( s->halo()->color() );
    }

    osg::ref_ptr<osg::Image> image;
    if ( HTTPClient::readImageFile(_iconURI, image) == HTTPClient::RESULT_OK )
        _icon = new ImageControl( image.get() );

    _container = new HBox();
    _container->setChildSpacing( 8 );
    
    if ( _icon.valid() )
        _container->addControl( _icon.get() );

    _container->addControl( _label.get() );

    _container->setHorizAlign( Control::ALIGN_RIGHT );
    _container->setVertAlign( Control::ALIGN_CENTER );

    //temp:
    //_container->setFrame( new Frame() );

    //todo: set up the "ANCHOR POINT" for the sweet spot

    // wrap the other controls in a scene node.
    ControlNode* node = new ControlNode( _container.get() );
    this->addChild( node );
}

void
PlacemarkNode::setIconURI( const std::string& iconURI )
{
    if ( iconURI != _iconURI )
    {
        _iconURI = iconURI;
        osg::ref_ptr<osg::Image> image;
        if ( HTTPClient::readImageFile(_iconURI, image) == HTTPClient::RESULT_OK )
        {
            _icon->setImage( image.get() );
            // add if necessary?
        }
    }
}

void PlacemarkNode::setText( const std::string& text )
{
    if ( text != _text )
    {
        _text = text;
        _label->setText( text );
    }
}

//------------------------------------------------------------------------

GeometryNode::GeometryNode(Geometry*    geom,
                           const Style& style) :
DrapeableNode( NULL ),
_geom        ( geom ),
_style       ( style )
{
    init();
}

GeometryNode::GeometryNode(MapNode*     mapNode,
                           Geometry*    geom,
                           const Style& style,
                           bool         draped ) :
DrapeableNode( mapNode, draped ),
_geom        ( geom ),
_style       ( style )
{
    init();
}

void
GeometryNode::init()
{
    FeatureList features;
    features.push_back( new Feature(_geom.get(), _style) );
    BuildGeometryFilter bg;
    bg.push( features, FilterContext() );
    osg::Node* node = bg.getNode();
    setNode( node );
}

//------------------------------------------------------------------------

// options:
// - geodetic, draped
// - projected
// - localized

CircleNode::CircleNode(MapNode*                mapNode,
                       const osg::Vec3d&       center,
                       const Linear&           radius,
                       const Style&            style,
                       bool                    draped,
                       unsigned                numSegments) :
FeatureNode( mapNode, 0L, draped )
{
    if ( mapNode )
    {
        const SpatialReference* targetSRS = mapNode->getMap()->getProfile()->getSRS();

        GeometryFactory factory( targetSRS );

        Geometry* geom = factory.createCircle(center, radius, numSegments);
        if ( geom )
        {
            Feature* feature = new Feature( geom, style );
            feature->geoInterp() = GEOINTERP_GREAT_CIRCLE;
            setFeature( feature );
        }
    }
}

//------------------------------------------------------------------------

EllipseNode::EllipseNode(MapNode*          mapNode,
                         const osg::Vec3d& center,
                         const Linear&     radiusMajor,
                         const Linear&     radiusMinor,
                         const Angular&    rotationAngle,
                         const Style&      style,
                         bool              draped,
                         unsigned          numSegments) :
FeatureNode( mapNode, 0L, draped )
{
    if ( mapNode )
    {
        GeometryFactory factory( mapNode->getMap()->getProfile()->getSRS() );
        Geometry* geom = factory.createEllipse(center, radiusMajor, radiusMinor, rotationAngle, numSegments);
        if ( geom )
        {
            Feature* feature = new Feature( geom, style );
            feature->geoInterp() = GEOINTERP_GREAT_CIRCLE;
            setFeature( feature );
        }
    }
}
