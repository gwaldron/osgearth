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
#include <osgEarthFeatures/GeometryCompiler>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Util::Annotation;
using namespace osgEarth::Util::Controls;
using namespace OpenThreads;

//------------------------------------------------------------------------

AnnotationNode::AnnotationNode( MapNode* mapNode ) :
_mapNode( mapNode )
{
    this->setCullCallback( new CullNodeByHorizon(
        osg::Vec3d(0,0,1),
        _mapNode->getMap()->getProfile()->getSRS()->getEllipsoid()) );
}

void
AnnotationNode::setPosition( const osg::Vec3d& pos, const SpatialReference* srs )
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

//------------------------------------------------------------------------

PlacemarkNode::PlacemarkNode( MapNode* mapNode ) :
AnnotationNode( mapNode )
{
    //nop
}

PlacemarkNode::PlacemarkNode(const std::string& iconURI, const std::string& text, MapNode* mapNode ) :
AnnotationNode( mapNode ),
_iconURI( iconURI ),
_text   ( text )
{
    init();
}

void
PlacemarkNode::init()
{
    _label = new LabelControl(_text);

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

DrapeableNode::DrapeableNode( MapNode* mapNode, bool draped ) :
AnnotationNode( mapNode ),
_draped( draped )
{
    //nop
}

void
DrapeableNode::setDraped( bool value )
{
    if ( _draped != value )
    {
        osg::ref_ptr<osg::Node> save = _node.get();
        if ( save.valid() )
            setNode( 0L );

        _draped = value;

        if ( save.valid() )
            setNode( save.get() );
    }
}

void
DrapeableNode::setNode( osg::Node* node )
{
    if ( _node.valid() )
    {
        if ( _draped && _mapNode.valid() )
        {
            _mapNode->getOverlayGroup()->removeChild( _node.get() );
            _mapNode->updateOverlayGraph();
        }
        else
        {
            this->removeChild( _node.get() );
        }
    }

    _node = node;

    if ( _node.valid() )
    {
        if ( _draped && _mapNode.valid() )
        {
            _mapNode->getOverlayGroup()->addChild( _node.get() );
            _mapNode->updateOverlayGraph();
        }
        else
        {
            this->addChild( _node.get() );
        }
    }
}

//------------------------------------------------------------------------

FeatureNode::FeatureNode( Feature* feature, MapNode* mapNode, bool draped ) :
DrapeableNode( mapNode, draped ),
_feature     ( feature )
{
    init();
}

void
FeatureNode::init()
{
    GeometryCompilerOptions options;
    GeometryCompiler compiler( options );

    if ( _feature.valid() && _feature->getGeometry() )
    {
        Session* session = new Session( _mapNode->getMap() );
        GeoExtent extent(_mapNode->getMap()->getProfile()->getSRS(), _feature->getGeometry()->getBounds());
        FeatureProfile* profile = new FeatureProfile(extent);
        FilterContext context( session, profile, extent );
        
        osg::Node* node = compiler.compile( _feature.get(), *_feature->style(), context );
        setNode( node );
    }
}
