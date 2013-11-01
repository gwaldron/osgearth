/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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

#include <osgEarthAnnotation/FeatureEditing>
#include <osgEarth/Draggers>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Symbology;
using namespace osgEarth::Features;

/****************************************************************/
AddPointHandler::AddPointHandler( FeatureNode* featureNode):
_featureNode( featureNode ),
_mouseDown( false ),
_firstMove( false ),
_mouseButton( osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON ),
_intersectionMask( 0xffffffff )
{
}

void
AddPointHandler::setMouseButton( osgGA::GUIEventAdapter::MouseButtonMask mouseButton)
{
    _mouseButton = mouseButton;
}

osgGA::GUIEventAdapter::MouseButtonMask
AddPointHandler::getMouseButton() const
{
    return _mouseButton;
}

bool
AddPointHandler::addPoint( float x, float y, osgViewer::View* view )
{
    osg::Vec3d world;    
    MapNode* mapNode = _featureNode->getMapNode();

    if ( mapNode->getTerrain()->getWorldCoordsUnderMouse( view, x, y, world ) )
    {
        // Get the map point from the world
        GeoPoint mapPoint;
        mapPoint.fromWorld( mapNode->getMapSRS(), world );

        Feature* feature = _featureNode->getFeature();

        if ( feature )            
        {
            // Convert the map point to the feature's SRS
            GeoPoint featurePoint = mapPoint.transform( feature->getSRS() );

            feature->getGeometry()->push_back( featurePoint.vec3d() );            
            _featureNode->init();            
            return true;
        }        
    }
    return false;
}

bool
AddPointHandler::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
    osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
    if ( ea.getEventType() == osgGA::GUIEventAdapter::PUSH )
    {
        if (ea.getButton() == _mouseButton)
        {
            _mouseDown = true;
            _firstMove = true;
            return addPoint( ea.getX(), ea.getY(), view );
        }
    }
    else if (ea.getEventType() == osgGA::GUIEventAdapter::RELEASE)
    {
        if (ea.getButton() == _mouseButton)
        {
            _mouseDown = false;
        }
    }
    else if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE || ea.getEventType() == osgGA::GUIEventAdapter::DRAG)
    {
        if (_mouseDown)
        {
            if (!_firstMove)
            {
                return addPoint( ea.getX(), ea.getY(), view );
            }
            _firstMove = false;
        }
        return true;
    }

    return false;
}

/****************************************************************/

class MoveFeatureDraggerCallback : public Dragger::PositionChangedCallback
{
public:
    MoveFeatureDraggerCallback(FeatureNode* featureNode, int point):
      _featureNode( featureNode ),      
      _point(point)
      {}

      virtual void onPositionChanged(const Dragger* sender, const osgEarth::GeoPoint& position)
      {
          (*_featureNode->getFeature()->getGeometry())[_point] =  osg::Vec3d(position.x(), position.y(), 0);
          _featureNode->init();
      }

      osg::ref_ptr< FeatureNode > _featureNode;
      
      int _point;

};

/****************************************************************/
FeatureEditor::FeatureEditor( FeatureNode* featureNode):
_featureNode( featureNode ),
_color(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f)),
_pickColor(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f)),
_size( 5.0f )
{
    init();
}


const osg::Vec4f&
FeatureEditor::getPickColor() const
{
    return _pickColor;
}

void
FeatureEditor::setPickColor( const osg::Vec4f& pickColor )
{
    if (_pickColor != pickColor)
    {
        _pickColor = pickColor;
        init();
    }
}

const osg::Vec4f&
FeatureEditor::getColor() const
{
    return _color;
}

void
FeatureEditor::setColor( const osg::Vec4f& color )
{
    if (_color != color)
    {
        _color = color;
        init();
    }
}        

float
FeatureEditor::getSize() const
{
    return _size;
}

void
FeatureEditor::setSize( float size )
{
    if (_size != size)
    {
        _size = size;
        init();
    }
}

void
FeatureEditor::init()
{
    removeChildren( 0, getNumChildren() );

    Feature* feature = _featureNode->getFeature();
    //Create a dragger for each point
    for (unsigned int i = 0; i < _featureNode->getFeature()->getGeometry()->size(); i++)
    {
        SphereDragger* dragger = new SphereDragger( _featureNode->getMapNode() );
        dragger->setColor( _color );
        dragger->setPickColor( _pickColor );
        dragger->setSize( _size );
        dragger->setPosition(GeoPoint(feature->getSRS(),  (*feature->getGeometry())[i].x(),  (*feature->getGeometry())[i].y()));
        dragger->addPositionChangedCallback(new MoveFeatureDraggerCallback( _featureNode.get(), i) );

        addChild(dragger);        
    }
}        
