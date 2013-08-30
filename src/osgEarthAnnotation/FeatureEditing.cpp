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
AddPointHandler::AddPointHandler(Feature* feature, FeatureSource* source, MapNode* mapNode):
_feature(feature),
_source( source ),
_mapNode( mapNode ),
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
    if (_mapNode->getTerrain()->getWorldCoordsUnderMouse( view, x, y, world ))
    {
        // Get the map point from the world
        GeoPoint mapPoint;
        mapPoint.fromWorld( _mapNode->getMapSRS(), world );

        if (_feature.valid())            
        {
            // Convert the map point to the feature's SRS
            GeoPoint featurePoint = mapPoint.transform( _feature->getSRS() );

            _feature->getGeometry()->push_back( featurePoint.vec3d() );
            _source->dirty();
            //Also must dirty the feature profile since the geometry has changed
            _source->dirtyFeatureProfile();
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
    MoveFeatureDraggerCallback(Feature* feature, FeatureSource* source, int point):
      _feature(feature),
      _source(source),
      _point(point)
      {}

      virtual void onPositionChanged(const Dragger* sender, const osgEarth::GeoPoint& position)
      {
          (*_feature->getGeometry())[_point] = osg::Vec3d(position.x(), position.y(), 0);
          _source->dirty();
          _source->dirtyFeatureProfile();
      }

      osg::ref_ptr< Feature > _feature;
      osg::ref_ptr< FeatureSource > _source;

      int _point;

};

/****************************************************************/
FeatureEditor::FeatureEditor( Feature* feature, FeatureSource* source, MapNode* mapNode ):
_feature( feature ),
_source( source ),
_mapNode( mapNode ),
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

    //Create a dragger for each point
    for (unsigned int i = 0; i < _feature->getGeometry()->size(); i++)
    {
        SphereDragger* dragger = new SphereDragger( _mapNode );
        dragger->setColor( _color );
        dragger->setPickColor( _pickColor );
        dragger->setSize( _size );
        dragger->setPosition(GeoPoint(_feature->getSRS(),  (*_feature->getGeometry())[i].x(),  (*_feature->getGeometry())[i].y()));
        dragger->addPositionChangedCallback(new MoveFeatureDraggerCallback(_feature.get(), _source.get(), i) );

        addChild(dragger);        
    }
}        
