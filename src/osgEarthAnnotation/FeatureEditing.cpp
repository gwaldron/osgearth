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

#include <osgEarthAnnotation/FeatureEditing>
#include <osgEarth/Draggers>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Symbology;
using namespace osgEarth::Features;

/****************************************************************/
AddPointHandler::AddPointHandler(Feature* feature, FeatureListSource* source, const osgEarth::SpatialReference* mapSRS):
_feature(feature),
_source( source ),
_mapSRS( mapSRS ),
_mouseDown( false ),
_firstMove( false ),
_secondMove( false ),
_moveOption( POLYLINE_ONLY ),
_moveModKeyMask( 0 ),
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
AddPointHandler::addPoint( float x, float y, osgViewer::View* view, bool modifyLast )
{
    osgUtil::LineSegmentIntersector::Intersections results;
    if ( view->computeIntersections( x, y, results, _intersectionMask ) )
    {
        // find the first hit under the mouse:
        osgUtil::LineSegmentIntersector::Intersection first = *(results.begin());
        osg::Vec3d point = first.getWorldIntersectPoint();

        // transform it to map coordinates:
        double lat_rad, lon_rad, alt;
        _mapSRS->getEllipsoid()->convertXYZToLatLongHeight( point.x(), point.y(), point.z(), lat_rad, lon_rad, alt );

        double lat_deg = osg::RadiansToDegrees( lat_rad );
        double lon_deg = osg::RadiansToDegrees( lon_rad );

        if (_feature.valid())
        {
            if (modifyLast)
			{
                _feature->getGeometry()->pop_back();
			}
            _feature->getGeometry()->push_back( osg::Vec3d(lon_deg, lat_deg, alt) );
            _source->dirty();
        }
        return true;
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
            _secondMove = false;
            return addPoint( ea.getX(), ea.getY(), view, false );
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
        if ((_moveOption != NONE) && _mouseDown)
        {
            if (!_firstMove)
            {
                bool modifyLast = false;
                switch (_moveOption)
                {
                case POLYLINE_ONLY:
                    // nothing to do, drop through
                case NONE:
                default:
                    // should not get here
                    break;
                case LINE_ONLY:
                    modifyLast = _secondMove;
                    break;
                case LINE_DURING_MODKEY:
                    modifyLast = _secondMove;
                    if (_moveModKeyMask != 0)
                    {
                        modifyLast &= ((ea.getModKeyMask() & _moveModKeyMask) == _moveModKeyMask);
                    }
                    break;
                case POLY_DURING_MODKEY:
                    modifyLast = _secondMove;
                    if (_moveModKeyMask != 0)
                    {
                        modifyLast &= ((ea.getModKeyMask() & _moveModKeyMask) == _moveModKeyMask);
                    }
                    modifyLast = !modifyLast; // do just the opposite of LINE_DURING_MODKEY
                    break;
                }
                _secondMove = true; // would be used the next time
				return addPoint( ea.getX(), ea.getY(), view,  modifyLast );
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
          (*_feature->getGeometry())[_point] = osg::Vec3d(position.x(), position.y(), position.z());
          _source->dirty();
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
        dragger->setPosition(GeoPoint(_feature->getSRS(), (*_feature->getGeometry())[i].x(), (*_feature->getGeometry())[i].y(), (*_feature->getGeometry())[i].z(), ALTMODE_ABSOLUTE));
        dragger->addPositionChangedCallback(new MoveFeatureDraggerCallback(_feature.get(), _source.get(), i) );

        addChild(dragger);
    }
}
