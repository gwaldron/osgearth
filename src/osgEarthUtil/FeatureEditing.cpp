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

#include <osgEarthUtil/FeatureEditing>
#include <osgEarthUtil/Draggers>

using namespace osgEarth::Util;
using namespace osgEarth::Symbology;
using namespace osgEarth::Features;

/****************************************************************/
AddPointHandler::AddPointHandler(Feature* feature, FeatureListSource* source, const osgEarth::SpatialReference* mapSRS):
_feature(feature),
_source( source ),
_mapSRS( mapSRS ),
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
    osgUtil::LineSegmentIntersector::Intersections results;
    if ( view->computeIntersections( x, y, results, _intersectionMask ) )
    {
        // find the first hit under the mouse:
        osgUtil::LineSegmentIntersector::Intersection first = *(results.begin());
        osg::Vec3d point = first.getWorldIntersectPoint();

        // transform it to map coordinates:
        double lat_rad, lon_rad, dummy;
        _mapSRS->getEllipsoid()->convertXYZToLatLongHeight( point.x(), point.y(), point.z(), lat_rad, lon_rad, dummy );

        double lat_deg = osg::RadiansToDegrees( lat_rad );
        double lon_deg = osg::RadiansToDegrees( lon_rad );

        if (_feature.valid())            
        {
            _feature->getGeometry()->push_back( osg::Vec3d(lon_deg, lat_deg, 0) );
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

class MoveFeatureDraggerCallback : public osgManipulator::DraggerCallback
{
public:
    MoveFeatureDraggerCallback(Feature* feature, FeatureSource* source, const Map* map, int point):
      _feature(feature),
      _source(source),
      _map(map),
      _point(point)
      {}

      osg::Vec2d getLocation(const osg::Matrixd& matrix)
      {
          osg::Vec3d trans = matrix.getTrans();
          double lat, lon, height;
          _map->getProfile()->getSRS()->getEllipsoid()->convertXYZToLatLongHeight(trans.x(), trans.y(), trans.z(), lat, lon, height);
          return osg::Vec2d(osg::RadiansToDegrees(lon), osg::RadiansToDegrees(lat));
      }


      virtual bool receive(const osgManipulator::MotionCommand& command)
      {
          switch (command.getStage())
          {
          case osgManipulator::MotionCommand::START:
              {
                  // Save the current matrix                  
                  osg::Vec3d startLocation = (*_feature->getGeometry())[_point];
                  double x, y, z;
                  _map->getProfile()->getSRS()->getEllipsoid()->convertLatLongHeightToXYZ(osg::DegreesToRadians(startLocation.y()), osg::DegreesToRadians(startLocation.x()), 0, x, y, z);
                  _startMotionMatrix = osg::Matrixd::translate(x, y, z);

                  // Get the LocalToWorld and WorldToLocal matrix for this node.
                  osg::NodePath nodePathToRoot;
                  _localToWorld = osg::Matrixd::identity();
                  _worldToLocal = osg::Matrixd::identity();

                  return true;
              }
          case osgManipulator::MotionCommand::MOVE:
              {
                  // Transform the command's motion matrix into local motion matrix.
                  osg::Matrix localMotionMatrix = _localToWorld * command.getWorldToLocal()
                      * command.getMotionMatrix()
                      * command.getLocalToWorld() * _worldToLocal;

                  osg::Matrixd newMatrix = localMotionMatrix * _startMotionMatrix;
                  osg::Vec2d location = getLocation( newMatrix );
                  (*_feature->getGeometry())[_point] = osg::Vec3d(location.x(), location.y(), 0);
                  _source->dirty();

                  return true;
              }
          case osgManipulator::MotionCommand::FINISH:
              {
                  return true;
              }
          case osgManipulator::MotionCommand::NONE:
          default:
              return false;
          }
      }


      osg::ref_ptr<const Map>            _map;      
      osg::ref_ptr< Feature > _feature;
      osg::ref_ptr< FeatureSource > _source;

      osg::Matrix _startMotionMatrix;
      int _point;

      osg::Matrix _localToWorld;
      osg::Matrix _worldToLocal;
};

/****************************************************************/
FeatureEditor::FeatureEditor( Feature* feature, FeatureSource* source, MapNode* mapNode ):
_feature( feature ),
_source( source ),
_mapNode( mapNode )
{
    init();
}

void
FeatureEditor::init()
{
    removeChildren( 0, this->getNumChildren() );
    //Create a dragger for each point
    for (unsigned int i = 0; i < _feature->getGeometry()->size(); i++)
    {
        osg::Matrixd matrix;
        double lat = (*_feature->getGeometry())[i].y();
        double lon = (*_feature->getGeometry())[i].x();
        _mapNode->getMap()->getProfile()->getSRS()->getEllipsoid()->computeLocalToWorldTransformFromLatLongHeight(osg::DegreesToRadians(lat), osg::DegreesToRadians(lon), 0, matrix);    

        IntersectingDragger* dragger = new IntersectingDragger;
        dragger->setNode( _mapNode->getTerrainEngine() );
        dragger->setupDefaultGeometry();
        dragger->setMatrix(matrix);
        dragger->setHandleEvents( true );        
        dragger->addDraggerCallback(new MoveFeatureDraggerCallback(_feature, _source, _mapNode->getMap(), i) );

        addChild(dragger);        
    }
}        