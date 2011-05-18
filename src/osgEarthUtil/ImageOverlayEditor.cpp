#include <osgEarthUtil/ImageOverlayEditor>

#include <osg/Geode>
#include <osg/AutoTransform>
#include <osg/ShapeDrawable>
#include <osgViewer/Viewer>

using namespace osgEarth;
using namespace osgEarth::Util;

/***************************************************************************/

class ImageOverlayDraggerCallback : public osgManipulator::DraggerCallback
{
public:
    ImageOverlayDraggerCallback(ImageOverlay* overlay, const osg::EllipsoidModel* ellipsoid, ImageOverlay::ControlPoint controlPoint):
      _overlay(overlay),
          _ellipsoid(ellipsoid),
          _controlPoint(controlPoint)
      {}

      osg::Vec2d getLocation(const osg::Matrixd& matrix)
      {
          osg::Vec3d trans = matrix.getTrans();
          double lat, lon, height;
          _ellipsoid->convertXYZToLatLongHeight(trans.x(), trans.y(), trans.z(), lat, lon, height);
          return osg::Vec2d(osg::RadiansToDegrees(lon), osg::RadiansToDegrees(lat));
      }


      virtual bool receive(const osgManipulator::MotionCommand& command)
      {
          switch (command.getStage())
          {
          case osgManipulator::MotionCommand::START:
              {
                  // Save the current matrix
                  osg::Vec2d startLocation = _overlay->getControlPoint(_controlPoint);
                  double x, y, z;
                  _ellipsoid->convertLatLongHeightToXYZ(osg::DegreesToRadians(startLocation.y()), osg::DegreesToRadians(startLocation.x()), 0, x, y, z);
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
                  _overlay->setControlPoint(_controlPoint, location.x(), location.y());

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


      osg::ref_ptr<const osg::EllipsoidModel>            _ellipsoid;
      osg::ref_ptr<ImageOverlay>           _overlay;

      osg::Matrix _startMotionMatrix;
      ImageOverlay::ControlPoint _controlPoint;

      osg::Matrix _localToWorld;
      osg::Matrix _worldToLocal;
};

/***************************************************************************/
class UpdateDraggersCallback : public osgManipulator::DraggerCallback
{
public:
    UpdateDraggersCallback(ImageOverlayEditor* editor)
    {
        _editor = editor;
    }

    virtual bool receive(const osgManipulator::MotionCommand& command)
    {
        for (ImageOverlayEditor::ControlPointDraggerMap::iterator itr = _editor->getDraggers().begin(); itr != _editor->getDraggers().end(); ++itr)
        {
            //Get the location of the control point
            osg::Vec2d location = _editor->getOverlay()->getControlPoint( itr->first );

            osg::Matrixd matrix;

            //Compute the geocentric location
            osg::ref_ptr< const osg::EllipsoidModel > ellipsoid = _editor->getMapNode()->getMap()->getProfile()->getSRS()->getEllipsoid();
            osg::Vec3d geo_loc;
            ellipsoid->convertLatLongHeightToXYZ(osg::DegreesToRadians(location.y()), osg::DegreesToRadians(location.x()), 0, geo_loc.x(), geo_loc.y(), geo_loc.z());

            osg::Vec3d up = geo_loc;
            up.normalize();

            double segOffset = 50000;

            osg::Vec3d start = geo_loc + (up * segOffset);
            osg::Vec3d end = geo_loc - (up * segOffset);

            osgUtil::LineSegmentIntersector* i = new osgUtil::LineSegmentIntersector( start, end );

            osgUtil::IntersectionVisitor iv;
            iv.setIntersector( i );
            _editor->getMapNode()->accept( iv );

            osgUtil::LineSegmentIntersector::Intersections& results = i->getIntersections();
            if ( !results.empty() )
            {
                const osgUtil::LineSegmentIntersector::Intersection& result = *results.begin();
                geo_loc = result.getWorldIntersectPoint();
            }

            _editor->getMapNode()->getMap()->getProfile()->getSRS()->getEllipsoid()->computeLocalToWorldTransformFromXYZ(geo_loc.x(), geo_loc.y(), geo_loc.z(), matrix);


            itr->second.get()->setMatrix( matrix );                               

        }
        return false;
    }

    friend class ImageOverlayEditor;

    ImageOverlayEditor* _editor;
};

/***************************************************************************/




ImageOverlayEditor::ImageOverlayEditor(ImageOverlay* overlay, MapNode* mapNode):
_overlay(overlay),
_mapNode(mapNode)
{   
    _updateDraggersCallback = new UpdateDraggersCallback(this);
    addDragger( ImageOverlay::CONTROLPOINT_CENTER );
    addDragger( ImageOverlay::CONTROLPOINT_LOWER_LEFT );
    addDragger( ImageOverlay::CONTROLPOINT_LOWER_RIGHT );
    addDragger( ImageOverlay::CONTROLPOINT_UPPER_LEFT );
    addDragger( ImageOverlay::CONTROLPOINT_UPPER_RIGHT );
}

void
ImageOverlayEditor::addDragger( ImageOverlay::ControlPoint controlPoint )
{
    osg::Vec2d location = _overlay->getControlPoint( controlPoint );
    osg::Matrixd matrix;
    _mapNode->getMap()->getProfile()->getSRS()->getEllipsoid()->computeLocalToWorldTransformFromLatLongHeight(osg::DegreesToRadians(location.y()), osg::DegreesToRadians(location.x()), 0, matrix);    

    IntersectingDragger* dragger = new IntersectingDragger;
    dragger->setNode( _mapNode );
    dragger->setupDefaultGeometry();
    dragger->setMatrix(matrix);
    dragger->setHandleEvents( true );
    dragger->addDraggerCallback(new ImageOverlayDraggerCallback(_overlay, _mapNode->getMap()->getProfile()->getSRS()->getEllipsoid(), controlPoint));
    dragger->addDraggerCallback( this->_updateDraggersCallback.get() );

    addChild(dragger);
    _draggers[ controlPoint ] = dragger;
}
