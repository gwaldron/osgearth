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

#include <osgEarthAnnotation/AnnotationEditing>

#include <osg/io_utils>

using namespace osgEarth::Annotation;
using namespace osgEarth::Symbology;

/**********************************************************************/
class DraggerCallback : public osgManipulator::DraggerCallback
{
public:
    DraggerCallback(LocalizedNode* node):
      _node(node)
      {
          _ellipsoid = _node->getMapNode()->getMap()->getProfile()->getSRS()->getEllipsoid();
      }

      osg::Vec3d getLocation(const osg::Matrixd& matrix)
      {
          osg::Vec3d trans = matrix.getTrans();
          double lat, lon, height;
          _ellipsoid->convertXYZToLatLongHeight(trans.x(), trans.y(), trans.z(), lat, lon, height);
          return osg::Vec3d(osg::RadiansToDegrees(lon), osg::RadiansToDegrees(lat), height);
      }

      virtual bool receive(const osgManipulator::MotionCommand& command)
      {
          switch (command.getStage())
          {
          case osgManipulator::MotionCommand::START:
              {
                  // Save the current matrix
                  osg::Vec3d location = _node->getPosition();
                  double x, y, z;
                  _ellipsoid->convertLatLongHeightToXYZ(osg::DegreesToRadians(location.y()), osg::DegreesToRadians(location.x()), location.z(), x, y, z);
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
                  osg::Vec3d location = getLocation( newMatrix );

                  _node->setPosition( location );                  

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
      LocalizedNode* _node;

      osg::Matrix _startMotionMatrix;

      osg::Matrix _localToWorld;
      osg::Matrix _worldToLocal;
};

/**********************************************************************/
LocalizedNodeEditor::LocalizedNodeEditor(LocalizedNode* node):
_node( node )
{
    _dragger  = new IntersectingDragger;
    _dragger->setNode( _node->getMapNode() );    
    _dragger->setHandleEvents( true );
    _dragger->addDraggerCallback(new DraggerCallback(_node ) );        
    _dragger->setupDefaultGeometry();    
    addChild(_dragger);

    updateDraggers();
}

LocalizedNodeEditor::~LocalizedNodeEditor()
{    
}

void
LocalizedNodeEditor::updateDraggers()
{
    const osg::EllipsoidModel* em = _node->getMapNode()->getMap()->getProfile()->getSRS()->getEllipsoid();

    osg::Matrixd matrix;
    osg::Vec3d location = _node->getPosition();    
    em->computeLocalToWorldTransformFromLatLongHeight(osg::DegreesToRadians( location.y() ),  osg::DegreesToRadians(location.x()), location.z(), matrix);
    _dragger->setMatrix(matrix);        
}