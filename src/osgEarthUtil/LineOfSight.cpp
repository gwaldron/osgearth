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
#include <osgEarthUtil/LineOfSight>
#include <osgSim/LineOfSight>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Annotation;

LineOfSightNode::LineOfSightNode(osgEarth::MapNode *mapNode):
_mapNode(mapNode),
_start(0,0,0),
_end(0,0,0),
_hit(0,0,0),
_hasLOS( true ),
_goodColor(0.0f, 1.0f, 0.0f, 1.0f),
_badColor(1.0f, 0.0f, 0.0f, 1.0f),
_displayMode( MODE_SPLIT )
{
    compute();
}

LineOfSightNode::LineOfSightNode(osgEarth::MapNode *mapNode, const osg::Vec3d& start, const osg::Vec3d& end):
_mapNode(mapNode),
_start(start),
_end(end),
_hit(0,0,0),
_hasLOS( true ),
_goodColor(0.0f, 1.0f, 0.0f, 1.0f),
_badColor(1.0f, 0.0f, 0.0f, 1.0f),
_displayMode( MODE_SPLIT )
{
    compute();
}

const osg::Vec3d&
LineOfSightNode::getStart() const
{
    return _start;
}

void
LineOfSightNode::setStart(const osg::Vec3& start)
{
    if (_start != start)
    {
        _start = start;
        compute();
    }
}

const osg::Vec3d&
LineOfSightNode::getEnd() const
{
    return _end;
}

void
LineOfSightNode::setEnd(const osg::Vec3& end)
{
    if (_end != end)
    {
        _end = end;
        compute();
    }
}

const osg::Vec3d&
LineOfSightNode::getHit() const
{
    return _hit;
}

bool
LineOfSightNode::getHasLOS() const
{
    return _hasLOS;
}


void
LineOfSightNode::compute()
{
    //Computes the LOS and redraws the scene
    osg::Vec3d a, b;
    _mapNode->getMap()->mapPointToWorldPoint( _start, a );
    _mapNode->getMap()->mapPointToWorldPoint( _end, b );
    //LineOfSight los(a, b);
    osgSim::LineOfSight los;
    los.setDatabaseCacheReadCallback(0);
    unsigned int index = los.addLOS(a, b);
    los.computeIntersections(_mapNode.get());
    osgSim::LineOfSight::Intersections hits = los.getIntersections(0);    
    if (hits.size() > 0)
    {
        _hasLOS = false;
        _hit = *hits.begin();
    }
    else
    {
        _hasLOS = true;
    }

    _mapNode->getMap()->worldPointToMapPoint( _hit, _hit);
    draw();
}

void
LineOfSightNode::draw()
{
    //Remove all children from this group
    removeChildren(0, getNumChildren());

    osg::Geometry* geometry = new osg::Geometry;
    osg::Vec3Array* verts = new osg::Vec3Array();
    verts->reserve(4);
    geometry->setVertexArray( verts );

    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->reserve( 4 );

    geometry->setColorArray( colors );
    geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    osg::Vec3d startWorld, endWorld, hitWorld;
    _mapNode->getMap()->mapPointToWorldPoint( _start, startWorld );
    _mapNode->getMap()->mapPointToWorldPoint( _end, endWorld );
    _mapNode->getMap()->mapPointToWorldPoint( _hit, hitWorld );

    if (_hasLOS)
    {
        verts->push_back( startWorld - startWorld );
        verts->push_back( endWorld   - startWorld );
        colors->push_back( _goodColor );
        colors->push_back( _goodColor );
    }
    else
    {
        if (_displayMode == MODE_SINGLE)
        {
            verts->push_back( startWorld - startWorld );
            verts->push_back( endWorld - startWorld );
            colors->push_back( _badColor );
            colors->push_back( _badColor );
        }
        else if (_displayMode == MODE_SPLIT)
        {
            verts->push_back( startWorld - startWorld );
            colors->push_back( _goodColor );
            verts->push_back( hitWorld   - startWorld );
            colors->push_back( _goodColor );

            verts->push_back( hitWorld   - startWorld );
            colors->push_back( _badColor );
            verts->push_back( endWorld   - startWorld );
            colors->push_back( _badColor );
        }
    }

    geometry->addPrimitiveSet( new osg::DrawArrays( GL_LINES, 0, verts->size()) );        

    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( geometry );

    osg::MatrixTransform* mt = new osg::MatrixTransform;
    mt->setMatrix(osg::Matrixd::translate(startWorld));
    mt->addChild(geode);  

    getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);


    addChild( mt );
}

void
LineOfSightNode::setGoodColor( const osg::Vec4f &color )
{
    if (_goodColor != color)
    {
        _goodColor = color;
        draw();
    }
}

const osg::Vec4f&
LineOfSightNode::getGoodColor() const
{
    return _goodColor;
}

void
LineOfSightNode::setBadColor( const osg::Vec4f &color )
{
    if (_badColor != color)
    {
        _badColor = color;
        draw();
    }
}

const osg::Vec4f&
LineOfSightNode::getBadColor() const
{
    return _badColor;
}

LOSDisplayMode
LineOfSightNode::getDisplayMode() const
{
    return _displayMode;
}

void
LineOfSightNode::setDisplayMode( LOSDisplayMode displayMode )
{
    if (_displayMode != displayMode)
    {
        _displayMode = displayMode;
        draw();
    }
}

osg::Vec3d getNodeCenter(osg::Node* node)
{
    osg::NodePathList nodePaths = node->getParentalNodePaths();
    if ( nodePaths.empty() )
        return node->getBound().center();

    osg::NodePath path = nodePaths[0];

    osg::Matrixd localToWorld = osg::computeLocalToWorld( path );
    osg::Vec3d center = osg::Vec3d(0,0,0) * localToWorld;

    // if the tether node is a MT, we are set. If it's not, we need to get the
    // local bound and add its translation to the localToWorld. We cannot just use
    // the bounds directly because they are single precision (unless you built OSG
    // with double-precision bounding spheres, which you probably did not :)
    if ( !dynamic_cast<osg::MatrixTransform*>( node ) )
    {
        const osg::BoundingSphere& bs = node->getBound();
        center += bs.center();
    }   
    return center;
}

/**********************************************************************/
LineOfSightTether::LineOfSightTether(osg::Node* startNode, osg::Node* endNode):
_startNode(startNode),
_endNode(endNode)
{
}

void 
LineOfSightTether::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    if (nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
    {
        LineOfSightNode* los = static_cast<LineOfSightNode*>(node);

        osg::Vec3d start = getNodeCenter( _startNode );
        osg::Vec3d end   = getNodeCenter( _endNode );

        //Convert these to mappoints since that is what LOS expects
        los->getMapNode()->getMap()->worldPointToMapPoint( start, start );
        los->getMapNode()->getMap()->worldPointToMapPoint( end, end );

        los->setStart( start );
        los->setEnd( end );


    }
    traverse(node, nv);
}

/**********************************************************************/
RadialLineOfSightNode::RadialLineOfSightNode( MapNode* mapNode):
_mapNode( mapNode ),
_numSpokes(20),
_radius(500),
_center(0,0,0),
_goodColor(0.0f, 1.0f, 0.0f, 1.0f),
_badColor(1.0f, 0.0f, 0.0f, 1.0f),
_outlineColor( 1.0f, 1.0f, 1.0f, 1.0f),
_displayMode( MODE_SPLIT )
{
    compute();
}

double
RadialLineOfSightNode::getRadius() const
{
    return _radius;
}

void
RadialLineOfSightNode::setRadius(double radius)
{
    if (_radius != radius)
    {
        _radius = osg::clampAbove(radius, 1.0);
        compute();
    }
}

int
RadialLineOfSightNode::getNumSpokes() const
{
    return _numSpokes;
}

void RadialLineOfSightNode::setNumSpokes(int numSpokes)
{
    if (numSpokes != _numSpokes)
    {
        _numSpokes = osg::clampAbove(numSpokes, 1);
        compute();
    }
}

const osg::Vec3d&
RadialLineOfSightNode::getCenter() const
{
    return _center;
}

void
RadialLineOfSightNode::setCenter(const osg::Vec3d& center)
{
    if (_center != center)
    {
        _center = center;
        compute();
    }
}

void
RadialLineOfSightNode::compute()
{
    //Remove all the children
    removeChildren(0, getNumChildren());

    //Get the center point in geocentric
    osg::Vec3d centerWorld;
    _mapNode->getMap()->mapPointToWorldPoint( _center, centerWorld );

    osg::Vec3d up = centerWorld;
    up.normalize();

    //Get the "side" vector
    osg::Vec3d side = up ^ osg::Vec3d(0,0,1);

    //Get the number of spokes
    double delta = osg::PI * 2.0 / (double)_numSpokes;
    
    osg::Geometry* geometry = new osg::Geometry;
    osg::Vec3Array* verts = new osg::Vec3Array();
    verts->reserve(_numSpokes * 5);
    geometry->setVertexArray( verts );

    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->reserve( _numSpokes * 5 );

    geometry->setColorArray( colors );
    geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    osg::Vec3d previousEnd;
    osg::Vec3d firstEnd;

    osgSim::LineOfSight los;
    los.setDatabaseCacheReadCallback(0);

    for (unsigned int i = 0; i < _numSpokes; i++)
    {
        double angle = delta * (double)i;
        osg::Quat quat(angle, up );
        osg::Vec3d spoke = quat * (side * _radius);
        osg::Vec3d end = centerWorld + spoke;        
        los.addLOS( centerWorld, end);      
    }

    los.computeIntersections(_mapNode.get());

    for (unsigned int i = 0; i < _numSpokes; i++)
    {
        osg::Vec3d start = los.getStartPoint(i);
        osg::Vec3d end = los.getEndPoint(i);

        osgSim::LineOfSight::Intersections hits = los.getIntersections(i);
        osg::Vec3d hit;
        bool hasLOS = hits.empty();
        if (!hasLOS)
        {
            hit = *hits.begin();
        }

        if (hasLOS)
        {
            verts->push_back( start - centerWorld );
            verts->push_back( end - centerWorld );
            colors->push_back( _goodColor );
            colors->push_back( _goodColor );
        }
        else
        {
            if (_displayMode == MODE_SPLIT)
            {
                verts->push_back( start - centerWorld );
                verts->push_back( hit - centerWorld  );
                colors->push_back( _goodColor );
                colors->push_back( _goodColor );

                verts->push_back( hit - centerWorld );
                verts->push_back( end - centerWorld );
                colors->push_back( _badColor );
                colors->push_back( _badColor );
            }
            else if (_displayMode == MODE_SINGLE)
            {
                verts->push_back( start - centerWorld );
                verts->push_back( end - centerWorld );
                colors->push_back( _badColor );                                
                colors->push_back( _badColor );                
            }
        }


        if (i > 0)
        {
            verts->push_back( end - centerWorld );
            verts->push_back( previousEnd - centerWorld );
            colors->push_back( _outlineColor );
            colors->push_back( _outlineColor );
        }
        else
        {
            firstEnd = end;
        }

        previousEnd = end;
    }


    //Add the last outside of circle
    verts->push_back( firstEnd - centerWorld );
    verts->push_back( previousEnd - centerWorld );
    colors->push_back( osg::Vec4(1,1,1,1));
    colors->push_back( osg::Vec4(1,1,1,1));

    geometry->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, verts->size()));

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( geometry );

    getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    osg::MatrixTransform* mt = new osg::MatrixTransform;
    mt->setMatrix(osg::Matrixd::translate(centerWorld));
    mt->addChild(geode);


    addChild( mt );  
}

void
RadialLineOfSightNode::setGoodColor( const osg::Vec4f &color )
{
    if (_goodColor != color)
    {
        _goodColor = color;
        compute();
    }
}

const osg::Vec4f&
RadialLineOfSightNode::getGoodColor() const
{
    return _goodColor;
}

void
RadialLineOfSightNode::setBadColor( const osg::Vec4f &color )
{
    if (_badColor != color)
    {
        _badColor = color;
        compute();
    }
}

const osg::Vec4f&
RadialLineOfSightNode::getBadColor() const
{
    return _badColor;
}

void
RadialLineOfSightNode::setOutlineColor( const osg::Vec4f &color )
{
    if (_outlineColor != color)
    {
        _outlineColor = color;
        compute();
    }
}

const osg::Vec4f&
RadialLineOfSightNode::getOutlineColor() const
{
    return _outlineColor;
}

LOSDisplayMode
RadialLineOfSightNode::getDisplayMode() const
{
    return _displayMode;
}

void
RadialLineOfSightNode::setDisplayMode( LOSDisplayMode displayMode )
{
    if (_displayMode != displayMode)
    {
        _displayMode = displayMode;
        compute();
    }
}

/**********************************************************************/
RadialLineOfSightTether::RadialLineOfSightTether(osg::Node* node):
_node(node)
{
}

void 
RadialLineOfSightTether::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    if (nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
    {
        RadialLineOfSightNode* los = static_cast<RadialLineOfSightNode*>(node);

        osg::Vec3d center = getNodeCenter( _node );

        //Convert center to mappoint since that is what LOS expects
        los->getMapNode()->getMap()->worldPointToMapPoint( center, center );

        los->setCenter( center );      
    }
    traverse(node, nv);
}

/**********************************************************************/
class LOSDraggerCallback : public osgManipulator::DraggerCallback
{
public:
    LOSDraggerCallback(LineOfSightNode* los, bool start):
      _los(los),
          _start(start)
      {
          _ellipsoid = _los->getMapNode()->getMap()->getProfile()->getSRS()->getEllipsoid();
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
                  osg::Vec3d location = _start ? _los->getStart() : _los->getEnd();                  
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
                  if (_start)
                  {
                      _los->setStart( location );
                  }
                  else
                  {
                      _los->setEnd( location );
                  }

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
      LineOfSightNode* _los;
      bool _start;

      osg::Matrix _startMotionMatrix;

      osg::Matrix _localToWorld;
      osg::Matrix _worldToLocal;
};

/**********************************************************************/

LineOfSightEditor::LineOfSightEditor(LineOfSightNode* los):
_los(los)
{

    _startDragger  = new IntersectingDragger;
    _startDragger->setNode( _los->getMapNode() );    
    _startDragger->setHandleEvents( true );
    _startDragger->addDraggerCallback(new LOSDraggerCallback(_los, true ) );    
    _startDragger->setHeightAboveTerrain( 10 );
    _startDragger->setColor(osg::Vec4(0,0,1,0));
    _startDragger->setupDefaultGeometry();    
    addChild(_startDragger);

    _endDragger = new IntersectingDragger;
    _endDragger->setHeightAboveTerrain( 10 );
    _endDragger->setNode( _los->getMapNode() );    
    _endDragger->setHandleEvents( true );
    _endDragger->setColor(osg::Vec4(0,0,1,0));
    _endDragger->setupDefaultGeometry();
    _endDragger->addDraggerCallback(new LOSDraggerCallback(_los, false ) );

    addChild(_endDragger);

    updateDraggers();
}

void
LineOfSightEditor::updateDraggers()
{
    const osg::EllipsoidModel* em = _los->getMapNode()->getMap()->getProfile()->getSRS()->getEllipsoid();

    osg::Matrixd startMatrix;
    osg::Vec3d start = _los->getStart();        
    em->computeLocalToWorldTransformFromLatLongHeight(osg::DegreesToRadians(start.y()), osg::DegreesToRadians(start.x()), start.z(), startMatrix);    
    _startDragger->setMatrix(startMatrix);        

    osg::Matrixd endMatrix;
    osg::Vec3d end = _los->getEnd();        
    em->computeLocalToWorldTransformFromLatLongHeight(osg::DegreesToRadians(end.y()), osg::DegreesToRadians(end.x()), end.z(), endMatrix);    
    _endDragger->setMatrix(endMatrix);       
}



/*****************************************************************************/
//TODO:  Need to consolidate this and the regular LOS callback.  
class RadialLOSDraggerCallback : public osgManipulator::DraggerCallback
{
public:
    RadialLOSDraggerCallback(RadialLineOfSightNode* los):
      _los(los)
      {
          _ellipsoid = _los->getMapNode()->getMap()->getProfile()->getSRS()->getEllipsoid();
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
                  osg::Vec3d location = _los->getCenter();
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
                  _los->setCenter( location );
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
      RadialLineOfSightNode* _los;
      bool _start;

      osg::Matrix _startMotionMatrix;

      osg::Matrix _localToWorld;
      osg::Matrix _worldToLocal;
};

/**********************************************************************/

RadialLineOfSightEditor::RadialLineOfSightEditor(RadialLineOfSightNode* los):
_los(los)
{

    _dragger  = new IntersectingDragger;
    _dragger->setNode( _los->getMapNode() );    
    _dragger->setHandleEvents( true );
    _dragger->addDraggerCallback(new RadialLOSDraggerCallback(_los ) );    
    _dragger->setHeightAboveTerrain( 10 );
    _dragger->setColor(osg::Vec4(0,0,1,0));
    _dragger->setupDefaultGeometry();    
    addChild(_dragger);    
    updateDraggers();
}

void
RadialLineOfSightEditor::updateDraggers()
{
    const osg::EllipsoidModel* em = _los->getMapNode()->getMap()->getProfile()->getSRS()->getEllipsoid();

    osg::Matrixd matrix;
    osg::Vec3d center = _los->getCenter();        
    em->computeLocalToWorldTransformFromLatLongHeight(osg::DegreesToRadians(center.y()), osg::DegreesToRadians(center.x()), center.z(), matrix);    
    _dragger->setMatrix(matrix);        
}