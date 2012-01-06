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

#include <osg/Notify>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/MapNode>
#include <osgEarth/XmlUtils>
#include <osgEarth/Viewpoint>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/ObjectLocator>
#include <osgEarthAnnotation/Draggers>
#include <osg/io_utils>
#include <osgSim/LineOfSight>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Annotation;

class LineOfSightNode: public osg::Group
{
public:
    LineOfSightNode( osgEarth::MapNode* mapNode );
    
    LineOfSightNode( osgEarth::MapNode* mapNode, const osg::Vec3d& start, const osg::Vec3d& end );

    const osg::Vec3d& getStart() const;
    void setStart(const osg::Vec3& start);

    const osg::Vec3d& getEnd() const;
    void setEnd(const osg::Vec3& end);

    const osg::Vec3d& getHit() const;    
    bool getHasLOS() const;

    osgEarth::MapNode* getMapNode() const { return _mapNode.get(); }

private:
    void compute();
    void draw();
    osg::ref_ptr< osgEarth::MapNode > _mapNode;
    bool _hasLOS;
    osg::Vec3d _hit;
    osg::Vec3d _start;
    osg::Vec3d _end;
};

LineOfSightNode::LineOfSightNode(osgEarth::MapNode *mapNode):
_mapNode(mapNode),
_start(0,0,0),
_end(0,0,0),
_hit(0,0,0),
_hasLOS( true )
{
    compute();
}

LineOfSightNode::LineOfSightNode(osgEarth::MapNode *mapNode, const osg::Vec3d& start, const osg::Vec3d& end):
_mapNode(mapNode),
_start(start),
_end(end),
_hit(0,0,0),
_hasLOS( true )
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

    //_hasLOS = los.compute(_mapNode.get(), _hit);
    _mapNode->getMap()->worldPointToMapPoint( _hit, _hit);
    draw();
}

void
LineOfSightNode::draw()
{
    //Remove all children from this group
    removeChildren(0, getNumChildren());

    osg::Vec4 goodColor(0,1,0,1);
    osg::Vec4 badColor(1,0,0,1);
    
    //TODO:  Localize
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
        colors->push_back( goodColor );
        colors->push_back( goodColor );
    }
    else
    {
        /*
        verts->push_back( startWorld );
        verts->push_back( endWorld );
        colors->push_back( badColor );
        colors->push_back( badColor );
        */

        verts->push_back( startWorld - startWorld );
        colors->push_back( goodColor );
        verts->push_back( hitWorld   - startWorld );
        colors->push_back( goodColor );

        verts->push_back( hitWorld   - startWorld );
        colors->push_back( badColor );
        verts->push_back( endWorld   - startWorld );
        colors->push_back( badColor );
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

class LineOfSightTether : public osg::NodeCallback
{
public:
    LineOfSightTether(osg::Node* startNode, osg::Node* endNode);
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);  

private:
    osg::ref_ptr< osg::Node > _startNode;
    osg::ref_ptr< osg::Node > _endNode;
};

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

class RadialLineOfSightNode : public osg::Group
{
public:
    RadialLineOfSightNode( MapNode* mapNode );

    osgEarth::MapNode* getMapNode() { return _mapNode.get(); }

    void setRadius( double radius );
    double getRadius() const;

    void setNumSpokes( int numSpokes );
    int getNumSpokes() const;

    const osg::Vec3d& getCenter() const;
    void setCenter(const osg::Vec3d& center);

private:
    void compute();
    int _numSpokes;
    double _radius;
    osg::Vec3d _center;
    osg::ref_ptr< MapNode > _mapNode;
};


RadialLineOfSightNode::RadialLineOfSightNode( MapNode* mapNode):
_mapNode( mapNode ),
_numSpokes(20),
_radius(500),
_center(0,0,0)
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

    //TODO:  Localize
    osg::Geometry* geometry = new osg::Geometry;
    osg::Vec3Array* verts = new osg::Vec3Array();
    verts->reserve(_numSpokes * 2);
    geometry->setVertexArray( verts );

    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->reserve( _numSpokes * 2 );
    
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
            colors->push_back( osg::Vec4(0,1,0,1));
            colors->push_back( osg::Vec4(0,1,0,1));
        }
        else
        {
            verts->push_back( start - centerWorld );
            verts->push_back( hit - centerWorld  );
            colors->push_back( osg::Vec4(0,1,0,1));
            colors->push_back( osg::Vec4(0,1,0,1));

            verts->push_back( hit - centerWorld );
            verts->push_back( end - centerWorld );
            colors->push_back( osg::Vec4(1,0,0,1));
            colors->push_back( osg::Vec4(1,0,0,1));

            //colors->push_back( osg::Vec4(1,0,0,1));
            //colors->push_back( osg::Vec4(1,0,0,1));
        }


        if (i > 0)
        {
            verts->push_back( end - centerWorld );
            verts->push_back( previousEnd - centerWorld );
            colors->push_back( osg::Vec4(1,1,1,1));
            colors->push_back( osg::Vec4(1,1,1,1));
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



class RadialLineOfSightTether : public osg::NodeCallback
{
public:
    RadialLineOfSightTether(osg::Node* node);
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);  

private:
    osg::ref_ptr< osg::Node > _node;
};

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




class LineOfSightEditor : public osg::Group
{
public:
    LineOfSightEditor(LineOfSightNode* los);    

    void updateDraggers();
private:
    double _startHAT;
    double _endHAT;
    osg::ref_ptr< LineOfSightNode > _los;
    IntersectingDragger* _startDragger;
    IntersectingDragger* _endDragger;
};


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


LineOfSightEditor::LineOfSightEditor(LineOfSightNode* los):
_los(los)
{

    _startDragger  = new IntersectingDragger;
    _startDragger->setNode( _los->getMapNode() );
    _startDragger->setupDefaultGeometry();    
    _startDragger->setHandleEvents( true );
    _startDragger->addDraggerCallback(new LOSDraggerCallback(_los, true ) );    
    _startDragger->setHeightAboveTerrain( 10 );
    addChild(_startDragger);

    _endDragger = new IntersectingDragger;
    _endDragger->setHeightAboveTerrain( 10 );
    _endDragger->setNode( _los->getMapNode() );
    _endDragger->setupDefaultGeometry();
    _endDragger->setHandleEvents( true );
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


struct LOSHandler : public osgGA::GUIEventHandler 
{
    LOSHandler(LineOfSightNode* los, MapNode* mapNode, LineOfSightEditor* editor)
        : _los( los ),
        _mapNode( mapNode ),
        _editor( editor )
    {        
    }

    bool getHit( float x, float y, osgViewer::View* view, osg::Vec3d& hit )
    {
        osgUtil::LineSegmentIntersector::Intersections results;
        if ( view->computeIntersections( x, y, results ) )
        {
            // find the first hit under the mouse:
            osgUtil::LineSegmentIntersector::Intersection first = *(results.begin());
            hit = first.getWorldIntersectPoint();            
            return true;
        }
        return false;
    }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if ( ea.getEventType() == osgGA::GUIEventAdapter::PUSH )
        {
            osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
            osg::Vec3d hit;
            if (getHit( ea.getX(), ea.getY(), view, hit ))
            {
                if (ea.getButtonMask() == osgGA::GUIEventAdapter::MouseButtonMask::LEFT_MOUSE_BUTTON)
                {
                    Map* map = _mapNode->getMap();
                    osg::Vec3d lla;
                    map->worldPointToMapPoint( hit, lla );                    
                    //lla.z() += _hat;
                    //map->mapPointToWorldPoint( lla, hit );
                    if (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL)
                    {
                        OE_NOTICE << "Setting start " << lla << std::endl;
                        _los->setStart( lla );
                    }
                    else if (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_SHIFT)
                    {
                        OE_NOTICE << "Setting end" << lla << std::endl;
                        _los->setEnd( lla );
                    }
                }
            }
            _editor->updateDraggers();
        }
        else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
        {
            double delta = 100.0;
            if (ea.getKey() == 'u')
            {              
                _los->setStart( _los->getStart() + osg::Vec3d(0,0,delta) );
                _editor->updateDraggers();            
            }
            else if (ea.getKey() == 'U')
            {
                _los->setEnd( _los->getEnd() + osg::Vec3d(0,0,delta) );
                _editor->updateDraggers();
            }
            else if (ea.getKey() == 'j')
            {                
                _los->setStart( _los->getStart() + osg::Vec3d(0,0,-delta) );
                _editor->updateDraggers();            
            }
            else if (ea.getKey() == 'J')
            {
                _los->setEnd( _los->getEnd() + osg::Vec3d(0,0,-delta) );                
                _editor->updateDraggers();            
            }
        }
        return false;
    }    

    LineOfSightNode* _los;
    LineOfSightEditor* _editor;
    MapNode* _mapNode;
};


struct RadialLOSHandler : public osgGA::GUIEventHandler 
{
    RadialLOSHandler(RadialLineOfSightNode* los, MapNode* mapNode)
        : _los( los ),
        _mapNode( mapNode ),
        _hat(5)
    {        
    }

    bool getHit( float x, float y, osgViewer::View* view, osg::Vec3d& hit )
    {
        osgUtil::LineSegmentIntersector::Intersections results;
        if ( view->computeIntersections( x, y, results ) )
        {
            // find the first hit under the mouse:
            osgUtil::LineSegmentIntersector::Intersection first = *(results.begin());
            hit = first.getWorldIntersectPoint();            
            return true;
        }
        return false;
    }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        //if ( ea.getEventType() == osgGA::GUIEventAdapter::PUSH )
        if ( ea.getEventType() == osgGA::GUIEventAdapter::MOVE )
        {
            osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
            osg::Vec3d hit;
            if (getHit( ea.getX(), ea.getY(), view, hit ))
            {
                //if (ea.getButtonMask() == osgGA::GUIEventAdapter::MouseButtonMask::MIDDLE_MOUSE_BUTTON)
                if (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL)
                {
                    Map* map = _mapNode->getMap();
                    osg::Vec3d lla;
                    map->worldPointToMapPoint( hit, lla );                    
                    _los->setCenter( lla + osg::Vec3d(0,0,_hat) );            
                    OE_NOTICE << "Setting center to " << _los->getCenter() << std::endl;
                }
            }
        }
        else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
        {
            double radiusDelta = 100.0;
            int spokeDelta = 10;
            if (ea.getKey() == 'u')
            {              
                _los->setRadius( _los->getRadius() + radiusDelta );
            }
            else if (ea.getKey() == 'U')
            {                
                _los->setRadius( _los->getRadius() - radiusDelta );
            }
            else if (ea.getKey() == 'j')
            {
                _los->setNumSpokes( _los->getNumSpokes() + spokeDelta );
            }
            else if (ea.getKey() == 'J')
            {
                _los->setNumSpokes( _los->getNumSpokes() - spokeDelta );
            }
            else if (ea.getKey() == 'k')
            {
                _los->setCenter( _los->getCenter() + osg::Vec3d(0,0,10));
            }
            else if (ea.getKey() == 'K')
            {
                _los->setCenter( _los->getCenter() + osg::Vec3d(0,0,-10));
            }

        }
        return false;
    }    

    RadialLineOfSightNode* _los;    
    MapNode* _mapNode;
    double _hat;
};


osg::AnimationPath* createAnimationPath( MapNode* mapNode, const osg::Vec3& center, float radius,double looptime)
{
    // set up the animation path 
    osg::AnimationPath* animationPath = new osg::AnimationPath;
    animationPath->setLoopMode(osg::AnimationPath::LOOP);
    
    int numSamples = 40;

    double delta = osg::PI * 2.0 / (double)numSamples;

    //Get the center point in geocentric
    osg::Vec3d centerWorld;
    mapNode->getMap()->mapPointToWorldPoint( center, centerWorld );

    osg::Vec3d up = centerWorld;
    up.normalize();

    //Get the "side" vector
    osg::Vec3d side = up ^ osg::Vec3d(0,0,1);


    double time=0.0f;
    double time_delta = looptime/(double)numSamples;

    osg::Vec3d firstPosition;
    osg::Quat firstRotation;

    for (unsigned int i = 0; i < numSamples; i++)
    {
        double angle = delta * (double)i;
        osg::Quat quat(angle, up );
        osg::Vec3d spoke = quat * (side * radius);
        osg::Vec3d end = centerWorld + spoke;                

        osg::Quat makeUp;
        makeUp.makeRotate(osg::Vec3d(0,0,1), up);
        
        animationPath->insert(time,osg::AnimationPath::ControlPoint(end,makeUp));
        if (i == 0)
        {
            firstPosition = end;
            firstRotation = makeUp;
        }
        time += time_delta;            
    }
   
    animationPath->insert(time, osg::AnimationPath::ControlPoint(firstPosition, firstRotation));

    return animationPath;    
}

osg::Node* createPlane(osg::Node* node, MapNode* mapNode, const osg::Vec3d& center, double radius, double time)
{
    osg::MatrixTransform* positioner = new osg::MatrixTransform;
    positioner->addChild( node);
    osg::AnimationPath* animationPath = createAnimationPath(mapNode, center, radius, time);
    positioner->setUpdateCallback( new osg::AnimationPathCallback(animationPath, 0.0, 1.0));
    return positioner;
}



int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);

    // load the .earth file from the command line.
    osg::Node* earthNode = osgDB::readNodeFiles( arguments );
    if (!earthNode)
    {
        OE_NOTICE << "Unable to load earth model" << std::endl;
        return 1;
    }

    osg::Group* root = new osg::Group();

    osgEarth::MapNode * mapNode = osgEarth::MapNode::findMapNode( earthNode );
    if (!mapNode)
    {
        OE_NOTICE << "Could not find MapNode " << std::endl;
        return 1;
    }

    osgEarth::Util::EarthManipulator* manip = new EarthManipulator();    
    viewer.setCameraManipulator( manip );
    
    root->addChild( earthNode );    
    viewer.getCamera()->addCullCallback( new AutoClipPlaneCullCallback(mapNode->getMap()) );

    LineOfSightNode* los = new LineOfSightNode( mapNode, osg::Vec3d(-121.656, 46.0935, 4133.06), osg::Vec3d(-121.466, 46.1576, 6304.91));
    root->addChild( los );
    LineOfSightEditor* editor = new LineOfSightEditor( los );
    root->addChild( editor );

    RadialLineOfSightNode* radial = new RadialLineOfSightNode( mapNode );
    radial->setCenter( osg::Vec3d(-121.656, 46.0935, 4133.06) );
    radial->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    viewer.addEventHandler( new RadialLOSHandler( radial, mapNode) );
    root->addChild( radial );

    osg::ref_ptr< osg::Node >  plane = osgDB::readNodeFile("cessna.osgt.50,50,50.scale");
    osg::Node* plane1 = createPlane(plane, mapNode, osg::Vec3d(-121.656, 46.0935, 4133.06), 5000, 20);
    osg::Node* plane2 = createPlane(plane, mapNode, osg::Vec3d(-121.321, 46.2589, 1390.09), 3000, 5);
    root->addChild( plane1  );
    root->addChild( plane2 );
    los->setUpdateCallback( new LineOfSightTether( plane1, plane2 ) );


    osg::Node* plane3 = createPlane(plane, mapNode, osg::Vec3d( -121.463, 46.3548, 1348.71), 10000, 5);    
    root->addChild( plane3 );
    RadialLineOfSightNode* tetheredRadial = new RadialLineOfSightNode( mapNode );    
    tetheredRadial->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);    
    tetheredRadial->setRadius( 5000 );
    tetheredRadial->setNumSpokes( 100 );
    root->addChild( tetheredRadial );
    tetheredRadial->setUpdateCallback( new RadialLineOfSightTether( plane3 ) );

    


    //viewer.addEventHandler( new LOSHandler( los, mapNode, editor ) );
      

    // osgEarth benefits from pre-compilation of GL objects in the pager. In newer versions of
    // OSG, this activates OSG's IncrementalCompileOpeartion in order to avoid frame breaks.
    viewer.getDatabasePager()->setDoPreCompile( true );

    viewer.setSceneData( root );    

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
