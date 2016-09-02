/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include <osgEarthAnnotation/Draggers>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthAnnotation/GeoPositionNodeAutoScaler>

#include <osgEarth/MapNode>
#include <osgEarth/GeometryClamper>
#include <osgEarth/IntersectionPicker>

#include <osg/AutoTransform>
#include <osgViewer/View>

#include <osg/io_utils>

#include <osgGA/EventVisitor>

#include <osgManipulator/Dragger>

#define LC "[Dragger] "


using namespace osgEarth;
using namespace osgEarth::Annotation;

/**********************************************************/

Dragger::Dragger( MapNode* mapNode, int modKeyMask, const DragMode& defaultMode ):
GeoPositionNode( mapNode ),
_dragging(false),
_hovered(false),
_modKeyMask(modKeyMask),
_defaultMode(defaultMode),
_elevationDragging(false),
_verticalMinimum(0.0)
{    
    setNumChildrenRequiringEventTraversal( 1 );

    //_clampCallback = new ClampDraggerCallback( this );
    _projector = new osgManipulator::LineProjector;

    //setMapNode( mapNode );

    this->getOrCreateStateSet()->setRenderBinDetails(50, "DepthSortedBin");
}

Dragger::~Dragger()
{
    setMapNode( 0L );
}

bool Dragger::getDragging() const
{
    return _dragging;
}

bool Dragger::getHovered() const
{
    return _hovered;
}

void Dragger::setPosition(const GeoPoint& position)
{
    Dragger::setPosition( position, true );
}

void Dragger::setPosition(const GeoPoint& position, bool fireEvents)
{
    GeoPositionNode::setPosition( position );
    if ( fireEvents )
        firePositionChanged();
}

void Dragger::firePositionChanged()
{
    for( PositionChangedCallbackList::iterator i = _callbacks.begin(); i != _callbacks.end(); i++ )
    {
        i->get()->onPositionChanged(this, getPosition());
    }
}

void Dragger::enter()
{
}

void Dragger::leave()
{        
}    

void Dragger::addPositionChangedCallback( PositionChangedCallback* callback )
{
    _callbacks.push_back( callback );
}

void Dragger::removePositionChangedCallback( PositionChangedCallback* callback )
{
    PositionChangedCallbackList::iterator i = std::find( _callbacks.begin(), _callbacks.end(), callback);
    if (i != _callbacks.end())
    {
        _callbacks.erase( i );
    }    
}

void Dragger::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == osg::NodeVisitor::EVENT_VISITOR)
    {
        osgGA::EventVisitor* ev = static_cast<osgGA::EventVisitor*>(&nv);
        for(osgGA::EventQueue::Events::iterator itr = ev->getEvents().begin();
            itr != ev->getEvents().end();
            ++itr)
        {
            osgGA::GUIEventAdapter* ea = dynamic_cast<osgGA::GUIEventAdapter*>(itr->get());
            if ( ea && handle(*ea, *(ev->getActionAdapter())))
                ea->setHandled(true);
        }
    }
    GeoPositionNode::traverse( nv );
}

bool Dragger::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    if (ea.getHandled()) return false;

    osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
    if (!view) return false;
    if (!getMapNode()) return false;

    if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH)
    {
        IntersectionPicker picker( view, this );
        IntersectionPicker::Hits hits;

        if ( picker.pick( ea.getX(), ea.getY(), hits ) )
        {
            const GeoPoint& position = getPosition();

            _dragging = true;

            //Check for and handle vertical dragging if necessary
            bool pressedAlt = _modKeyMask && (ea.getModKeyMask() & _modKeyMask) > 0;
            _elevationDragging = (_defaultMode == Dragger::DRAGMODE_VERTICAL && !pressedAlt) || (_defaultMode == Dragger::DRAGMODE_HORIZONTAL && pressedAlt);

            if (_elevationDragging)
            {
              _pointer.reset();

              // set movement range
              // TODO: values 0.0 and 300000.0 are rather experimental
              GeoPoint posStart(position.getSRS(), position.x(), position.y(), 0.0, ALTMODE_ABSOLUTE);
              osg::Vec3d posStartXYZ;
              posStart.toWorld(posStartXYZ);

              GeoPoint posEnd(position.getSRS(), position.x(), position.y(), 300000.0, ALTMODE_ABSOLUTE);
              osg::Vec3d posEndXYZ;
              posEnd.toWorld(posEndXYZ);

              _projector->setLine(posStartXYZ, posEndXYZ);

              // set camera
              osgUtil::LineSegmentIntersector::Intersections intersections;
              osg::Node::NodeMask intersectionMask = 0xffffffff;
              osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
              if ( !view )
                  return true;

              if (view->computeIntersections(ea.getX(),ea.getY(),intersections, intersectionMask))
              {
                  for (osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin(); hitr != intersections.end(); ++hitr)
                  {
                      _pointer.addIntersection(hitr->nodePath, hitr->getLocalIntersectPoint());
                  }

                  bool draggerFound = false;
                  for (osgManipulator::PointerInfo::IntersectionList::iterator piit = _pointer._hitList.begin(); piit != _pointer._hitList.end(); ++piit)
                  {
                      for (osg::NodePath::iterator itr = piit->first.begin(); itr != piit->first.end(); ++itr)
                      {
                          Dragger* dragger = dynamic_cast<Dragger*>(*itr);
                          if (dragger==this)
                          {
                              draggerFound = true;
                              osg::Camera *rootCamera = view->getCamera();
                              osg::NodePath nodePath = _pointer._hitList.front().first;
                              osg::NodePath::reverse_iterator ritr;
                              for (ritr = nodePath.rbegin(); ritr != nodePath.rend(); ++ritr)
                              {
                                  osg::Camera* camera = dynamic_cast<osg::Camera*>(*ritr);
                                  if (camera && (camera->getReferenceFrame()!=osg::Transform::RELATIVE_RF || camera->getParents().empty()))
                                  {
                                       rootCamera = camera;
                                       break;
                                  }
                              }
                              _pointer.setCamera(rootCamera);
                              _pointer.setMousePosition(ea.getX(), ea.getY());

                              break;
                          }
                      }

                      if (draggerFound)
                        break;
                  }
              }
            }

            aa.requestRedraw();
            return true;
        }
    }
    else if (ea.getEventType() == osgGA::GUIEventAdapter::RELEASE)
    {
        _elevationDragging = false;

        if ( _dragging )
        {
            _dragging = false;
            firePositionChanged();
        }

        aa.requestRedraw();
    }
    else if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG)
    {
        if (_elevationDragging) 
        {
            _pointer._hitIter = _pointer._hitList.begin();
            _pointer.setMousePosition(ea.getX(), ea.getY());

            if (_projector->project(_pointer, _startProjectedPoint)) 
            {
                const GeoPoint& position = getPosition();

                //Get the absolute mapPoint that they've drug it to.
                GeoPoint projectedPos;
                projectedPos.fromWorld(position.getSRS(), _startProjectedPoint);

                // make sure point is not dragged down below
                // TODO: think of a better solution / HeightAboveTerrain performance issues?
                if (projectedPos.z() >= _verticalMinimum)
                {
                    //If the current position is relative, we need to convert the absolute world point to relative.
                    //If the point is absolute then just emit the absolute point.
                    if (position.altitudeMode() == ALTMODE_RELATIVE)
                    {
                        projectedPos.transformZ(ALTMODE_RELATIVE, getMapNode()->getTerrain());
                    }

                    setPosition( projectedPos );
                    aa.requestRedraw();
                }
            }

            return true;
        }
        
        if (_dragging)
        {
            osg::Vec3d world;
            if ( getMapNode() && getMapNode()->getTerrain()->getWorldCoordsUnderMouse(view, ea.getX(), ea.getY(), world) )
            {
                const GeoPoint& position = getPosition();

                //Get the absolute mapPoint that they've drug it to.
                GeoPoint mapPoint;
                mapPoint.fromWorld( getMapNode()->getMapSRS(), world );

                //If the current position is relative, we need to convert the absolute world point to relative.
                //If the point is absolute then just emit the absolute point.
                if (position.altitudeMode() == ALTMODE_RELATIVE)
                {
                    mapPoint.alt() = position.alt();
                    mapPoint.altitudeMode() = ALTMODE_RELATIVE;
                }

                setPosition( mapPoint );
                aa.requestRedraw();
                return true;
            }
        }
    }   
    else if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE)
    {
        IntersectionPicker picker( view, this );
        IntersectionPicker::Hits hits;

        if ( picker.pick( ea.getX(), ea.getY(), hits ) )
        {
            setHover( true );
        }
        else
        {
            setHover( false );
        }        
        aa.requestRedraw();
    }
    return false;
}

void Dragger::setHover( bool hovered)
{
    if (_hovered != hovered)
    {
        bool wasHovered = _hovered;
        _hovered = hovered;
        if (wasHovered)
        {
            leave();            
        }
        else
        {
            enter();
        }
    }
}

namespace
{
    struct DC : public osg::Drawable::DrawCallback
    {
        void drawImplementation(osg::RenderInfo& ri, const osg::Drawable* drawable) const {
            osg::Matrix mvm = ri.getState()->getModelViewMatrix();
            osg::Matrix pm = ri.getState()->getProjectionMatrix();
            double l, r, b, t, n, f;
            pm.getFrustum(l, r, b, t, n, f);            

            ri.getState()->applyModelViewMatrix( new osg::RefMatrix(pm) );            

            drawable->drawImplementation( ri );
        }
    };
}

/**********************************************************/

SphereDragger::SphereDragger(MapNode* mapNode):
Dragger(mapNode),
_pickColor(1.0f, 1.0f, 0.0f, 1.0f),
_color(0.0f, 1.0f, 0.0f, 1.0f),
_size( 5.0 )
{
    //Disable culling
    setCullingActive( false );

    //Build the handle
    osg::Sphere* shape = new osg::Sphere(osg::Vec3(0,0,0), _size);

    osg::Geode* geode = new osg::Geode();
    _shapeDrawable = new osg::ShapeDrawable( shape );    
    _shapeDrawable->setDataVariance( osg::Object::DYNAMIC );
    geode->addDrawable( _shapeDrawable );          

    geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    getPositionAttitudeTransform()->addChild( geode );

    this->addCullCallback( new GeoPositionNodeAutoScaler() );
    
    updateColor();
}

const osg::Vec4f& SphereDragger::getColor() const
{
    return _color;
}

void SphereDragger::setColor(const osg::Vec4f& color)
{
    if (_color != color)
    {
        _color = color;
        updateColor();
    }
}

const osg::Vec4f& SphereDragger::getPickColor() const
{
    return _pickColor;
}

void SphereDragger::setPickColor(const osg::Vec4f& pickColor)
{
    if (_pickColor != pickColor)
    {
        _pickColor = pickColor;
        updateColor();
    }
}

float SphereDragger::getSize() const
{
    return _size;
}

void SphereDragger::setSize(float size)
{
    if (_size != size)
    {
        _size = size;
        _shapeDrawable->setShape( new osg::Sphere(osg::Vec3f(0,0,0), _size) );
        _shapeDrawable->setColor( _color );
        //getPositionAttitudeTransform()->setScale(osg::Vec3d(_size,_size,_size));
    }
}

void SphereDragger::enter()
{
    updateColor();
}

void SphereDragger::leave()
{
    updateColor();
}

void SphereDragger::updateColor()
{
    if (getHovered())
    {
        _shapeDrawable->setColor( _pickColor );
    }        
    else
    {
        _shapeDrawable->setColor( _color );
    }
}

