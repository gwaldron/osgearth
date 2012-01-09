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
#include <osgEarthAnnotation/Draggers>
#include <osg/AutoTransform>
#include <osgViewer/View>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>
#include <osg/io_utils>

using namespace osgEarth;
using namespace osgEarth::Annotation;

/**********************************************************/
TranslateCommand::TranslateCommand():
osgManipulator::MotionCommand()
{
}

osgManipulator::MotionCommand*
TranslateCommand::createCommandInverse()
{
    TranslateCommand *cmd = new TranslateCommand();
    cmd->setTranslation( -_translation );
    return cmd;
}

osg::Matrix
TranslateCommand::getMotionMatrix() const 
{
    return osg::Matrixd::translate(_translation);
}


TranslateCommand::~TranslateCommand()
{
}

/**********************************************************/
    

IntersectingDragger::IntersectingDragger():
osgManipulator::Dragger(),
_size(5.0f)
{
    setColor(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    setPickColor(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f));
}

IntersectingDragger::~IntersectingDragger()
{
}

void
IntersectingDragger::setupDefaultGeometry()
{
    //Build the handle
    osg::Sphere* shape = new osg::Sphere(osg::Vec3(0,0,0), _size);   
    osg::Geode* geode = new osg::Geode();
    _shapeDrawable = new osg::ShapeDrawable( shape );    
    geode->addDrawable( _shapeDrawable.get() );
    setDrawableColor( _color );

    geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    osg::AutoTransform* at = new osg::AutoTransform;
    at->setAutoScaleToScreen( true );
    at->addChild( geode );
    addChild( at );
}

bool
IntersectingDragger::getHit(const osg::Vec3d& start, const osg::Vec3d &end, osg::Vec3d& intersection)
{
    if (_node.valid())
    {
        osg::ref_ptr<osgUtil::LineSegmentIntersector> lsi = new osgUtil::LineSegmentIntersector(start,end);

        osgUtil::IntersectionVisitor iv(lsi.get());

        _node->accept(iv);

        if (lsi->containsIntersections())
        {
            OE_DEBUG << "Got get hit at " << start << " to " << end << std::endl;
            intersection = lsi->getIntersections().begin()->getWorldIntersectPoint();
            osg::Vec3d normal = intersection;
            normal.normalize();
            intersection += normal * _heightAboveTerrain;
            return true;
        }
    }

    OE_DEBUG << "Warning:  Couldn't get hit at " << start << " to " << end << std::endl;
    return false;
}

void
IntersectingDragger::setDrawableColor(const osg::Vec4f& color)
{
    if (_shapeDrawable.valid()) _shapeDrawable->setColor( color );
};

void
IntersectingDragger::setHeightAboveTerrain( double hat )
{
    _heightAboveTerrain = hat;
}

double
IntersectingDragger::getHeightAboveTerrain() const
{
    return _heightAboveTerrain;
}

bool IntersectingDragger::handle(const osgManipulator::PointerInfo& pointer, const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    // Check if the dragger node is in the nodepath.
    //if (!pointer.contains(this)) return false; 


    switch (ea.getEventType())
    {
        // Pick start.
        case (osgGA::GUIEventAdapter::PUSH):
            {
                osg::NodePath nodePathToRoot;
                computeNodePathToRoot(*this,nodePathToRoot);
                _localToWorld = osg::computeLocalToWorld(nodePathToRoot);
                _worldToLocal.invert( _localToWorld );

                osg::Vec3d near, far, hit;
                pointer.getNearFarPoints(near, far);
                if (getHit(near, far, hit))
                {
                    _startPoint = hit;
                   
                    // Generate the motion command.
                    osg::ref_ptr<TranslateCommand> cmd = new TranslateCommand();
                    cmd->setTranslation(osg::Vec3d(0,0,0));
                    cmd->setStage(osgManipulator::MotionCommand::START);
                    cmd->setLocalToWorldAndWorldToLocal(_localToWorld, _worldToLocal);

                    // Dispatch command.
                    dispatch(*cmd);

                    cmd = new TranslateCommand();
                    hit = hit * _worldToLocal;
                    osg::Vec3d start = getMatrix().getTrans() * _worldToLocal;                    
                    osg::Vec3d translation = hit - start;
                    cmd->setTranslation(translation);
                    cmd->setStage(osgManipulator::MotionCommand::MOVE);
                    cmd->setLocalToWorldAndWorldToLocal(_localToWorld, _worldToLocal);

                    // Dispatch command.
                    dispatch(*cmd);

                    setDrawableColor( _pickColor );

                    aa.requestRedraw();
                }                 
                return true; 
            }
            
        // Pick move.
        case (osgGA::GUIEventAdapter::DRAG):
            {                
                osg::Vec3d near, far, hit;
                pointer.getNearFarPoints(near, far);
                if (getHit(near, far, hit))
                {
                    // Generate the motion command.
                    osg::ref_ptr<TranslateCommand> cmd = new TranslateCommand();
                    hit = hit * _worldToLocal;
                    osg::Vec3d start = _startPoint * _worldToLocal;
                    osg::Vec3d translation = hit - start;
                    cmd->setTranslation(translation);
                    cmd->setStage(osgManipulator::MotionCommand::MOVE);
                    cmd->setLocalToWorldAndWorldToLocal(_localToWorld, _worldToLocal);

                    // Dispatch command.
                    dispatch(*cmd);

                    aa.requestRedraw();
                }           
                return true; 
            }
            
        // Pick finish.
        case (osgGA::GUIEventAdapter::RELEASE):
            {             
                osg::ref_ptr<TranslateCommand> cmd = new TranslateCommand();
                cmd->setStage(osgManipulator::MotionCommand::FINISH);
                cmd->setLocalToWorldAndWorldToLocal(_localToWorld, _worldToLocal);

                // Dispatch command.
                dispatch(*cmd);

                // Reset color.
                setDrawableColor(_color);

                aa.requestRedraw();

                return true;
            }
        default:
            return false;
    }
}

bool IntersectingDragger::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    //This is essentialy the same as the original Dragger handle code except for it checks for "all" intersections and not just the first one.
    //This allows you to turn depth testing off and have control points that might be obstructed by a mountain still be clickable.
    if (ea.getHandled()) return false;

    osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
    if (!view) return false;

    bool handled = false;

    bool activationPermitted = true;
    if (_activationModKeyMask!=0 || _activationKeyEvent!=0)
    {
        _activationPermittedByModKeyMask = (_activationModKeyMask!=0) ?
            ((ea.getModKeyMask() & _activationModKeyMask)!=0) :
            false;

        if (_activationKeyEvent!=0)
        {
            switch (ea.getEventType())
            {
                case osgGA::GUIEventAdapter::KEYDOWN:
                {
                    if (ea.getKey()==_activationKeyEvent) _activationPermittedByKeyEvent = true;
                    break;
                }
                case osgGA::GUIEventAdapter::KEYUP:
                {
                    if (ea.getKey()==_activationKeyEvent) _activationPermittedByKeyEvent = false;
                    break;
                }
                default:
                    break;
            }
        }

        activationPermitted =  _activationPermittedByModKeyMask || _activationPermittedByKeyEvent;

    }

    if (activationPermitted || _draggerActive)
    {
        switch (ea.getEventType())
        {
            case osgGA::GUIEventAdapter::PUSH:
            {
                osgUtil::LineSegmentIntersector::Intersections intersections;

                _pointer.reset();

                if (view->computeIntersections(ea.getX(),ea.getY(),intersections))
                {
                    for(osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
                        hitr != intersections.end();
                        ++hitr)
                    {
                        _pointer.addIntersection(hitr->nodePath, hitr->getLocalIntersectPoint());
                    }
                    
                    for(osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
                        hitr != intersections.end();
                        ++hitr)
                    {
                        for (osg::NodePath::const_iterator itr = hitr->nodePath.begin();
                            itr != hitr->nodePath.end();
                            ++itr)
                        {
                            osgManipulator::Dragger* dragger = dynamic_cast<osgManipulator::Dragger*>(*itr);
                            if (dragger)
                            {
                                if (dragger==this)
                                {
                                    osg::Camera *rootCamera = view->getCamera();
                                    osg::NodePath nodePath = _pointer._hitList.front().first;
                                    osg::NodePath::reverse_iterator ritr;
                                    for(ritr = nodePath.rbegin();
                                        ritr != nodePath.rend();
                                        ++ritr)
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

                                    dragger->handle(_pointer, ea, aa);
                                    dragger->setDraggerActive(true);
                                    handled = true;
                                }
                            }
                        }
                    }
                }
            }
            case osgGA::GUIEventAdapter::DRAG:
            case osgGA::GUIEventAdapter::RELEASE:
            {
                if (_draggerActive)
                {
                    _pointer._hitIter = _pointer._hitList.begin();
//                    _pointer.setCamera(view->getCamera());
                    _pointer.setMousePosition(ea.getX(), ea.getY());

                    handle(_pointer, ea, aa);

                    handled = true;
                }
                break;
            }
            default:
                break;
        }

        if (_draggerActive && ea.getEventType() == osgGA::GUIEventAdapter::RELEASE)
        {
            setDraggerActive(false);
            _pointer.reset();
        }
    }

    return handled;
}
