/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarthUtil/EarthManipulator>
#include <osg/Quat>
#include <osg/Notify>
#include <osgUtil/LineSegmentIntersector>

using namespace osgEarthUtil;

EarthManipulator::EarthManipulator()
{
    _distance = 1.0;
    _thrown = false;
}


EarthManipulator::~EarthManipulator()
{
    //NOP
}

void EarthManipulator::setNode(osg::Node* node)
{
    _node = node;

    if (_node.get())
    {
        const osg::BoundingSphere& boundingSphere=_node->getBound();
        const float minimumDistanceScale = 0.001f;
        _minimumDistance = osg::clampBetween(
            float(boundingSphere._radius) * minimumDistanceScale,
            0.00001f,1.0f);
    }
    if (getAutoComputeHomePosition()) computeHomePosition();
}

const osg::Node*
EarthManipulator::getNode() const
{
    return _node.get();
}

osg::Node*
EarthManipulator::getNode()
{
    return _node.get();
}

bool
EarthManipulator::intersect(const osg::Vec3d& start, const osg::Vec3d& end, osg::Vec3d& intersection) const
{
    osg::ref_ptr<osgUtil::LineSegmentIntersector> lsi = new osgUtil::LineSegmentIntersector(start,end);

    osgUtil::IntersectionVisitor iv(lsi.get());
    iv.setTraversalMask(_intersectTraversalMask);
    
    _node->accept(iv);
    
    if (lsi->containsIntersections())
    {
        intersection = lsi->getIntersections().begin()->getWorldIntersectPoint();
        return true;
    }
    return false;
}

void
EarthManipulator::home(const osgGA::GUIEventAdapter& ,osgGA::GUIActionAdapter& us)
{
    if (getAutoComputeHomePosition()) computeHomePosition();

    computePosition(_homeEye, _homeCenter, _homeUp);
    us.requestRedraw();
}

void
EarthManipulator::init(const osgGA::GUIEventAdapter& ,osgGA::GUIActionAdapter& )
{
    flushMouseEventStack();
}


void
EarthManipulator::getUsage(osg::ApplicationUsage& usage) const
{
}

bool
EarthManipulator::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& us)
{
    switch(ea.getEventType())
    {
        case(osgGA::GUIEventAdapter::FRAME):
            if (_thrown)
            {
                if (calcMovement()) us.requestRedraw();
            }
            return false;
        default:
            break;
    }

    if (ea.getHandled()) return false;

    switch(ea.getEventType())
    {
        case(osgGA::GUIEventAdapter::PUSH):
        {
            flushMouseEventStack();
            addMouseEvent(ea);
            if (calcMovement()) us.requestRedraw();
            us.requestContinuousUpdate(false);
            _thrown = false;
            return true;
        }

        case(osgGA::GUIEventAdapter::RELEASE):
        {
            if (ea.getButtonMask()==0)
            {

                if (isMouseMoving())
                {
                    if (calcMovement())
                    {
                        us.requestRedraw();
                        us.requestContinuousUpdate(true);
                        _thrown = true;
                    }
                }
                else
                {
                    flushMouseEventStack();
                    addMouseEvent(ea);
                    if (calcMovement()) us.requestRedraw();
                    us.requestContinuousUpdate(false);
                    _thrown = false;
                }

            }
            else
            {
                flushMouseEventStack();
                addMouseEvent(ea);
                if (calcMovement()) us.requestRedraw();
                us.requestContinuousUpdate(false);
                _thrown = false;
            }
            return true;
        }

        case(osgGA::GUIEventAdapter::DRAG):
        {
            addMouseEvent(ea);
            if (calcMovement()) us.requestRedraw();
            us.requestContinuousUpdate(false);
            _thrown = false;
            return true;
        }

        case(osgGA::GUIEventAdapter::MOVE):
        {
            return false;
        }

        case(osgGA::GUIEventAdapter::KEYDOWN):
            if (ea.getKey()== osgGA::GUIEventAdapter::KEY_Space)
            {
                flushMouseEventStack();
                _thrown = false;
                home(ea,us);
                us.requestRedraw();
                us.requestContinuousUpdate(false);
                return true;
            }
            return false;
        default:
            return false;
    }
}


bool
EarthManipulator::isMouseMoving()
{
    if (_ga_t0.get()==NULL || _ga_t1.get()==NULL) return false;

    static const float velocity = 0.1f;

    float dx = _ga_t0->getXnormalized()-_ga_t1->getXnormalized();
    float dy = _ga_t0->getYnormalized()-_ga_t1->getYnormalized();
    float len = sqrtf(dx*dx+dy*dy);
    float dt = _ga_t0->getTime()-_ga_t1->getTime();

    return (len>dt*velocity);
}


void
EarthManipulator::flushMouseEventStack()
{
    _ga_t1 = NULL;
    _ga_t0 = NULL;
}


void
EarthManipulator::addMouseEvent(const osgGA::GUIEventAdapter& ea)
{
    _ga_t1 = _ga_t0;
    _ga_t0 = &ea;
}

void
EarthManipulator::setByMatrix(const osg::Matrixd& matrix)
{
    osg::Vec3d lookVector(- matrix(2,0),-matrix(2,1),-matrix(2,2));
    osg::Vec3d eye(matrix(3,0),matrix(3,1),matrix(3,2));

    if (!_node)
    {
        _center = eye+ lookVector;
        _distance = lookVector.length();
        _rotation = matrix.getRotate();
        return;
    }


    // need to reintersect with the terrain
    const osg::BoundingSphere& bs = _node->getBound();
    float distance = (eye-bs.center()).length() + _node->getBound().radius();
    osg::Vec3d start_segment = eye;
    osg::Vec3d end_segment = eye + lookVector*distance;
    
    osg::Vec3d ip;
    bool hitFound = false;
    if (intersect(start_segment, end_segment, ip))
    {
        _center = ip;
        _distance = (eye-ip).length();

        osg::Matrixd rotation_matrix = osg::Matrixd::translate(0.0,0.0,-_distance)*
                                       matrix*
                                       osg::Matrixd::translate(-_center);

        _rotation = rotation_matrix.getRotate();
        hitFound = true;
    }

    if (!hitFound)
    {
        osg::CoordinateFrame eyePointCoordFrame = getCoordinateFrame( eye );

        if (intersect(eye+getUpVector(eyePointCoordFrame)*distance,
                      eye-getUpVector(eyePointCoordFrame)*distance,
                      ip))
        {
            _center = ip;

            _distance = (eye-ip).length();

            _rotation.set(0,0,0,1);

            hitFound = true;
        }
    }


    osg::CoordinateFrame coordinateFrame = getCoordinateFrame(_center);
    _previousUp = getUpVector(coordinateFrame);

    clampOrientation();
}

osg::Matrixd
EarthManipulator::getMatrix() const
{
    return osg::Matrixd::translate(0.0,0.0,_distance)*osg::Matrixd::rotate(_rotation)*osg::Matrixd::translate(_center);
}

osg::Matrixd
EarthManipulator::getInverseMatrix() const
{
    return osg::Matrixd::translate(-_center)*osg::Matrixd::rotate(_rotation.inverse())*osg::Matrixd::translate(0.0,0.0,-_distance);
}

void
EarthManipulator::computePosition(const osg::Vec3d& eye,const osg::Vec3d& center,const osg::Vec3d& up)
{
    if (!_node) return;

    // compute rotation matrix
    osg::Vec3d lv(center-eye);
    _distance = lv.length();
    _center = center;

    if (_node.valid())
    {
        bool hitFound = false;

        double distance = lv.length();
        double maxDistance = distance+2*(eye-_node->getBound().center()).length();
        osg::Vec3d farPosition = eye+lv*(maxDistance/distance);
        osg::Vec3d endPoint = center;
        for(int i=0;
            !hitFound && i<2;
            ++i, endPoint = farPosition)
        {
            // compute the intersection with the scene.
            
            osg::Vec3d ip;
            if (intersect(eye, endPoint, ip))
            {
                _center = ip;
                _distance = (ip-eye).length();

                hitFound = true;
            }
        }
    }

    // note LookAt = inv(CF)*inv(RM)*inv(T) which is equivalent to:
    // inv(R) = CF*LookAt.

    osg::Matrixd rotation_matrix = osg::Matrixd::lookAt(eye,center,up);

    _rotation = rotation_matrix.getRotate().inverse();

    osg::CoordinateFrame coordinateFrame = getCoordinateFrame(_center);
    _previousUp = getUpVector(coordinateFrame);

    clampOrientation();
}


bool
EarthManipulator::calcMovement()
{
    // return if less then two events have been added.
    if (_ga_t0.get()==NULL || _ga_t1.get()==NULL) return false;

    double dx = _ga_t0->getXnormalized()-_ga_t1->getXnormalized();
    double dy = _ga_t0->getYnormalized()-_ga_t1->getYnormalized();


    // return if there is no movement.
    if (dx==0 && dy==0) return false;

    unsigned int buttonMask = _ga_t1->getButtonMask();
    
    if (buttonMask==osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON ||
        buttonMask==(osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON|osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON))
    {
        osg::Matrix rotation_matrix;
        rotation_matrix.makeRotate(_rotation);

        osg::Vec3d lookVector = -getUpVector(rotation_matrix);
        osg::Vec3d sideVector = getSideVector(rotation_matrix);
        osg::Vec3d upVector = getFrontVector(rotation_matrix);

        osg::CoordinateFrame coordinateFrame = getCoordinateFrame(_center);
        osg::Vec3d localUp = getUpVector(coordinateFrame);
        //osg::Vec3d localUp = _previousUp;


        osg::Vec3d forwardVector = localUp^sideVector;
        sideVector = forwardVector^localUp;

        forwardVector.normalize();
        sideVector.normalize();

        osg::Quat rotate_elevation;
        rotate_elevation.makeRotate(dy,sideVector);

        osg::Quat rotate_azim;
        rotate_azim.makeRotate(-dx,localUp);

        _rotation = _rotation * rotate_elevation * rotate_azim;

        return true;
    }
    else if (buttonMask==osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)     
    {

        // pan model.
        double scale = -0.3f*_distance;

        osg::Matrixd rotation_matrix;
        rotation_matrix.makeRotate(_rotation);


        // compute look vector.
        osg::Vec3d lookVector = -getUpVector(rotation_matrix);
        osg::Vec3d sideVector = getSideVector(rotation_matrix);
        osg::Vec3d upVector = getFrontVector(rotation_matrix);

        // CoordinateFrame coordinateFrame = getCoordinateFrame(_center);
        // osg::Vec3d localUp = getUpVector(coordinateFrame);
        osg::Vec3d localUp = _previousUp;

        osg::Vec3d forwardVector =localUp^sideVector;
        sideVector = forwardVector^localUp;

        forwardVector.normalize();
        sideVector.normalize();

        osg::Vec3d dv = forwardVector * (dy*scale) + sideVector * (dx*scale);

        _center += dv;

        // need to recompute the intersection point along the look vector.

        bool hitFound = false;

        if (_node.valid())
        {

            // now reorientate the coordinate frame to the frame coords.
            osg::CoordinateFrame coordinateFrame =  getCoordinateFrame(_center);

            // need to reintersect with the terrain
            double distance = _node->getBound().radius()*0.25f;
            
            osg::Vec3d ip1;
            osg::Vec3d ip2;
            bool hit_ip1 = intersect(_center, _center + getUpVector(coordinateFrame) * distance, ip1);
            bool hit_ip2 = intersect(_center, _center - getUpVector(coordinateFrame) * distance, ip2);
            if (hit_ip1)
            {
                if (hit_ip2)
                {
                    _center = (_center-ip1).length2() < (_center-ip2).length2() ?
                                ip1 :
                                ip2;
                                
                    hitFound = true;
                }
                else
                {
                    _center = ip1;
                    hitFound = true;
                }
            }
            else if (hit_ip2)
            {
                _center = ip2;
                hitFound = true;
            }
            
            if (!hitFound)
            {
                // ??
                osg::notify(osg::INFO)<<"EarthManipulator unable to intersect with terrain."<<std::endl;
            }

            coordinateFrame = getCoordinateFrame(_center);
            osg::Vec3d new_localUp = getUpVector(coordinateFrame);


            osg::Quat pan_rotation;
            pan_rotation.makeRotate(localUp,new_localUp);

            if (!pan_rotation.zeroRotation())
            {
                _rotation = _rotation * pan_rotation;
                _previousUp = new_localUp;
                //osg::notify(osg::NOTICE)<<"Rotating from "<<localUp<<" to "<<new_localUp<<"  angle = "<<acos(localUp*new_localUp/(localUp.length()*new_localUp.length()))<<std::endl;

                //clampOrientation();
            }
            else
            {
                osg::notify(osg::INFO)<<"New up orientation nearly inline - no need to rotate"<<std::endl;
            }
        }

        return true;
    }
    else if (buttonMask==osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
    {

        // zoom model.

        double fd = _distance;
        double scale = 1.0f+dy;
        if (fd*scale>_minimumDistance)
        {
            _distance *= scale;
        }
        else
        {
            _distance = _minimumDistance;
        }

        return true;

    }

    return false;
}

void
EarthManipulator::clampOrientation()
{
    osg::Matrixd rotation_matrix;
    rotation_matrix.makeRotate(_rotation);

    osg::Vec3d lookVector = -getUpVector(rotation_matrix);
    osg::Vec3d upVector = getFrontVector(rotation_matrix);

    osg::CoordinateFrame coordinateFrame = getCoordinateFrame(_center);
    osg::Vec3d localUp = getUpVector(coordinateFrame);
    //osg::Vec3d localUp = _previousUp;

    osg::Vec3d sideVector = lookVector ^ localUp;

    if (sideVector.length()<0.1)
    {
        osg::notify(osg::INFO)<<"Side vector short "<<sideVector.length()<<std::endl;

        sideVector = upVector^localUp;
        sideVector.normalize();

    }

    osg::Vec3d newUpVector = sideVector^lookVector;
    newUpVector.normalize();

    osg::Quat rotate_roll;
    rotate_roll.makeRotate(upVector,newUpVector);

    if (!rotate_roll.zeroRotation())
    {
        _rotation = _rotation * rotate_roll;
    }
}
