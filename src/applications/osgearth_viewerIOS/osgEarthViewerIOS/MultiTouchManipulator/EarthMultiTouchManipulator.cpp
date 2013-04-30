/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2010 Robert Osfield
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
*/


#include "EarthMultiTouchManipulator.h"

using namespace osgEarth::Util;

/// Constructor.
EarthMultiTouchManipulator::EarthMultiTouchManipulator()
   : EarthManipulator(),
    _gestureState(NO_GESTURE)
{

}


/// Constructor.
EarthMultiTouchManipulator::EarthMultiTouchManipulator( const EarthMultiTouchManipulator& tm)
    : EarthManipulator(tm)
{
}

EarthMultiTouchManipulator::~EarthMultiTouchManipulator(){
    
}

osgGA::GUIEventAdapter* EarthMultiTouchManipulator::handleMultiTouchDrag(osgGA::GUIEventAdapter::TouchData* now, 
                                                                         osgGA::GUIEventAdapter::TouchData* last, 
                                                                         const osgGA::GUIEventAdapter& ea,
                                                                         const double eventTimeDelta)
{
    const float zoom_threshold = 1.0f;
    
    osg::Vec2 pt_1_now(now->get(0).x,now->get(0).y);
    osg::Vec2 pt_2_now(now->get(1).x,now->get(1).y);
    osg::Vec2 pt_1_last(last->get(0).x,last->get(0).y);
    osg::Vec2 pt_2_last(last->get(1).x,last->get(1).y);
    
    float gap_now((pt_1_now - pt_2_now).length());
    float gap_last((pt_1_last - pt_2_last).length());
    
    osg::Vec2 pt1Dir = pt_1_now - pt_1_last;
    //float pt1Traveled = pt1Dir.normalize();
    osg::Vec2 pt2Dir = pt_2_now - pt_2_last;
    //float pt2Traveled = pt2Dir.normalize();
    float dotNow = pt1Dir * pt2Dir;
    
    
    //osg::notify(osg::ALWAYS) << "Gap: " << gap_now << " " << gap_last << ", Dot: " << dotNow << std::endl;
    
    if (fabs(gap_last - gap_now) >= zoom_threshold && dotNow <= -0.6f)// && _gestureState != TWO_DRAGING)
    {
        _gestureState = PINCHING;
        
        // zoom gesture
        osgGA::GUIEventAdapter* zoomAdpt = new osgGA::GUIEventAdapter(ea);
        zoomAdpt->setButtonMask(osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON);
        _pinchVector.y() += gap_last - gap_now;
        zoomAdpt->setY(_pinchVector.y());
        return zoomAdpt;
        
    }else if(fabs(gap_last - gap_now) >= zoom_threshold && dotNow >= 0.3f){// && _gestureState != PINCHING){
    
        _gestureState = TWO_DRAGING;
        OSG_ALWAYS << "two drag" << std::endl;
        // drag gesture
        osgGA::GUIEventAdapter* dragAdpt = new osgGA::GUIEventAdapter(ea);
        dragAdpt->setButtonMask(osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON);
        return dragAdpt;
        
    }else{
        if(_gestureState == PINCHING){
            osgGA::GUIEventAdapter* zoomAdpt = new osgGA::GUIEventAdapter(ea);
            zoomAdpt->setButtonMask(osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON);
            zoomAdpt->setY(_pinchVector.y());
            return zoomAdpt;
        }
        if(_gestureState == TWO_DRAGING){
            osgGA::GUIEventAdapter* dragAdpt = new osgGA::GUIEventAdapter(ea);
            dragAdpt->setButtonMask(osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON);
            return dragAdpt;
        }
    }
    return NULL;
}


bool EarthMultiTouchManipulator::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
{
    
    osg::ref_ptr<osgGA::GUIEventAdapter> touchAdpt = NULL;

    switch(ea.getEventType())
    {

        case osgGA::GUIEventAdapter::PUSH:
        case osgGA::GUIEventAdapter::DRAG:
        case osgGA::GUIEventAdapter::RELEASE:
            if (ea.isMultiTouchEvent())
            {
                double eventTimeDelta = 1/60.0; //_ga_t0->getTime() - _ga_t1->getTime();
                if( eventTimeDelta < 0. )
                {
                    OSG_WARN << "Manipulator warning: eventTimeDelta = " << eventTimeDelta << std::endl;
                    eventTimeDelta = 0.;
                }
                osgGA::GUIEventAdapter::TouchData* data = ea.getTouchData();

                // single double tap replaces left double click
                if((data->getNumTouchPoints() == 1) && (data->get(0).tapCount >= 2))
                {
                    OSG_ALWAYS << "Left Click" << std::endl;
                    //build dummy input event
                    touchAdpt = new osgGA::GUIEventAdapter(ea);
                    touchAdpt->setButtonMask(osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON);
                    touchAdpt->setEventType(osgGA::GUIEventAdapter::DOUBLECLICK);
                    
                //two touch doble tap replaces right click
                }else if((data->getNumTouchPoints() == 2) && (data->get(0).tapCount >= 2))
                {
                    OSG_ALWAYS << "Right Click" << std::endl;
                    //build dummy input event
                    touchAdpt = new osgGA::GUIEventAdapter(ea);
                    touchAdpt->setButtonMask(osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON);
                    touchAdpt->setEventType(osgGA::GUIEventAdapter::DOUBLECLICK);
                    
                }else if (data->getNumTouchPoints() >= 2)//handle multi touch
                {
                    if ((_lastTouchData.valid()) && (_lastTouchData->getNumTouchPoints() >= 2))
                    {
                        touchAdpt = handleMultiTouchDrag(data, _lastTouchData.get(), ea, eventTimeDelta);
                    }else{
                        _pinchVector.y() = ea.getY();
                    }
                    
                    //handled = true;
                }

                _lastTouchData = data;

                // check if all touches ended
                unsigned int num_touches_ended(0);
                for(osgGA::GUIEventAdapter::TouchData::iterator i = data->begin(); i != data->end(); ++i)
                {
                    if ((*i).phase == osgGA::GUIEventAdapter::TOUCH_ENDED)
                        num_touches_ended++;
                }

                if(num_touches_ended == data->getNumTouchPoints())
                {
                    _lastTouchData = NULL;
                    _pinchVector = osg::Vec2(0,0);
                    _gestureState = NO_GESTURE;
                }

            }
            break;
        default:
            break;
    }

    if(touchAdpt.valid()){
        return EarthManipulator::handle(*touchAdpt.get(), us);
    }
    return EarthManipulator::handle(ea, us);
}
