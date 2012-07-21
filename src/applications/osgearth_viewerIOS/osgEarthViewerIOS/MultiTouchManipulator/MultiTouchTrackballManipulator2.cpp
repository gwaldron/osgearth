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


#include "MultiTouchTrackballManipulator2.h"
#include <osg/io_utils>

using namespace osg;
using namespace osgGA;



/// Constructor.
MultiTouchTrackballManipulator2::MultiTouchTrackballManipulator2( int flags )
   : inherited( flags )
{
    setVerticalAxisFixed( false );
}


/// Constructor.
MultiTouchTrackballManipulator2::MultiTouchTrackballManipulator2( const MultiTouchTrackballManipulator2& tm, const CopyOp& copyOp )
    : inherited( tm, copyOp )
{
}


void MultiTouchTrackballManipulator2::handleMultiTouchDrag(GUIEventAdapter::TouchData* now, GUIEventAdapter::TouchData* last, const double eventTimeDelta)
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
    
    if (fabs(gap_last - gap_now) >= zoom_threshold && dotNow <= -0.6f)
    {
        // zoom gesture
        float _zoomScaler = 0.1f;
        zoomModel( ((gap_last - gap_now)*_zoomScaler) * eventTimeDelta, false );
    }

    // drag gesture

    osg::Vec2 delta = ((pt_1_last - pt_1_now) + (pt_2_last - pt_2_now)) / 2.0f;

    float scale = 0.2f * _distance * eventTimeDelta;

    // osg::notify(osg::ALWAYS) << "drag: " << delta << " scale: " << scale << std::endl;

   // panModel( delta.x() * scale, delta.y() * scale * (-1)); // flip y-coord because of different origins.
}


bool MultiTouchTrackballManipulator2::handle( const GUIEventAdapter& ea, GUIActionAdapter& us )
{

    bool handled(false);

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

                // three touches or two taps for home position
                if ((data->getNumTouchPoints() == 3) || ((data->getNumTouchPoints() == 1) && (data->get(0).tapCount >= 2)))
                {
                    flushMouseEventStack();
                    _thrown = false;
                    home(ea,us);
                    handled = true;
                }

                else if (data->getNumTouchPoints() >= 2)
                {
                    if ((_lastTouchData.valid()) && (_lastTouchData->getNumTouchPoints() >= 2))
                    {
                        handleMultiTouchDrag(data, _lastTouchData.get(), eventTimeDelta);
                    }

                    handled = true;
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
                }

            }
            break;
        default:
            break;
    }

    return handled ? handled : TrackballManipulator::handle(ea, us);
}
