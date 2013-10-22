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

#ifndef OSGEARTHUTIL_EARTHMULTITOUCHMANIPULATOR
#define OSGEARTHUTIL_EARTHMULTITOUCHMANIPULATOR

#include <osgEarthUtil/EarthManipulator>


namespace osgEarth { namespace Util
{
    
class EarthMultiTouchManipulator : public EarthManipulator
{
public:
    enum GestureState{
        NO_GESTURE,
        PINCHING,
        TWO_DRAGING
    };

    EarthMultiTouchManipulator();
    EarthMultiTouchManipulator( const EarthMultiTouchManipulator& tm);


    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );

public: // osgGA::MatrixManipulator
    
    virtual const char* className() const { return "EarthMultiTouchManipulator"; }
    
protected:
    virtual ~EarthMultiTouchManipulator();

    osgGA::GUIEventAdapter* handleMultiTouchDrag(osgGA::GUIEventAdapter::TouchData* now, 
                                                 osgGA::GUIEventAdapter::TouchData* last, 
                                                 const osgGA::GUIEventAdapter& ea,
                                                 const double eventTimeDelta);

    GestureState _gestureState;
    osg::Vec2 _pinchVector;
    osg::ref_ptr<osgGA::GUIEventAdapter::TouchData> _lastTouchData;
};

}
}

#endif /* OSGEARTHUTIL_EARTHMULTITOUCHMANIPULATOR */
