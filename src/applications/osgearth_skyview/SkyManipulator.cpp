#include "SkyManipulator"
#include <iostream>


SkyManipulator::SkyManipulator():
_heading(0.0),
_pitch(0.0),
_distance(1.0),
_prevX(FLT_MAX),
_prevY(FLT_MAX)
{
}

void SkyManipulator::setByMatrix(const osg::Matrixd& matrix)
{
}

void SkyManipulator::setByInverseMatrix(const osg::Matrixd& matrix)
{
}

osg::Matrixd SkyManipulator::getMatrix() const
{
    osg::Quat rot = getRotation();
    return osg::Matrixd::translate(0,0,-_distance) *  
           osg::Matrixd::rotate(rot);
}

osg::Matrixd SkyManipulator::getInverseMatrix() const
{
    return osg::Matrixd::inverse(getMatrix());
}

bool SkyManipulator::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    double maxDistance = osg::WGS_84_RADIUS_EQUATOR;

    if (ea.getEventType() == osgGA::GUIEventAdapter::SCROLL)
    {
        
        double speed = 0.1;
        double diff = (maxDistance - _distance) * speed;

        if (ea.getScrollingMotion() == osgGA::GUIEventAdapter::SCROLL_UP)
        {
            _distance += diff;
        }
        else
        {
            _distance -= diff;
        }

        _distance = osg::clampBetween(_distance, 0.0, maxDistance);
        
        return true;
    }
    else if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG)
    {
        if (_prevX != FLT_MAX && _prevY != FLT_MAX)
        {
            float dx = ea.getX() - _prevX;
            float dy = ea.getY() - _prevY;

            double maxSpeed = osg::PI * 2.0 / 300.0;
            double minSpeed = osg::PI * 2.0 / 30000.0;

            double speed = minSpeed + (1.0 - _distance / maxDistance) * (maxSpeed - minSpeed);

            _heading -= dx * speed;
            _pitch -= dy * speed;
        }

        _prevX = ea.getX();
        _prevY = ea.getY();
        return true;
    }
    else if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH)
    {
        _prevX = ea.getX();
        _prevY = ea.getY();
    }
    else if (ea.getEventType() == osgGA::GUIEventAdapter::RELEASE)
    {
        _prevX = FLT_MAX;
        _prevY = FLT_MAX;
    }
    else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
    {
        if (ea.getKey()== osgGA::GUIEventAdapter::KEY_Space)
        {
            home(0.0);
        }
    }
        
    
    return false;
}

osg::Quat SkyManipulator::getRotation() const
{
    osg::Quat azim_q (  _heading,            osg::Vec3d(0,0,1) );
    osg::Quat pitch_q( -_pitch-osg::PI_2, osg::Vec3d(1,0,0) );
    osg::Matrix newRot = osg::Matrixd( azim_q * pitch_q );
    return osg::Matrixd::inverse(newRot).getRotate();  
}

void
SkyManipulator::home(double unused)
{
    _heading = 0.0;
    _pitch = 0.0;
    _distance = 0.0;
}

void
SkyManipulator::home(const osgGA::GUIEventAdapter& ,osgGA::GUIActionAdapter& us)
{
    home( 0.0 );
    us.requestRedraw();
}