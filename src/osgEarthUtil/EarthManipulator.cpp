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


/****************************************************************************/


EarthManipulator::Action::Action( ActionType type, bool continuous, double scale_x, double scale_y ) :
_type( type ),
_scale_x( scale_x ),
_scale_y( scale_y ),
_continuous( continuous )
{
    _dir =
        _type == ACTION_PAN_LEFT || _type == ACTION_ROTATE_LEFT? DIR_LEFT :
        _type == ACTION_PAN_RIGHT || _type == ACTION_ROTATE_RIGHT? DIR_RIGHT :
        _type == ACTION_PAN_UP || _type == ACTION_ROTATE_UP || _type == ACTION_ZOOM_IN ? DIR_UP :
        _type == ACTION_PAN_DOWN || _type == ACTION_ROTATE_DOWN || _type == ACTION_ZOOM_OUT ? DIR_DOWN :
        DIR_NA;
}

EarthManipulator::Action::Action( const Action& rhs ) :
_type( rhs._type ),
_scale_x( rhs._scale_x ),
_scale_y( rhs._scale_y ),
_dir( rhs._dir ),
_continuous( rhs._continuous )
{
    //NOP
}


/****************************************************************************/

EarthManipulator::Action EarthManipulator::NullAction( EarthManipulator::ACTION_NULL, 1, 1 );


EarthManipulator::Settings::Settings() :
_throwing( false ),
_single_axis_rotation( false ),
_force_north_up( false ),
_mouse_sens( 1.0 ),
_keyboard_sens( 1.0 ),
_scroll_sens( 1.0 ),
_min_pitch( -90.0 ),
_max_pitch( -10.0 )
{
}

EarthManipulator::Settings::Settings( const EarthManipulator::Settings& rhs ) :
_bindings( rhs._bindings ),
_throwing( rhs._throwing ),
_single_axis_rotation( rhs._single_axis_rotation ),
_force_north_up( rhs._force_north_up ),
_mouse_sens( rhs._mouse_sens ),
_keyboard_sens( rhs._keyboard_sens ),
_scroll_sens( rhs._scroll_sens ),
_min_pitch( rhs._min_pitch ),
_max_pitch( rhs._max_pitch )
{
    //NOP
}

void
EarthManipulator::Settings::bindMouse(ActionType action,
                                      int button_mask, int modkey_mask,
                                      bool continuous, double scale_x, double scale_y )
{
    InputSpec spec( osgGA::GUIEventAdapter::DRAG, button_mask, modkey_mask );
    _bindings.push_back( ActionBinding( spec, Action( action, continuous, scale_x, scale_y ) ) );
}

void
EarthManipulator::Settings::bindKey(ActionType action, int key,
                                    int modkey_mask, bool continuous)
{
    InputSpec spec( osgGA::GUIEventAdapter::KEYDOWN, key, modkey_mask );
    _bindings.push_back( ActionBinding( spec, Action( action, continuous ) ) );
}

void
EarthManipulator::Settings::bindScroll(ActionType action, int scrolling_motion,
                                       int modkey_mask )
{
    InputSpec spec( osgGA::GUIEventAdapter::SCROLL, scrolling_motion, modkey_mask );
    _bindings.push_back( ActionBinding( spec, Action( action ) ) );
}

const EarthManipulator::Action&
EarthManipulator::Settings::getAction(int event_type, int input_mask, int modkey_mask) const
{
    InputSpec spec( event_type, input_mask, modkey_mask );
    for( ActionBindings::const_iterator i = _bindings.begin(); i != _bindings.end(); i++ )
        if ( i->first == spec )
            return i->second;
    return NullAction;
}


/************************************************************************/

#define DISCRETE_DELTA 1.0


EarthManipulator::EarthManipulator() :
_distance( 1.0 ),
_thrown( false ),
_continuous( false ),
_settings( new Settings() ),
_task( new Task() ),
_last_action( ACTION_NULL )
{
    // install default action bindings:

    _settings->bindKey( ACTION_HOME, osgGA::GUIEventAdapter::KEY_Space );

    _settings->bindMouse( ACTION_PAN,    osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON );
    _settings->bindMouse( ACTION_ZOOM,   osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON );
    _settings->bindMouse( ACTION_ROTATE, osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON );
    _settings->bindMouse( ACTION_ROTATE, osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON | osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON );

    _settings->bindScroll( ACTION_ZOOM_IN,  osgGA::GUIEventAdapter::SCROLL_UP );
    _settings->bindScroll( ACTION_ZOOM_OUT, osgGA::GUIEventAdapter::SCROLL_DOWN );
    _settings->setScrollSensitivity( 1.5 );

    _settings->bindKey( ACTION_PAN_LEFT,  osgGA::GUIEventAdapter::KEY_Left );
    _settings->bindKey( ACTION_PAN_RIGHT, osgGA::GUIEventAdapter::KEY_Right );
    _settings->bindKey( ACTION_PAN_UP,    osgGA::GUIEventAdapter::KEY_Up );
    _settings->bindKey( ACTION_PAN_DOWN,  osgGA::GUIEventAdapter::KEY_Down );

    _settings->setThrowingEnabled( false );
}


EarthManipulator::~EarthManipulator()
{
    //NOP
}

void
EarthManipulator::applySettings( Settings* settings )
{
    if ( settings )
    {
        _settings = settings;

        //TODO: make any immediate changes.
        _task->_type = TASK_NONE;
        flushMouseEventStack();
    }
}

EarthManipulator::Settings*
EarthManipulator::getSettings() const
{
    return _settings.get();
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

    setByLookAt(_homeEye, _homeCenter, _homeUp);
    _local_pitch = osg::DegreesToRadians( -90.0 );
    //_local_azim = 0.0;
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

void
EarthManipulator::resetMouse( osgGA::GUIActionAdapter& us )
{
    flushMouseEventStack();
    us.requestContinuousUpdate( false );
    _thrown = false;
    _continuous = false;
    _single_axis_x = 1.0;
    _single_axis_y = 1.0;
}

bool
EarthManipulator::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& us)
{
    bool handled = false;

    if ( ea.getEventType() == osgGA::GUIEventAdapter::FRAME )
    {
        if ( _thrown || _continuous )
        {
            if ( handleMouseAction( _last_action ) )
                us.requestRedraw();
            //osg::notify(osg::NOTICE) << "throwing, action = " << _last_action._type << std::endl;
        }

        if ( !_continuous )
        {
            _continuous_dx = 0.0;
            _continuous_dy = 0.0;
        }
        
        if ( _task.valid() )
        {
            if ( serviceTask() )
                us.requestRedraw();
        }

        return false;
    }

    if (ea.getHandled()) return false;
   
    // form the current Action based on the event type:
    Action action = ACTION_NULL;

    switch( ea.getEventType() )
    {
        case osgGA::GUIEventAdapter::PUSH:
            resetMouse( us );
            addMouseEvent( ea );
            handled = true;
            break;

        case osgGA::GUIEventAdapter::RELEASE:

            // bail out of continuous mode if necessary:
            _continuous = false;

            // check for a mouse-throw continuation:
            if ( _settings->getThrowingEnabled() && isMouseMoving() )
            {
                action = _last_action;
                if( handleMouseAction( action ) )
                {
                    us.requestRedraw();
                    us.requestContinuousUpdate( true );
                    _thrown = true;
                }
            }
            else
            {
                resetMouse( us );
                addMouseEvent( ea );
            }

            handled = true;
            break;

        case osgGA::GUIEventAdapter::MOVE: // MOVE not currently bindable
            handled = false;
            break;

        case osgGA::GUIEventAdapter::DRAG:
            action = _settings->getAction( ea.getEventType(), ea.getButtonMask(), ea.getModKeyMask() );
            addMouseEvent( ea );
            if ( handleMouseAction( action ) )
                us.requestRedraw();
            us.requestContinuousUpdate(false);
            _continuous = action._continuous;
            _thrown = false;
            handled = true;
            break;

        case osgGA::GUIEventAdapter::KEYDOWN:
            resetMouse( us );
            action = _settings->getAction( ea.getEventType(), ea.getKey(), ea.getModKeyMask() );
            if ( handleKeyboardAction( action ) )
                us.requestRedraw();
            handled = true;
            break;
            
        case osgGA::GUIEventAdapter::KEYUP:
            resetMouse( us );
            _task->_type = TASK_NONE;
            handled = true;
            break;

        case osgGA::GUIEventAdapter::SCROLL:
            resetMouse( us );
            addMouseEvent( ea );
            action = _settings->getAction( ea.getEventType(), ea.getScrollingMotion(), ea.getModKeyMask() );
            if ( handleScrollAction( action, 0.2 ) )
                us.requestRedraw();
            handled = true;
            break;
    }

    if ( handled && action._type != ACTION_NULL )
        _last_action = action;

    return handled;
}

bool
EarthManipulator::serviceTask()
{
    bool result;

    osg::Timer_t now = osg::Timer::instance()->tick();

    if ( _task.valid() && _task->_type != TASK_NONE )
    {
        // normalize for 60fps..
        double dt = osg::Timer::instance()->delta_s( _time_last_frame, now ); // / (1.0/60.0);

        switch( _task->_type )
        {
            case TASK_PAN:
                pan( dt * _task->_dx, dt * _task->_dy );
                break;
            case TASK_ROTATE:
                rotate( dt * _task->_dx, dt * _task->_dy );
                break;
            case TASK_ZOOM:
                zoom( dt * _task->_dx, dt * _task->_dy );
                break;
        }

        _task->_duration_s -= dt;
        if ( _task->_duration_s <= 0.0 )
            _task->_type = TASK_NONE;

        result = true;
    }
    else
    {
        result = false;
    }

    _time_last_frame = now;
    return result;
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

    osg::CoordinateFrame coordinateFrame = getCoordinateFrame( _center );
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
EarthManipulator::setByLookAt(const osg::Vec3d& eye,const osg::Vec3d& center,const osg::Vec3d& up)
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


void
EarthManipulator::pan( double dx, double dy )
{
    double scale = -0.3f*_distance;

    osg::Matrixd rotation_matrix;
    rotation_matrix.makeRotate(_rotation);


    // compute look vector.
    osg::Vec3d lookVector = -getUpVector(rotation_matrix);
    osg::Vec3d sideVector = getSideVector(rotation_matrix);
    osg::Vec3d upVector = getFrontVector(rotation_matrix);

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
                _center = (_center-ip1).length2() < (_center-ip2).length2() ? ip1 : ip2;
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
        }
        else
        {
            osg::notify(osg::INFO)<<"New up orientation nearly inline - no need to rotate"<<std::endl;
        }
    }
}

void
EarthManipulator::rotate( double dx, double dy )
{
    // clamp the local pitch delta:
    double minp = osg::DegreesToRadians( _settings->getMinPitch() );
    double maxp = osg::DegreesToRadians( _settings->getMaxPitch() );

    // clamp pitch range:
    if ( dy + _local_pitch > maxp || dy + _local_pitch < minp )
        dy = 0;

    osg::Matrix rotation_matrix;
    rotation_matrix.makeRotate(_rotation);

    osg::Vec3d lookVector = -getUpVector(rotation_matrix);
    osg::Vec3d sideVector = getSideVector(rotation_matrix);
    osg::Vec3d upVector = getFrontVector(rotation_matrix);

    osg::CoordinateFrame coordinateFrame = getCoordinateFrame(_center);
    osg::Vec3d localUp = getUpVector(coordinateFrame);

    osg::Vec3d forwardVector = localUp^sideVector; // cross product
    sideVector = forwardVector^localUp; // cross product

    forwardVector.normalize();
    sideVector.normalize();

    osg::Quat rotate_elevation;
    rotate_elevation.makeRotate( dy, sideVector );

    osg::Quat rotate_azim;
    rotate_azim.makeRotate( -dx, localUp );

    _rotation = _rotation * rotate_elevation * rotate_azim;

    _local_pitch += dy;
//    _local_azim -= dx;
}

void
EarthManipulator::zoom( double dx, double dy )
{
    double fd = _distance;
    double scale = 1.0f + dy;

    if ( fd * scale > _minimumDistance )
    {
        _distance *= scale;
    }
    else
    {
        _distance = _minimumDistance;
    }
}

bool
EarthManipulator::handleMouseAction( const Action& action )
{
    // return if less then two events have been added.
    if (_ga_t0.get()==NULL || _ga_t1.get()==NULL) return false;

    double dx = _ga_t0->getXnormalized()-_ga_t1->getXnormalized();
    double dy = _ga_t0->getYnormalized()-_ga_t1->getYnormalized();

    // return if there is no movement.
    if (dx==0 && dy==0) return false;

    dx *= action._scale_x * _settings->getMouseSensitivity();
    dy *= action._scale_y * _settings->getMouseSensitivity();

    // in "continuous" mode, we accumulate the deltas each frame - thus
    // the deltas act more like speeds.
    if ( _continuous )
    {
        _continuous_dx += dx * 0.01;
        _continuous_dy += dy * 0.01;
        dx = _continuous_dx;
        dy = _continuous_dy;
    }
    
    switch( action._type )
    {
    case ACTION_PAN:
        pan( dx, dy );
        break;

    case ACTION_ROTATE:
        // in "single axis" mode, zero out one of the deltas.
        if ( _continuous && _settings->getSingleAxisRotation() )
        {
            if ( ::fabs(dx) > ::fabs(dy) )
                dy = 0.0;
            else
                dx = 0.0;
        }
        rotate( dx, dy );
        break;

    case ACTION_ZOOM:
        zoom( dx, dy );
        break;

    default:
        return handleAction( action, dx, dy, DBL_MAX );
    }

    return true;
}

bool
EarthManipulator::handleKeyboardAction( const Action& action, double duration )
{
    double dx = 0, dy = 0;

    switch( action._dir )
    {
    case DIR_LEFT:  dx =  DISCRETE_DELTA; break;
    case DIR_RIGHT: dx = -DISCRETE_DELTA; break;
    case DIR_UP:    dy = -DISCRETE_DELTA; break;
    case DIR_DOWN:  dy =  DISCRETE_DELTA; break;
    }

    dx *= action._scale_x * _settings->getKeyboardSensitivity();
    dy *= action._scale_y * _settings->getKeyboardSensitivity();

    return handleAction( action, dx, dy, duration );
}

bool
EarthManipulator::handleScrollAction( const Action& action, double duration )
{
    double dx = 0, dy = 0;

    switch( action._dir )
    {
    case DIR_LEFT:  dx =  DISCRETE_DELTA; break;
    case DIR_RIGHT: dx = -DISCRETE_DELTA; break;
    case DIR_UP:    dy =  DISCRETE_DELTA; break;
    case DIR_DOWN:  dy = -DISCRETE_DELTA; break;
    }

    dx *= action._scale_x * _settings->getScrollSensitivity();
    dy *= action._scale_y * _settings->getScrollSensitivity();

    return handleAction( action, dx, dy, duration );
}

bool
EarthManipulator::handleAction( const Action& action, double dx, double dy, double duration )
{
    bool handled = true;

    //osg::notify(osg::NOTICE) << "action=" << action << ", dx=" << dx << ", dy=" << dy << std::endl;

    switch( action._type )
    {
    case ACTION_HOME:
        if ( getAutoComputeHomePosition() )
            computeHomePosition();
        setByLookAt( _homeEye, _homeCenter, _homeUp );
        break;


    case ACTION_PAN:
    case ACTION_PAN_LEFT:
    case ACTION_PAN_RIGHT:
    case ACTION_PAN_UP:
    case ACTION_PAN_DOWN:
        _task->set( TASK_PAN, dx, dy, duration );
        break;

    case ACTION_ROTATE:
    case ACTION_ROTATE_LEFT:
    case ACTION_ROTATE_RIGHT:
    case ACTION_ROTATE_UP:
    case ACTION_ROTATE_DOWN:
        _task->set( TASK_ROTATE, dx, dy, duration );
        break;

    case ACTION_ZOOM:
    case ACTION_ZOOM_IN:
    case ACTION_ZOOM_OUT:
        _task->set( TASK_ZOOM, dx, dy, duration );
        break;

    default:
        handled = false;
    }

    return handled;
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
