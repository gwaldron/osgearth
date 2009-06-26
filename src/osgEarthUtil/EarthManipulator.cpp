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
using namespace osgEarth;

/****************************************************************************/


EarthManipulator::Action::Action( ActionType type, bool continuous, double scale_x, double scale_y ) :
_type( type ),
_scale_x( scale_x ),
_scale_y( scale_y ),
_continuous( continuous )
{
    _dir =
        _type == ACTION_PAN_LEFT  || _type == ACTION_ROTATE_LEFT? DIR_LEFT :
        _type == ACTION_PAN_RIGHT || _type == ACTION_ROTATE_RIGHT? DIR_RIGHT :
        _type == ACTION_PAN_UP    || _type == ACTION_ROTATE_UP   || _type == ACTION_ZOOM_IN ? DIR_UP :
        _type == ACTION_PAN_DOWN  || _type == ACTION_ROTATE_DOWN || _type == ACTION_ZOOM_OUT ? DIR_DOWN :
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
_mouse_sens( 1.0 ),
_keyboard_sens( 1.0 ),
_scroll_sens( 1.0 ),
_min_pitch( -89.9 ),
_max_pitch( -10.0 ),
_lock_azim_while_panning( true )
{
}

EarthManipulator::Settings::Settings( const EarthManipulator::Settings& rhs ) :
_bindings( rhs._bindings ),
_throwing( rhs._throwing ),
_single_axis_rotation( rhs._single_axis_rotation ),
_mouse_sens( rhs._mouse_sens ),
_keyboard_sens( rhs._keyboard_sens ),
_scroll_sens( rhs._scroll_sens ),
_min_pitch( rhs._min_pitch ),
_max_pitch( rhs._max_pitch ),
_lock_azim_while_panning( rhs._lock_azim_while_panning )
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

void
EarthManipulator::Settings::setMinMaxPitch( double min_pitch, double max_pitch )
{
    _min_pitch = osg::clampBetween( min_pitch, -89.9, 89.0 );
    _max_pitch = osg::clampBetween( max_pitch, min_pitch, 89.0 );
}


/************************************************************************/


EarthManipulator::EarthManipulator() :
_distance( 1.0 ),
_thrown( false ),
_continuous( false ),
_settings( new Settings() ),
_task( new Task() ),
_last_action( ACTION_NULL ),
_srs_lookup_failed( false )
{
    // install default action bindings:

    _settings->bindKey( ACTION_HOME, osgGA::GUIEventAdapter::KEY_Space );

    _settings->bindMouse( ACTION_PAN,    osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON );
    _settings->bindMouse( ACTION_ZOOM,   osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON, 0L, true );
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
    _settings->setLockAzimuthWhilePanning( true );
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

        _task->_type = TASK_NONE;
        flushMouseEventStack();

        // apply new pitch restrictions
        double old_pitch = osg::RadiansToDegrees( _local_pitch );
        double new_pitch = osg::clampBetween( old_pitch, _settings->getMinPitch(), _settings->getMaxPitch() );

        if ( new_pitch != old_pitch )
        {
            Viewpoint vp = getViewpoint();
            setViewpoint( Viewpoint(vp.getFocalPoint(), vp.getHeading(), new_pitch, vp.getRange(), vp.getSRS()) );
        }
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

    // reset the srs cache:
    _cached_srs = NULL;
    _srs_lookup_failed = false;

    // track the local angles.
    recalculateLocalPitchAndAzimuth();
}

osg::Node*
EarthManipulator::getNode()
{
    return _node.get();
}


const osgEarth::SpatialReference*
EarthManipulator::getSRS() const
{
    if ( !_cached_srs.valid() && !_srs_lookup_failed && _node.valid() )
    {
        EarthManipulator* nonconst_this = const_cast<EarthManipulator*>(this);

        nonconst_this->_is_geocentric = true;

        // first try to find a map node:
        osgEarth::Map* map = osgEarth::Map::findMapNode( _node.get() );
        if ( map )
        {
            nonconst_this->_cached_srs = map->getProfile()->getSRS();
            nonconst_this->_is_geocentric = map->isGeocentric();
        }

        // if that doesn't work, try gleaning info from a CSN:
        if ( !_cached_srs.valid() )
        {
            osg::CoordinateSystemNode* csn = osgEarth::Map::findCoordinateSystemNode( _node.get() );
            if ( csn )
            {
                nonconst_this->_cached_srs = osgEarth::SpatialReference::create( csn );
                nonconst_this->_is_geocentric = csn->getEllipsoidModel() != NULL;
            }
        }

        nonconst_this->_srs_lookup_failed = !_cached_srs.valid();
    }

    return _cached_srs.get();
}


static double
normalizeAzimRad( double input ) {
    while( input < -osg::PI ) input += osg::PI*2.0;
    while( input > osg::PI ) input -= osg::PI*2.0;
    return input;
}

osg::Matrixd
EarthManipulator::getRotation(const osg::Vec3d& point) const
{
    //The look vector will be going directly from the eye point to the point on the earth,
    //so the look vector is simply the up vector at the center point
    osg::CoordinateFrame cf = getCoordinateFrame(point);
    osg::Vec3d lookVector = -getUpVector(cf);

    osg::Vec3d side;

    //Force the side vector to be orthogonal to north
    osg::Vec3d worldUp(0,0,1);

    double dot = osg::absolute(worldUp * lookVector);
    if (osg::equivalent(dot, 1.0))
    {
        //We are looking nearly straight down the up vector, so use the Y vector for world up instead
        worldUp = osg::Vec3d(0, 1, 0);
        //osg::notify(osg::NOTICE) << "using y vector victor" << std::endl;
    }

    side = lookVector ^ worldUp;
    osg::Vec3d up = side ^ lookVector;
    up.normalize();

    //We want a very slight offset
    double offset = 1e-6;

    return osg::Matrixd::lookAt( point - (lookVector * offset), point, up);
}


void
EarthManipulator::setViewpoint( const Viewpoint& vp )
{
    osg::Vec3d new_center = vp.getFocalPoint();

    // start by transforming the requested focal point into world coordinates:
    if ( getSRS() )
    {
        // resolve the VP's srs. If the VP's SRS is not specified, assume that it
        // is either lat/long (if the map is geocentric) or X/Y (otherwise).
        osg::ref_ptr<const SpatialReference> vp_srs = vp.getSRS()? vp.getSRS() :
            _is_geocentric? getSRS()->getGeographicSRS() :
            getSRS();

        if ( !getSRS()->isEquivalentTo( vp_srs.get() ) )
        {
            osg::Vec3d local = new_center;
            // reproject the focal point if necessary:
            vp_srs->transform( new_center.x(), new_center.y(), getSRS(), local.x(), local.y() );
            new_center = local;
        }

        // convert to geocentric coords if necessary:
        if ( _is_geocentric )
        {
            osg::Vec3d geocentric;

            getSRS()->getEllipsoid()->convertLatLongHeightToXYZ(
                osg::DegreesToRadians( new_center.y() ),
                osg::DegreesToRadians( new_center.x() ),
                new_center.z(),
                geocentric.x(), geocentric.y(), geocentric.z() );

            new_center = geocentric;            
        }
    }

    // now calculate the new rotation matrix based on the angles:

    double new_pitch = osg::DegreesToRadians(
        osg::clampBetween( vp.getPitch(), _settings->getMinPitch(), _settings->getMaxPitch() ) );

    double new_azim = normalizeAzimRad( osg::DegreesToRadians( vp.getHeading() ) );

    _center = new_center;
    _distance = osg::maximum( vp.getRange(), 1.0 );
    
    osg::CoordinateFrame local_frame = getCoordinateFrame( new_center );
    _previousUp = getUpVector( local_frame );

    osg::Matrixd center_rot = getRotation( new_center );
    osg::Quat azim_q( new_azim, osg::Vec3d(0,0,1) );
    osg::Quat pitch_q( -new_pitch -osg::PI_2, osg::Vec3d(1,0,0) );

    osg::Matrix new_rot = center_rot * osg::Matrixd( azim_q * pitch_q );

    _rotation = osg::Matrixd::inverse(new_rot).getRotate();

    _local_pitch = 0.0;
    _local_azim  = 0.0;

    recalculateLocalPitchAndAzimuth();
}


Viewpoint
EarthManipulator::getViewpoint() const
{
    osg::Vec3d focal_point = _center;

    if ( getSRS() && _is_geocentric )
    {
        // convert geocentric to lat/long:
        getSRS()->getEllipsoid()->convertXYZToLatLongHeight(
            _center.x(), _center.y(), _center.z(),
            focal_point.y(), focal_point.x(), focal_point.z() );

        focal_point.x() = osg::RadiansToDegrees( focal_point.x() );
        focal_point.y() = osg::RadiansToDegrees( focal_point.y() );
    }

    return Viewpoint(
        focal_point,
        osg::RadiansToDegrees( _local_azim ),
        osg::RadiansToDegrees( _local_pitch ),
        _distance,
        getSRS() );
}


void
EarthManipulator::setTetherNode( osg::Node* node )
{
    _tether_node = node;
}

osg::Node*
EarthManipulator::getTetherNode() const
{
    return _tether_node.get();
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
    us.requestRedraw();
}

void
EarthManipulator::computeHomePosition()
{    
    if( getNode() )
    {
        const osg::BoundingSphere& boundingSphere = getNode()->getBound();

        osg::Vec3d eye =
            boundingSphere._center +
            osg::Vec3( 0.0, -3.5f * boundingSphere._radius, boundingSphere._radius * 0.0001 );

        setHomePosition(
            eye,
            boundingSphere._center,
            osg::Vec3d( 0, 0, 1 ),
            _autoComputeHomePosition );
    }
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
        // check for _center update due to tethering:
        if ( _tether_node.valid() )
        {
            updateTether();
        }

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

void
EarthManipulator::updateTether()
{
    // capture a temporary ref since _tether_node is just an observer:
    osg::ref_ptr<osg::Node> temp = _tether_node.get();
    if ( temp.valid() )
    {
        const osg::BoundingSphere& bs = temp->getBound();
        if ( bs._center != _center )
        {
            osg::Vec3d new_center = bs._center;
            if ( getSRS() && _is_geocentric )
            {
                double lat_r, lon_r, h;
                getSRS()->getEllipsoid()->convertXYZToLatLongHeight(
                    new_center.x(), new_center.y(), new_center.z(),
                    lat_r, lon_r, h );
                new_center.set( osg::RadiansToDegrees(lon_r), osg::RadiansToDegrees(lat_r), h );
            }
            Viewpoint vp = getViewpoint();
            vp.setFocalPoint( new_center );
            setViewpoint( vp );
        }
    }
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

    recalculateRoll();
    recalculateLocalPitchAndAzimuth();
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

    recalculateRoll();
    recalculateLocalPitchAndAzimuth();
}


void
EarthManipulator::pan( double dx, double dy )
{
    double scale = -0.3f*_distance;
    double old_azim = _local_azim;

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

    // save the previous CF so we can do azimuth locking:
    osg::CoordinateFrame old_frame = getCoordinateFrame( _center );

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
        pan_rotation.makeRotate( localUp, new_localUp );

        if ( !pan_rotation.zeroRotation() )
        {
            _rotation = _rotation * pan_rotation;
            _previousUp = new_localUp;
        }
        else
        {
            osg::notify(osg::INFO)<<"New up orientation nearly inline - no need to rotate"<<std::endl;
        }

        if ( _settings->getLockAzimuthWhilePanning() )
        {
            recalculateLocalPitchAndAzimuth();

            double delta_azim = _local_azim - old_azim;

            osg::Quat q;
            q.makeRotate( delta_azim, new_localUp );
            if ( !q.zeroRotation() )
            {
                _rotation = _rotation * q;
            }
        }
    }

    recalculateLocalPitchAndAzimuth();
}

void
EarthManipulator::rotate( double dx, double dy )
{
    // clamp the local pitch delta; never allow the pitch to hit -90.
    double minp = osg::DegreesToRadians( osg::clampAbove( _settings->getMinPitch(), -89.9 ) );
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

    recalculateLocalPitchAndAzimuth();
//    _local_pitch += dy;
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
    case DIR_LEFT:  dx =  1; break;
    case DIR_RIGHT: dx = -1; break;
    case DIR_UP:    dy = -1; break;
    case DIR_DOWN:  dy =  1; break;
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
    case DIR_LEFT:  dx =  1; break;
    case DIR_RIGHT: dx = -1; break;
    case DIR_UP:    dy =  1; break;
    case DIR_DOWN:  dy = -1; break;
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
EarthManipulator::recalculateRoll()
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


void
EarthManipulator::recalculateLocalPitchAndAzimuth()
{
    // reproject the view matrix into the local CS of the focal point:
    osg::Matrix m = getMatrix() * osg::Matrixd::inverse( getCoordinateFrame( _center ) );
    osg::Vec3d look = -getUpVector( m ); // -m(2,0), -m(2,1), -m(2,2)
    osg::Vec3d up   =  getFrontVector( m );
    //osg::Vec3d look( -m(2,0), -m(2,1), -m(2,2) );
    
    look.normalize();
    up.normalize();

    double azim;    
    if ( look.z() < -0.9 )
        azim = atan2( up.x(), up.y() );
    else if ( look.z() > 0.9 )
        azim = atan2( -up.x(), -up.y() );
    else
        azim = atan2( look.x(), look.y() );

    _local_azim  = normalizeAzimRad( azim );
    _local_pitch = -asin( -look.z() );

    //osg::notify(osg::NOTICE)
    //    << "P=" << osg::RadiansToDegrees(_local_pitch)
    //    << ", A=" << osg::RadiansToDegrees(_local_azim)
    //    //<< ", X=" << lookVectorXY.x()
    //    //<< ", Y=" << lookVectorXY.y()
    //    << std::endl;
}
