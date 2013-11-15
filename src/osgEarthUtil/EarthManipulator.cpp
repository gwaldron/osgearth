/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarth/MapNode>
#include <osgEarth/NodeUtils>
#include <osg/Quat>
#include <osg/Notify>
#include <osg/MatrixTransform>
#include <osgUtil/LineSegmentIntersector>
#include <osgViewer/View>
#include <iomanip>
#include <osgEarth/DPLineSegmentIntersector>

#include <osg/io_utils>

#define LC "[EarthManip] "

using namespace osgEarth::Util;
using namespace osgEarth;


//------------------------------------------------------------------------


namespace
{
    // a reasonable approximation of cosine interpolation
    double
    smoothStepInterp( double t ) {
        return (t*t)*(3.0-2.0*t);
    }

    // rough approximation of pow(x,y)
    double
    powFast( double x, double y ) {
        return x/(x+y-y*x);
    }

    // accel/decel curve (a < 0 => decel)
    double
    accelerationInterp( double t, double a ) {
        return a == 0.0? t : a > 0.0? powFast( t, a ) : 1.0 - powFast(1.0-t, -a);
    }
}



namespace
{
    // Callback that notifies the manipulator whenever the terrain changes
    // around its center point.
    struct ManipTerrainCallback : public TerrainCallback
    {
        ManipTerrainCallback(EarthManipulator* manip) : _manip(manip) { }

        void onTileAdded( const TileKey& key, osg::Node* tile, TerrainCallbackContext& context )
        {
            const GeoPoint& centerMap = _manip->centerMap();
            if ( _manip.valid() && key.getExtent().contains(centerMap.x(), centerMap.y()) )
            {
                _manip->recalculateCenter();
            }
        }

        osg::observer_ptr<EarthManipulator> _manip;
    };
}


//------------------------------------------------------------------------



EarthManipulator::Action::Action( ActionType type, const ActionOptions& options ) :
_type( type ),
_options( options )
{ 
    init();
}

EarthManipulator::Action::Action( ActionType type ) :
_type( type )
{
    init();
}

void
EarthManipulator::Action::init()
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
_dir( rhs._dir ),
_options( rhs._options )
{
    //nop
}

bool 
EarthManipulator::Action::getBoolOption( int option, bool defaultValue ) const
{
    for(ActionOptions::const_iterator i = _options.begin(); i != _options.end(); i++ ) {
        if ( i->option() == option )
            return i->boolValue();
    }
    return defaultValue;
}

int 
EarthManipulator::Action::getIntOption( int option, int defaultValue ) const
{
    for(ActionOptions::const_iterator i = _options.begin(); i != _options.end(); i++ ) {
        if ( i->option() == option )
            return i->intValue();
    }
    return defaultValue;
}

double 
EarthManipulator::Action::getDoubleOption( int option, double defaultValue ) const
{
    for(ActionOptions::const_iterator i = _options.begin(); i != _options.end(); i++ ) {
        if ( i->option() == option )
            return i->doubleValue();
    }
    return defaultValue;
}

/****************************************************************************/

EarthManipulator::Action EarthManipulator::NullAction( EarthManipulator::ACTION_NULL );

static std::string s_actionNames[] = {
    "null",
    "home",
    "goto",
    "pan",
    "pan-left",
    "pan-right",
    "pan-up",
    "pan-down",
    "rotate",
    "rotate-left",
    "rotate-right",
    "rotate-up",
    "rotate-down",
    "zoom",
    "zoom-in",
    "zoom-out",
    "earth-drag"
};

static std::string s_actionOptionNames[] = {
    "scale-x",
    "scale-y",
    "continuous",
    "single-axis",
    "goto-range-factor",
    "duration"
};

static short s_actionOptionTypes[] = { 1, 1, 0, 0, 1, 1 }; // 0=bool, 1=double

//------------------------------------------------------------------------

EarthManipulator::Settings::Settings() :
osg::Referenced                 (),
Revisioned                      (),
_single_axis_rotation           ( false ),
_lock_azim_while_panning        ( true ),
_mouse_sens                     ( 1.0 ),
_keyboard_sens                  ( 1.0 ),
_scroll_sens                    ( 1.0 ),
_min_pitch                      ( -89.9 ),
_max_pitch                      ( -10.0 ),
_max_x_offset                   ( 0.0 ),
_max_y_offset                   ( 0.0 ),
_min_distance                   ( 0.001 ),
_max_distance                   ( DBL_MAX ),
_tether_mode                    ( TETHER_CENTER ),
_arc_viewpoints                 ( true ),
_auto_vp_duration               ( false ),
_min_vp_duration_s              ( 3.0 ),
_max_vp_duration_s              ( 8.0 ),
_camProjType                    ( PROJ_PERSPECTIVE ),
_camFrustOffsets                ( 0, 0 ),
_disableCollisionAvoidance      ( false ),
_throwingEnabled                ( false ),
_throwDecayRate                 ( 0.05 )
{
    //NOP
}

EarthManipulator::Settings::Settings( const EarthManipulator::Settings& rhs ) :
osg::Referenced( rhs ),
Revisioned     ( rhs ),
_bindings( rhs._bindings ),
_single_axis_rotation( rhs._single_axis_rotation ),
_lock_azim_while_panning( rhs._lock_azim_while_panning ),
_mouse_sens( rhs._mouse_sens ),
_keyboard_sens( rhs._keyboard_sens ),
_scroll_sens( rhs._scroll_sens ),
_min_pitch( rhs._min_pitch ),
_max_pitch( rhs._max_pitch ),
_max_x_offset( rhs._max_x_offset ),
_max_y_offset( rhs._max_y_offset ),
_min_distance( rhs._min_distance ),
_max_distance( rhs._max_distance ),
_tether_mode( rhs._tether_mode ),
_arc_viewpoints( rhs._arc_viewpoints ),
_auto_vp_duration( rhs._auto_vp_duration ),
_min_vp_duration_s( rhs._min_vp_duration_s ),
_max_vp_duration_s( rhs._max_vp_duration_s ),
_camProjType( rhs._camProjType ),
_camFrustOffsets( rhs._camFrustOffsets ),
_breakTetherActions( rhs._breakTetherActions ),
_disableCollisionAvoidance( rhs._disableCollisionAvoidance),
_throwingEnabled( rhs._throwingEnabled ),
_throwDecayRate( rhs._throwDecayRate )
{
    //NOP
}

#define HASMODKEY( W, V ) (( W & V ) == V )

// expands one input spec into many if necessary, to deal with modifier key combos.
void
EarthManipulator::Settings::expandSpec( const InputSpec& input, InputSpecs& output ) const
{
    int e = input._event_type;
    int i = input._input_mask;
    int m = input._modkey_mask;

    if ( HASMODKEY(m, osgGA::GUIEventAdapter::MODKEY_CTRL) )
    {
        expandSpec( InputSpec( e, i, m & ~osgGA::GUIEventAdapter::MODKEY_LEFT_CTRL ), output );
        expandSpec( InputSpec( e, i, m & ~osgGA::GUIEventAdapter::MODKEY_RIGHT_CTRL ), output );
    }
    else if ( HASMODKEY(m, osgGA::GUIEventAdapter::MODKEY_ALT) )
    {
        expandSpec( InputSpec( e, i, m & ~osgGA::GUIEventAdapter::MODKEY_LEFT_ALT ), output );
        expandSpec( InputSpec( e, i, m & ~osgGA::GUIEventAdapter::MODKEY_RIGHT_ALT ), output );
    }
    else if ( HASMODKEY(m, osgGA::GUIEventAdapter::MODKEY_SHIFT) )
    {
        expandSpec( InputSpec( e, i, m & ~osgGA::GUIEventAdapter::MODKEY_LEFT_SHIFT ), output );
        expandSpec( InputSpec( e, i, m & ~osgGA::GUIEventAdapter::MODKEY_RIGHT_SHIFT ), output );
    }
    else if ( HASMODKEY(m, osgGA::GUIEventAdapter::MODKEY_META) )
    {
        expandSpec( InputSpec( e, i, m & ~osgGA::GUIEventAdapter::MODKEY_LEFT_META ), output );
        expandSpec( InputSpec( e, i, m & ~osgGA::GUIEventAdapter::MODKEY_RIGHT_META ), output );
    }
    else if ( HASMODKEY(m, osgGA::GUIEventAdapter::MODKEY_HYPER) )
    {
        expandSpec( InputSpec( e, i, m & ~osgGA::GUIEventAdapter::MODKEY_LEFT_HYPER ), output );
        expandSpec( InputSpec( e, i, m & ~osgGA::GUIEventAdapter::MODKEY_RIGHT_HYPER ), output );
    }

    //Always add the input so if we are dealing with a windowing system like QT that just sends MODKEY_CTRL it will still work.
    output.push_back( input );
}

void
EarthManipulator::Settings::bind( const InputSpec& spec, const Action& action )
{
    InputSpecs specs;
    expandSpec( spec, specs );
    for( InputSpecs::const_iterator i = specs.begin(); i != specs.end(); i++ )
    {
        _bindings[*i] = action;
    }
}

void
EarthManipulator::Settings::bindMouse(ActionType actionType,
                                      int button_mask, int modkey_mask,
                                      const ActionOptions& options)
{
    bind(
        InputSpec( osgGA::GUIEventAdapter::DRAG, button_mask, modkey_mask ),
        Action( actionType, options ) );
}

void
EarthManipulator::Settings::bindMouseClick(ActionType action,
                                           int button_mask, int modkey_mask,
                                           const ActionOptions& options)
{
    bind(
        InputSpec( EVENT_MOUSE_CLICK, button_mask, modkey_mask ),
        Action( action, options ) );
}

void
EarthManipulator::Settings::bindMouseDoubleClick(ActionType action,
                                                 int button_mask, int modkey_mask,
                                                 const ActionOptions& options)
{
    bind(
        InputSpec( EVENT_MOUSE_DOUBLE_CLICK, button_mask, modkey_mask ),
        Action( action, options ) );
}

void
EarthManipulator::Settings::bindKey(ActionType action,
                                    int key, int modkey_mask,
                                    const ActionOptions& options)
{
    bind(
        InputSpec( osgGA::GUIEventAdapter::KEYDOWN, key, modkey_mask ),
        Action( action, options ) );
}

void
EarthManipulator::Settings::bindScroll(ActionType action, int scrolling_motion,
                                       int modkey_mask, const ActionOptions& options )
{
    bind(
        InputSpec ( osgGA::GUIEventAdapter::SCROLL, scrolling_motion, modkey_mask ),
        Action( action, options ) );
}


void
EarthManipulator::Settings::bindPinch(ActionType action, const ActionOptions& options)
{
    bind(
        InputSpec( EarthManipulator::EVENT_MULTI_PINCH, 0, 0 ),
        Action( action, options ) );
}

void
EarthManipulator::Settings::bindTwist(ActionType action, const ActionOptions& options)
{
    bind(
         InputSpec( EarthManipulator::EVENT_MULTI_TWIST, 0, 0 ),
         Action( action, options ) );
}

void
EarthManipulator::Settings::bindMultiDrag(ActionType action, const ActionOptions& options)
{
    bind(
        InputSpec( EarthManipulator::EVENT_MULTI_DRAG, 0, 0 ),
        Action( action, options ) );
}

const EarthManipulator::Action&
EarthManipulator::Settings::getAction(int event_type, int input_mask, int modkey_mask) const
{
    //Build the input spec but remove the numlock and caps lock from the modkey mask.  On Linux these seem to be passed in as part of the modkeymask
    //if they are on.  So if you bind an action like SCROLL to a modkey mask of 0 or a modkey mask of ctrl it will never match the spec exactly b/c 
    //the modkey mask also includes capslock and numlock.
    InputSpec spec( event_type, input_mask, modkey_mask & ~osgGA::GUIEventAdapter::MODKEY_NUM_LOCK & ~osgGA::GUIEventAdapter::MODKEY_CAPS_LOCK);
    ActionBindings::const_iterator i = _bindings.find(spec);
    return i != _bindings.end() ? i->second : NullAction;
}

void
EarthManipulator::Settings::setMinMaxPitch( double min_pitch, double max_pitch )
{
    _min_pitch = osg::clampBetween( min_pitch, -89.9, 89.0 );
    _max_pitch = osg::clampBetween( max_pitch, min_pitch, 89.0 );
    dirty();
}

void
EarthManipulator::Settings::setMaxOffset(double max_x_offset, double max_y_offset)
{
    _max_x_offset = max_x_offset;
    _max_y_offset = max_y_offset;
    dirty();
}

void
EarthManipulator::Settings::setMinMaxDistance( double min_distance, double max_distance)
{
    _min_distance = min_distance;
    _max_distance = max_distance;
    dirty();
}

void
EarthManipulator::Settings::setArcViewpointTransitions( bool value )
{
    _arc_viewpoints = value;
    dirty();
}

void
EarthManipulator::Settings::setAutoViewpointDurationEnabled( bool value )
{
    _auto_vp_duration = value;
    dirty();
}

void
EarthManipulator::Settings::setAutoViewpointDurationLimits( double minSeconds, double maxSeconds )
{
    _min_vp_duration_s = osg::clampAbove( minSeconds, 0.0 );
    _max_vp_duration_s = osg::clampAbove( maxSeconds, _min_vp_duration_s );
    dirty();
}

void
EarthManipulator::Settings::setCameraProjection(const EarthManipulator::CameraProjection& value)
{
    _camProjType = value;
    dirty();
}

void
EarthManipulator::Settings::setCameraFrustumOffsets( const osg::Vec2s& value )
{
    _camFrustOffsets = value;
    dirty();
}

/************************************************************************/


EarthManipulator::EarthManipulator() :
osgGA::CameraManipulator(),
_last_action           ( ACTION_NULL ),
_last_event            ( EVENT_MOUSE_DOUBLE_CLICK ),
_time_s_last_event     (0.0),
_frame_count           ( 0 ),
_findNodeTraversalMask ( 0x01 )
{
    reinitialize();
    configureDefaultSettings();
}

EarthManipulator::EarthManipulator( const EarthManipulator& rhs ) :
osgGA::CameraManipulator( rhs ),
_last_action            ( ACTION_NULL ),
_last_event             ( EVENT_MOUSE_DOUBLE_CLICK ),
_time_s_last_event      (0.0),
_frame_count            ( 0 ),
_settings               ( new Settings(*rhs.getSettings()) ),
_findNodeTraversalMask  ( rhs._findNodeTraversalMask )
{
    reinitialize();
}


EarthManipulator::~EarthManipulator()
{
    osg::ref_ptr<osg::Node> safeNode = _node.get();
    if (safeNode && _terrainCallback)
    {
        // find a map node.
        MapNode* mapNode = MapNode::findMapNode( safeNode.get(), _findNodeTraversalMask );
        if ( mapNode )
        {             
            mapNode->getTerrain()->removeTerrainCallback( _terrainCallback );
        }
    }    
}

void
EarthManipulator::configureDefaultSettings()
{
    _settings = new Settings();

    // install default action bindings:
    ActionOptions options;

    _settings->bindKey( ACTION_HOME, osgGA::GUIEventAdapter::KEY_Space );

    _settings->bindMouse( ACTION_PAN, osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON );
    //_settings->bindMouse( ACTION_EARTH_DRAG, osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON );

    // zoom as you hold the right button:
    options.clear();
    options.add( OPTION_CONTINUOUS, true );
    _settings->bindMouse( ACTION_ZOOM, osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON, 0L, options );

    // rotate with either the middle button or the left+right buttons:
    _settings->bindMouse( ACTION_ROTATE, osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON );
    _settings->bindMouse( ACTION_ROTATE, osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON | osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON );

    // zoom with the scroll wheel:
    _settings->bindScroll( ACTION_ZOOM_IN,  osgGA::GUIEventAdapter::SCROLL_DOWN );
    _settings->bindScroll( ACTION_ZOOM_OUT, osgGA::GUIEventAdapter::SCROLL_UP );

    // pan around with arrow keys:
    _settings->bindKey( ACTION_PAN_LEFT,  osgGA::GUIEventAdapter::KEY_Left );
    _settings->bindKey( ACTION_PAN_RIGHT, osgGA::GUIEventAdapter::KEY_Right );
    _settings->bindKey( ACTION_PAN_UP,    osgGA::GUIEventAdapter::KEY_Up );
    _settings->bindKey( ACTION_PAN_DOWN,  osgGA::GUIEventAdapter::KEY_Down );

    // double click the left button to zoom in on a point:
    options.clear();
    options.add( OPTION_GOTO_RANGE_FACTOR, 0.4 );
    _settings->bindMouseDoubleClick( ACTION_GOTO, osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON, 0L, options );

    // double click the right button (or CTRL-left button) to zoom out to a point
    options.clear();
    options.add( OPTION_GOTO_RANGE_FACTOR, 2.5 );
    _settings->bindMouseDoubleClick( ACTION_GOTO, osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON, 0L, options );
    _settings->bindMouseDoubleClick( ACTION_GOTO, osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON, osgGA::GUIEventAdapter::MODKEY_CTRL, options );

    // map multi-touch pinch to a discrete zoom
    options.clear();
    _settings->bindPinch( ACTION_ZOOM, options );

    options.clear();
    _settings->bindTwist( ACTION_ROTATE, options );
    _settings->bindMultiDrag( ACTION_ROTATE, options );

    //_settings->setThrowingEnabled( false );
    _settings->setLockAzimuthWhilePanning( true );
}

void
EarthManipulator::applySettings( Settings* settings )
{
    if ( settings )
    {
        _settings = settings;
    }
    else
    {
        configureDefaultSettings();
    }

    _task->_type = TASK_NONE;
    flushMouseEventStack();

    // apply new pitch restrictions
    double old_pitch;
    getLocalEulerAngles( 0L, &old_pitch );

    double new_pitch = osg::clampBetween( old_pitch, _settings->getMinPitch(), _settings->getMaxPitch() );

    setDistance(_distance);

    if ( new_pitch != old_pitch )
    {
        Viewpoint vp = getViewpoint();
        setViewpoint( Viewpoint(vp.getFocalPoint(), vp.getHeading(), new_pitch, vp.getRange(), vp.getSRS()) );
    }
}

EarthManipulator::Settings*
EarthManipulator::getSettings() const
{
    return _settings.get();
}

void
EarthManipulator::reinitialize()
{
    _distance = 1.0;
    _offset_x = 0.0;
    _offset_y = 0.0;
    _thrown = false;
    _dx = 0.0;
    _dy = 0.0;
    _throw_dx = 0.0;
    _throw_dy = 0.0;
    _continuous = false;
    _task = new Task();
    _last_action = ACTION_NULL;
    _srs_lookup_failed = false;
    _setting_viewpoint = false;
    _delta_t = 0.0;
    _t_factor = 1.0;
    _has_pending_viewpoint = false;
    _lastPointOnEarth.set(0.0, 0.0, 0.0);
    _arc_height = 0.0;
    _vfov = 30.0;
    _tanHalfVFOV = tan(0.5*osg::DegreesToRadians(_vfov));
}

bool
EarthManipulator::established()
{
#ifdef USE_OBSERVER_NODE_PATH
    bool needToReestablish = (!_csn.valid() || _csnObserverPath.empty()) && _node.valid();
#else
    bool needToReestablish = !_csn.valid() && _node.valid();
#endif

    if ( needToReestablish )
    {
        osg::ref_ptr<osg::Node> safeNode = _node.get();
        if ( !safeNode.valid() )
            return false;

        // find a map node.
        MapNode* mapNode = MapNode::findMapNode( safeNode.get(), _findNodeTraversalMask );
        if ( mapNode && !_settings->getDisableCollisionAvoidance() )
        {            
            _terrainCallback = new ManipTerrainCallback( this );
            mapNode->getTerrain()->addTerrainCallback( _terrainCallback );
        }   

        // find a CSN node - if there is one, we want to attach the manip to that
        _csn = findRelativeNodeOfType<osg::CoordinateSystemNode>( safeNode.get(), _findNodeTraversalMask );

        if ( _csn.valid() )
        {
            _node = _csn.get();

#if USE_OBSERVER_NODE_PATH
            _csnObserverPath.setNodePathTo( _csn.get() );
#endif

            if ( !_homeViewpoint.isSet() )
            {
                if ( _has_pending_viewpoint )
                {
                    setHomeViewpoint(
                        _pending_viewpoint,
                        _pending_viewpoint_duration_s );

                    _has_pending_viewpoint = false;
                }
                //If we have a CoordinateSystemNode and it has an ellipsoid model
                else if ( _csn->getEllipsoidModel() )
                {
                    setHomeViewpoint( 
                        Viewpoint(osg::Vec3d(-90,0,0), 0, -89,
                        _csn->getEllipsoidModel()->getRadiusEquator()*3.0 ) );
                }
                else
                {
                    setHomeViewpoint( Viewpoint(
                        safeNode->getBound().center(),
                        0, -89.9, 
                        safeNode->getBound().radius()*2.0) );
                }
            }

            if ( !_has_pending_viewpoint )
                setViewpoint( _homeViewpoint.get(), _homeViewpointDuration );
            else
                setViewpoint( _pending_viewpoint, _pending_viewpoint_duration_s );

            _has_pending_viewpoint = false;
        }

        // reset the srs cache:
        _cached_srs = NULL;
        _srs_lookup_failed = false;

        //OE_DEBUG << "[EarthManip] new CSN established." << std::endl;
    }

    return _csn.valid() && _node.valid();
}



bool
EarthManipulator::createLocalCoordFrame( const osg::Vec3d& worldPos, osg::CoordinateFrame& out_frame ) const
{
    if ( _cached_srs.valid() )
    {
        osg::Vec3d mapPos;
        _cached_srs->transformFromWorld( worldPos, mapPos ); 
        _cached_srs->createLocalToWorld( mapPos, out_frame );
    }
    return _cached_srs.valid();
}


void
EarthManipulator::setCenter( const osg::Vec3d& worldPos )
{
    _center = worldPos;
    createLocalCoordFrame( worldPos, _centerLocalToWorld );
    if ( _cached_srs.valid() )
    {
        _centerMap.fromWorld( _cached_srs.get(), worldPos );
    }
}


void
EarthManipulator::setNode(osg::Node* node)
{
    // you can only set the node if it has not already been set, OR if you are setting
    // it to NULL. (So to change it, you must first set it to NULL.) This is to prevent
    // OSG from overwriting the node after you have already set on manually.
    if ( node == 0L || !_node.valid() )
    {
        _node = node;
        _csn = 0L;

        if ( _viewCamera.valid() && _cameraUpdateCB.valid() )
        {
            _viewCamera->removeUpdateCallback( _cameraUpdateCB.get() );
            _cameraUpdateCB = 0L;
        }

        _viewCamera = 0L;

#ifdef USE_OBSERVER_NODE_PATH
        _csnObserverPath.clearNodePath();
#endif
        _csnPath.clear();
        reinitialize();

        // this might be unnecessary..
        established();
    }
}

osg::Node*
EarthManipulator::getNode()
{
    return _node.get();
}

const osgEarth::SpatialReference*
EarthManipulator::getSRS() const
{
    osg::ref_ptr<osg::Node> safeNode = _node.get();

    if ( !_cached_srs.valid() && !_srs_lookup_failed && safeNode.valid() )
    {
        EarthManipulator* nonconst_this = const_cast<EarthManipulator*>(this);

        nonconst_this->_is_geocentric = false;

        // first try to find a map node:
        osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( safeNode.get() );       
        if ( mapNode )
        {
            nonconst_this->_cached_srs = mapNode->getMap()->getProfile()->getSRS();
            nonconst_this->_is_geocentric = mapNode->isGeocentric();
        }

        // if that doesn't work, try gleaning info from a CSN:
        if ( !_cached_srs.valid() )
        {
            osg::CoordinateSystemNode* csn = osgEarth::findTopMostNodeOfType<osg::CoordinateSystemNode>( safeNode.get() );
            if ( csn )
            {
                nonconst_this->_cached_srs = osgEarth::SpatialReference::create( csn );
                nonconst_this->_is_geocentric = csn->getEllipsoidModel() != NULL;
            }
        }

        nonconst_this->_srs_lookup_failed = !_cached_srs.valid();

        if ( _cached_srs.valid() )
        {
            OE_DEBUG << "[EarthManip] cached SRS: "
                << _cached_srs->getName()
                << ", geocentric=" << _is_geocentric
                << std::endl;
        }
    }

    return _cached_srs.get();
}


static double
normalizeAzimRad( double input ) {
    if(fabs(input) > 2*osg::PI)
        input = fmod(input,2*osg::PI);
    if( input < -osg::PI ) input += osg::PI*2.0;
    if( input > osg::PI ) input -= osg::PI*2.0;
    return input;
}

osg::Matrixd
EarthManipulator::getRotation(const osg::Vec3d& point) const
{
    //The look vector will be going directly from the eye point to the point on the earth,
    //so the look vector is simply the up vector at the center point
    osg::CoordinateFrame cf;
    createLocalCoordFrame( point, cf );

    osg::Vec3d lookVector = -getUpVector(cf);

    osg::Vec3d side;

    //Force the side vector to be orthogonal to north
    osg::Vec3d worldUp(0,0,1);

    double dot = osg::absolute(worldUp * lookVector);
    if (osg::equivalent(dot, 1.0))
    {
        //We are looking nearly straight down the up vector, so use the Y vector for world up instead
        worldUp = osg::Vec3d(0, 1, 0);
        //OE_NOTICE << "using y vector victor" << std::endl;
    }

    side = lookVector ^ worldUp;
    osg::Vec3d up = side ^ lookVector;
    up.normalize();

    //We want a very slight offset
    double offset = 1e-6;

    return osg::Matrixd::lookAt( point - (lookVector * offset), point, up);
}

void
EarthManipulator::setViewpoint( const Viewpoint& vp, double duration_s )
{
    if ( !established() ) 
    {
        _pending_viewpoint = vp;
        _pending_viewpoint_duration_s = duration_s;
        _has_pending_viewpoint = true;
    }

    else if ( duration_s > 0.0 )
    {
        // xform viewpoint into map SRS
        osg::Vec3d vpFocalPoint = vp.getFocalPoint();
        if ( _cached_srs.valid() && vp.getSRS() && !_cached_srs->isEquivalentTo( vp.getSRS() ) )
        {
            vp.getSRS()->transform( vp.getFocalPoint(), _cached_srs.get(), vpFocalPoint );
        }

        _start_viewpoint = getViewpoint();
        
        _delta_heading = vp.getHeading() - _start_viewpoint.getHeading(); //TODO: adjust for crossing -180
        _delta_pitch   = vp.getPitch() - _start_viewpoint.getPitch();
        _delta_range   = vp.getRange() - _start_viewpoint.getRange();
        _delta_focal_point = vpFocalPoint - _start_viewpoint.getFocalPoint(); // TODO: adjust for lon=180 crossing

        while( _delta_heading > 180.0 ) _delta_heading -= 360.0;
        while( _delta_heading < -180.0 ) _delta_heading += 360.0;

        // adjust for geocentric date-line crossing
        if ( _is_geocentric )
        {
            while( _delta_focal_point.x() > 180.0 ) _delta_focal_point.x() -= 360.0;
            while( _delta_focal_point.x() < -180.0 ) _delta_focal_point.x() += 360.0;
        }

        // calculate an acceleration factor based on the Z differential
        double h0 = _start_viewpoint.getRange() * sin( osg::DegreesToRadians(-_start_viewpoint.getPitch()) );
        double h1 = vp.getRange() * sin( osg::DegreesToRadians( -vp.getPitch() ) );
        double dh = (h1 - h0);

        // calculate the total distance the focal point will travel and derive an arc height:
        double de;
        if ( _is_geocentric && (vp.getSRS() == 0L || vp.getSRS()->isGeographic()) )
        {
            osg::Vec3d startFP = _start_viewpoint.getFocalPoint();
            double x0,y0,z0, x1,y1,z1;
            _cached_srs->getEllipsoid()->convertLatLongHeightToXYZ(
                osg::DegreesToRadians( _start_viewpoint.y() ), osg::DegreesToRadians( _start_viewpoint.x() ), 0.0, x0, y0, z0 );
            _cached_srs->getEllipsoid()->convertLatLongHeightToXYZ(
                osg::DegreesToRadians( vpFocalPoint.y() ), osg::DegreesToRadians( vpFocalPoint.x() ), 0.0, x1, y1, z1 );
            de = (osg::Vec3d(x0,y0,z0) - osg::Vec3d(x1,y1,z1)).length();
        }
        else
        {
            de = _delta_focal_point.length();
        }

        _arc_height = 0.0;
        if ( _settings->getArcViewpointTransitions() )
        {         
            _arc_height = osg::maximum( de - fabs(dh), 0.0 );
        }

        // calculate acceleration coefficients
        if ( _arc_height > 0.0 )
        {
            // if we're arcing, we need seperate coefficients for the up and down stages
            double h_apex = 2.0*(h0+h1) + _arc_height;
            double dh2_up = fabs(h_apex - h0)/100000.0;
            _set_viewpoint_accel = log10( dh2_up );
            double dh2_down = fabs(h_apex - h1)/100000.0;
            _set_viewpoint_accel_2 = -log10( dh2_down );
        }
        else
        {
            // on arc => simple unidirectional acceleration:
            double dh2 = (h1 - h0)/100000.0;
            _set_viewpoint_accel = fabs(dh2) <= 1.0? 0.0 : dh2 > 0.0? log10( dh2 ) : -log10( -dh2 );
            if ( fabs( _set_viewpoint_accel ) < 1.0 ) _set_viewpoint_accel = 0.0;
        }
        
        if ( _settings->getAutoViewpointDurationEnabled() )
        {
            double maxDistance = _cached_srs->getEllipsoid()->getRadiusEquator();
            double ratio = osg::clampBetween( de/maxDistance, 0.0, 1.0 );
            ratio = accelerationInterp( ratio, -4.5 );
            double minDur, maxDur;
            _settings->getAutoViewpointDurationLimits( minDur, maxDur );
            duration_s = minDur + ratio*(maxDur-minDur);
        }
        
        // don't use _time_s_now; that's the time of the last event
        _time_s_set_viewpoint = osg::Timer::instance()->time_s();
        _set_viewpoint_duration_s = duration_s;

        _setting_viewpoint = true;
        
        _thrown = false;
        _task->_type = TASK_NONE;
    }
    else
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

    //TODO: streamline
            if ( !getSRS()->isEquivalentTo( vp_srs.get() ) )
            {
                osg::Vec3d local = new_center;
                // reproject the focal point if necessary:
                vp_srs->transform2D( new_center.x(), new_center.y(), getSRS(), local.x(), local.y() );
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

        setCenter( new_center );
        setDistance( vp.getRange() );

        _previousUp = getUpVector( _centerLocalToWorld );

        _centerRotation = getRotation( new_center ).getRotate().inverse();

        osg::Quat azim_q( new_azim, osg::Vec3d(0,0,1) );
        osg::Quat pitch_q( -new_pitch -osg::PI_2, osg::Vec3d(1,0,0) );

        osg::Matrix new_rot = osg::Matrixd( azim_q * pitch_q );

        _rotation = osg::Matrixd::inverse(new_rot).getRotate();
    }
}

void
EarthManipulator::updateSetViewpoint()
{
    double t = ( _time_s_now - _time_s_set_viewpoint ) / _set_viewpoint_duration_s;
    double tp = t;

    if ( t >= 1.0 )
    {
        t = tp = 1.0;
        _setting_viewpoint = false;
    }
    else if ( _arc_height > 0.0 )
    {
        if ( tp <= 0.5 )
        {
            double t2 = 2.0*tp;
            t2 = accelerationInterp( t2, _set_viewpoint_accel );
            tp = 0.5*t2;
        }
        else
        {
            double t2 = 2.0*(tp-0.5);
            t2 = accelerationInterp( t2, _set_viewpoint_accel_2 );
            tp = 0.5+(0.5*t2);
        }

        // the more smoothsteps you do, the more pronounced the fade-in/out effect        
        tp = smoothStepInterp( tp );
        tp = smoothStepInterp( tp );
    }
    else if ( t > 0.0 )
    {
        tp = accelerationInterp( tp, _set_viewpoint_accel );
        tp = smoothStepInterp( tp );
    }

    Viewpoint new_vp(
        _start_viewpoint.getFocalPoint() + _delta_focal_point * tp,
        _start_viewpoint.getHeading() + _delta_heading * tp,
        _start_viewpoint.getPitch() + _delta_pitch * tp,
        _start_viewpoint.getRange() + _delta_range * tp + (sin(osg::PI*tp)*_arc_height),
        _start_viewpoint.getSRS() );

#if 0
    OE_INFO
        << "t=" << t 
        << ", tp=" << tp
        << ", tsv=" << _time_s_set_viewpoint
        << ", now=" << _time_s_now
        << ", accel=" << _set_viewpoint_accel
        << ", accel2=" << _set_viewpoint_accel_2
        << std::endl;
#endif

    setViewpoint( new_vp );
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

    double localAzim, localPitch;
    getLocalEulerAngles( &localAzim, &localPitch );

    return Viewpoint(
        focal_point,
        osg::RadiansToDegrees( localAzim ),
        osg::RadiansToDegrees( localPitch ),
        _distance,
        getSRS() );
}


void
EarthManipulator::setTetherNode( osg::Node* node )
{
    if (_tether_node != node)
    {
        _offset_x = 0.0;
        _offset_y = 0.0;

        if ( node )
        {
            // pre-compute some tether properties. If the node is an MT, treat it
            // a little differently.

            // Find the deepest transform that has a single child. That is the one we
            // will use to calculate the tether location.
            _tether_xform = 0L;
            for( osg::Group* c = node->asGroup(); c != 0L; )
            {
                osg::Transform* xform = dynamic_cast<osg::Transform*>(c);
                if ( xform )
                    _tether_xform = xform;
                
                c = c->getNumChildren() == 1 ? c->getChild(0)->asGroup() : 0L;
            }

            if ( _tether_xform )
            {
                osg::BoundingSphere bs;

                for( unsigned i=0; i<_tether_xform->getNumChildren(); ++i )
                {
                    bs.expandBy( _tether_xform->getChild(i)->getBound() );
                }

                _tether_local_center = bs.center();
            }
            else
            {
                _tether_local_center.set( 0.0, 0.0, 0.0 );
            }
        }

        else
        {
            // rekajigger the distance, center, and pitch to legal non-tethered values:
            double pitch;
            getLocalEulerAngles(0L, &pitch);

            double maxPitch = osg::DegreesToRadians(-10.0);
            if ( pitch > maxPitch )
                rotate( 0.0, -(pitch-maxPitch) );

            osg::Vec3d eye = getMatrix().getTrans();

            // calculate the center point in front of the eye. The reference frame here 
            // is the view plane of the camera.
            osg::Matrix m( _rotation * _centerRotation );
            recalculateCenter( m );

            double newDistance = (eye-_center).length();
            setDistance( newDistance );
        }
    }

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
    osg::ref_ptr<osg::Node> safeNode = _node.get();
    if ( safeNode.valid() )
    {
		osg::ref_ptr<osgUtil::LineSegmentIntersector> lsi = NULL;

		lsi = new osgEarth::DPLineSegmentIntersector(start,end);
		//lsi = new osgUtil::LineSegmentIntersector(start,end);		

        osgUtil::IntersectionVisitor iv(lsi.get());
        iv.setTraversalMask(_intersectTraversalMask);

        safeNode->accept(iv);

        if (lsi->containsIntersections())
        {
            intersection = lsi->getIntersections().begin()->getWorldIntersectPoint();
            return true;
        }
    }
    return false;
}

void
EarthManipulator::home(double unused)
{
    handleAction( ACTION_HOME, 0, 0, 0 );
}

void
EarthManipulator::home(const osgGA::GUIEventAdapter& ,osgGA::GUIActionAdapter& us)
{
    home( 0.0 );
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
EarthManipulator::init(const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter& )
{
    flushMouseEventStack();
}


void
EarthManipulator::getUsage(osg::ApplicationUsage& usage) const
{
}

void
EarthManipulator::resetMouse( osgGA::GUIActionAdapter& aa, bool flushEventStack )
{
    if (flushEventStack)
      flushMouseEventStack();
    
    aa.requestContinuousUpdate( false );
    _thrown = false;
    _continuous = false;
    _single_axis_x = 1.0;
    _single_axis_y = 1.0;
    _lastPointOnEarth.set(0.0, 0.0, 0.0);
}


// this method will automatically install or uninstall the camera post-update callback 
// depending on whether there's a tether node.
//
// Camera updates get called AFTER the scene gets its update traversal. So, if you have
// tethering enabled (or some other feature that tracks scene graph nodes), this will
// update the camera after the scene graph. This is important in order to maintain
// frame coherency and prevent "jitter".
//
// The reason we install/uninstall instead of just leaving it there is so we can
// support OSG's "ON_DEMAND" frame scheme, which disables itself is there are any
// update callbacks in the scene graph.
void
EarthManipulator::updateCamera( osg::Camera* eventCamera )
{
    // check to see if the camera has changed, and update the callback if necessary
    if ( _viewCamera.get() != eventCamera )
    {
        if ( _cameraUpdateCB.valid() && _viewCamera.valid() )
            _viewCamera->removeUpdateCallback( _cameraUpdateCB.get() );

        _viewCamera = eventCamera;

        if ( _cameraUpdateCB.valid() && _viewCamera.valid() )
            _viewCamera->addUpdateCallback( _cameraUpdateCB.get() );
    }

    // check to see if we need to install a new camera callback:
    if ( _viewCamera.valid() )
    {
        if ( _tether_node.valid() && !_cameraUpdateCB.valid() )
        {
            _cameraUpdateCB = new CameraPostUpdateCallback(this);
            _viewCamera->addUpdateCallback( _cameraUpdateCB.get() );
        }
        else if ( !_tether_node.valid() && _cameraUpdateCB.valid() )
        {
            _viewCamera->removeUpdateCallback( _cameraUpdateCB.get() );
            _cameraUpdateCB = 0L;
        }

        // check whether a settings change requires an update:
        bool settingsChanged = _settings->outOfSyncWith(_viewCameraSettingsMonitor);

        // update the projection matrix if necessary
        osg::Viewport* vp = _viewCamera->getViewport();
        if ( vp )
        {
            const osg::Matrixd& proj = _viewCamera->getProjectionMatrix();
            bool isOrtho = ( proj(3,3) == 1. ) && ( proj(2,3) == 0. ) && ( proj(1,3) == 0. ) && ( proj(0,3) == 0.);
            CameraProjection type = _settings->getCameraProjection();

            if ( type == PROJ_PERSPECTIVE )
            {
                if ( isOrtho || settingsChanged )
                {
                    // need to switch from ortho to perspective
                    if ( isOrtho )
                        OE_INFO << LC << "Switching to PERSPECTIVE" << std::endl;

                    const osg::Vec2s& p = _settings->getCameraFrustumOffsets();
                    double px = 2.0*(((vp->width()/2)+p.x())/vp->width())-1.0;
                    double py = 2.0*(((vp->height()/2)+p.y())/vp->height())-1.0;

                    osg::Matrix projMatrix;
                    projMatrix.makePerspective(_vfov, vp->width()/vp->height(), 1.0f, 10000.0f);
                    projMatrix.postMult( osg::Matrix::translate(px, py, 0.0) );

                    _viewCamera->setProjectionMatrix( projMatrix );

                    if ( _savedCNFMode.isSet() )
                    {
                        _viewCamera->setComputeNearFarMode( *_savedCNFMode );
                        _savedCNFMode.unset();
                    }
                }
            }
            else if ( type == PROJ_ORTHOGRAPHIC )
            {
                if ( !isOrtho )
                {
                    // need to switch from perspective to ortho, so cache the VFOV of the perspective
                    // camera -- we'll need it in ortho mode to create a proper frustum.
                    OE_INFO << LC << "Switching to ORTHO" << std::endl;

                    double ar, zn, zf; // not used
                    _viewCamera->getProjectionMatrixAsPerspective(_vfov, ar, zn, zf);
                    _tanHalfVFOV = tan(0.5*(double)osg::DegreesToRadians(_vfov));
                    _savedCNFMode = _viewCamera->getComputeNearFarMode();
                    _viewCamera->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
                }
                
                double pitch;
                getLocalEulerAngles(0L, &pitch);

                // need to update the ortho projection matrix to reflect the camera distance.
                double ar = vp->width()/vp->height();
                double y = _distance * _tanHalfVFOV;
                double x = y * ar;
                double f = std::max(x,y);
                double znear = -f * 5.0;
                double zfar  =  f * (5.0 + 10.0 * sin(pitch+osg::PI_2));

                // assemble the projection matrix:
                osg::Matrixd orthoMatrix;

                // apply the offsets:
                double px = 0.0, py = 0.0;
                const osg::Vec2s& p = _settings->getCameraFrustumOffsets();
                if ( p.x() != 0 || p.y() != 0 )
                {
                    px = (2.0*x*(double)-p.x()) / (double)vp->width();
                    py = (2.0*y*(double)-p.y()) / (double)vp->height();
                }

                _viewCamera->setProjectionMatrixAsOrtho( px-x, px+x, py-y, py+y, znear, zfar );
            }
        }

        _settings->sync( _viewCameraSettingsMonitor );
    }
}


bool
EarthManipulator::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    bool handled = false;
    
    // first order of business: make sure the CSN is established.
    if ( !established() )
        return false;

    // make sure the camera callback is up to date:
    osg::View* view = aa.asView();
    updateCamera( view->getCamera() );

    double time_s_now = osg::Timer::instance()->time_s();

    if ( ea.getEventType() == osgGA::GUIEventAdapter::FRAME )
    {
        _time_s_last_frame = _time_s_now;
        _time_s_now = time_s_now;
        _delta_t = _time_s_now - _time_s_last_frame;
        // this factor adjusts for the variation of frame rate relative to 60fps
        _t_factor = _delta_t / 0.01666666666;

        if ( _has_pending_viewpoint && _node.valid() )
        {
            _has_pending_viewpoint = false;
            setViewpoint( _pending_viewpoint, _pending_viewpoint_duration_s );
            aa.requestRedraw();
        }

        else if ( _setting_viewpoint && _node.valid() )
        {
            if ( _frame_count < 2 )
                _time_s_set_viewpoint = _time_s_now;

            updateSetViewpoint();

            aa.requestContinuousUpdate( _setting_viewpoint );
        }

        else if (_thrown)
        {
            double decayFactor = 1.0 - _settings->getThrowDecayRate();

            _throw_dx = osg::absolute(_throw_dx) > osg::absolute(_dx * 0.01) ? _throw_dx * decayFactor : 0.0;
            _throw_dy = osg::absolute(_throw_dy) > osg::absolute(_dy * 0.01) ? _throw_dy * decayFactor : 0.0;

            if (_throw_dx == 0.0 && _throw_dy == 0.0)
                _thrown = false;
            else            
                handleMovementAction(_last_action._type, _throw_dx, _throw_dy, aa.asView());
        }

        if ( _continuous )
        {
            handleContinuousAction( _last_action, aa.asView() );
            aa.requestRedraw();
        }
        else
        {
            _continuous_dx = 0.0;
            _continuous_dy = 0.0;
        }
        
        if ( _task.valid() && _task->_type != TASK_NONE )
        {
            bool stillRunning = serviceTask();
            if ( stillRunning ) 
            {
                aa.requestContinuousUpdate( true );
            }
            else
            {
                // turn off the continuous, but we still need one last redraw
                // to process the final state.
                aa.requestContinuousUpdate( false );
                aa.requestRedraw();
            }
        }

        _frame_count++;

        return false;
    }


    // the camera manipulator runs last after any other event handlers. So bail out
    // if the incoming event has already been handled by another handler.
    if ( ea.getHandled() )
    {
        return false;
    }
   
    // form the current Action based on the event type:
    Action action = ACTION_NULL;
    _time_s_now = osg::Timer::instance()->time_s();

    // if tethering is active, check to see whether the incoming event 
    // will break the tether.
    if ( _tether_node.valid() )
    {
        const ActionTypeVector& atv = _settings->getBreakTetherActions();
        if ( atv.size() > 0 )
        {
            const Action& action = _settings->getAction( ea.getEventType(), ea.getButtonMask(), ea.getModKeyMask() );
            if ( std::find(atv.begin(), atv.end(), action._type) != atv.end() )
            {
                setTetherNode( 0L );
            }
        }
    }


    if ( ea.isMultiTouchEvent() )
    {
        // not a mouse event; clear the mouse queue.
        resetMouse( aa, false );
        
        // queue up a touch event set and figure out the current state:
        addTouchEvents(ea);
        TouchEvents te;
        if ( parseTouchEvents(te) )
        {
            for( TouchEvents::iterator i = te.begin(); i != te.end(); ++i )
            {
                action = _settings->getAction(i->_eventType, i->_mbmask, 0);
                
                if (action._type != ACTION_NULL)
                {
                    _last_event = i->_eventType;
                    
                    // here we adjust for action scale, global sensitivy
                    double dx = i->_dx, dy = i->_dy;
                    dx *= _settings->getMouseSensitivity();
                    dy *= _settings->getMouseSensitivity();
                    applyOptionsToDeltas( action, dx, dy );
                
                    _dx = dx;
                    _dy = dy;
                
                    if (action._type == ACTION_GOTO)
                        handlePointAction(action, ea.getX(), ea.getY(), view);
                    else
                        handleMovementAction(action._type, dx, dy, view);
                
                    aa.requestRedraw();
                }
            }
            
            handled = true;
        }
        else
        {
            // The only multitouch event we want passed on if not handled is a release
            handled = ea.getEventType() != osgGA::GUIEventAdapter::RELEASE;
            
            // if a new push occurs we want to reset the dx/dy values to stop/prevent throwing
            if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH)
                _dx = _dy = 0.0;
        }
    }

    if ( !handled )
    {
        // not a touch event; clear the touch queue.
        //_touchPointQueue.clear();

        switch( ea.getEventType() )
        {
            case osgGA::GUIEventAdapter::PUSH:
                resetMouse( aa );
                addMouseEvent( ea );
                _mouse_down_event = &ea;
                aa.requestRedraw();
                handled = true;
                break;       
            
            case osgGA::GUIEventAdapter::RELEASE:
                if ( _continuous )
                {
                    // bail out of continuous mode if necessary:
                    _continuous = false;
                    aa.requestContinuousUpdate( false );
                }
                else
                {
                    action = _last_action;
                    
                    _throw_dx = fabs(_dx) > 0.01 ? _dx : 0.0;
                    _throw_dy = fabs(_dy) > 0.01 ? _dy : 0.0;
                    
                    if (_settings->getThrowingEnabled() && ( time_s_now - _time_s_last_event < 0.05 ) && (_throw_dx != 0.0 || _throw_dy != 0.0))
                    {
                        _thrown = true;
                        aa.requestRedraw();
                        aa.requestContinuousUpdate( true );
                    }
                    else if ( isMouseClick( &ea ) )
                    {
                        addMouseEvent( ea );
                        if ( _mouse_down_event )
                        {
                            action = _settings->getAction( EVENT_MOUSE_CLICK, _mouse_down_event->getButtonMask(), _mouse_down_event->getModKeyMask() );
                            if ( handlePointAction( action, ea.getX(), ea.getY(), aa.asView() ))
                                aa.requestRedraw();
                        }
                        resetMouse( aa );
                    }
                    else
                    {
                        resetMouse( aa );
                        addMouseEvent( ea );
                    }
                }
                handled = true;
                break;
                
            case osgGA::GUIEventAdapter::DOUBLECLICK:
                // bail out of continuous mode if necessary:
                _continuous = false;
                addMouseEvent( ea );
                if (_mouse_down_event)
                {
                    action = _settings->getAction( ea.getEventType(), _mouse_down_event->getButtonMask(), _mouse_down_event->getModKeyMask() );
                    if ( handlePointAction( action, ea.getX(), ea.getY(), aa.asView() ) )
                        aa.requestRedraw();
                    resetMouse( aa );
                    handled = true;
                }
                break;

            case osgGA::GUIEventAdapter::MOVE: // MOVE not currently bindable
                //NOP
                break;

            case osgGA::GUIEventAdapter::DRAG:
                {
                    action = _settings->getAction( ea.getEventType(), ea.getButtonMask(), ea.getModKeyMask() );
                    addMouseEvent( ea );
                    bool wasContinuous = _continuous;
                    _continuous = action.getBoolOption(OPTION_CONTINUOUS, false);
                    if ( handleMouseAction( action, aa.asView() ) )
                        aa.requestRedraw();

                    if ( _continuous && !wasContinuous )
                        _last_continuous_action_time = _time_s_now;

                    aa.requestContinuousUpdate(_continuous);
                    _thrown = false;
                    handled = true;
                }
                break;

            case osgGA::GUIEventAdapter::KEYDOWN:
                if ( ea.getKey() < osgGA::GUIEventAdapter::KEY_Shift_L )
                {
                    resetMouse( aa );
                    action = _settings->getAction( ea.getEventType(), ea.getKey(), ea.getModKeyMask() );
                    if ( handleKeyboardAction( action ) )
                        aa.requestRedraw();
                    handled = true;
                }
                break;
                
            case osgGA::GUIEventAdapter::KEYUP:
                resetMouse( aa );
                _task->_type = TASK_NONE;
                handled = true;
                break;

            case osgGA::GUIEventAdapter::SCROLL:
                resetMouse( aa );
                addMouseEvent( ea );
                action = _settings->getAction( ea.getEventType(), ea.getScrollingMotion(), ea.getModKeyMask() );
                if ( handleScrollAction( action, 0.2 ) )
                    aa.requestRedraw();
                handled = true;
                break;
            default: break;
        }
    }

    // if a new task was started, request continuous updates.
    if ( _task.valid() && _task->_type != TASK_NONE )
    {
        aa.requestContinuousUpdate( true );
    }

    if ( handled && action._type != ACTION_NULL )
    {
        _last_action = action;
        _time_s_last_event = time_s_now;
    }

    return handled;
}

void
EarthManipulator::postUpdate()
{
    updateTether();
}

void
EarthManipulator::updateTether()
{
    osg::ref_ptr<osg::Node> tether_node;
    if ( _tether_node.lock(tether_node) )
    {
        osg::Matrix localToWorld;

        if ( _tether_xform )
        {
            osg::NodePathList nodePaths = _tether_xform->getParentalNodePaths();
            if ( nodePaths.empty() )
                return;

            localToWorld = osg::computeLocalToWorld( nodePaths[0] );
            if ( !localToWorld.valid() )
                return;

            setCenter( _tether_local_center * localToWorld );
        }
        else
        {
            osg::NodePathList nodePaths = tether_node->getParentalNodePaths();
            if ( nodePaths.empty() )
                return;

            localToWorld = osg::computeLocalToWorld( nodePaths[0] );
            if ( !localToWorld.valid() )
                return;

            setCenter( localToWorld.getTrans() );
        }

        _previousUp = getUpVector( _centerLocalToWorld );

        double sx = 1.0/sqrt(localToWorld(0,0)*localToWorld(0,0) + localToWorld(1,0)*localToWorld(1,0) + localToWorld(2,0)*localToWorld(2,0));
        double sy = 1.0/sqrt(localToWorld(0,1)*localToWorld(0,1) + localToWorld(1,1)*localToWorld(1,1) + localToWorld(2,1)*localToWorld(2,1));
        double sz = 1.0/sqrt(localToWorld(0,2)*localToWorld(0,2) + localToWorld(1,2)*localToWorld(1,2) + localToWorld(2,2)*localToWorld(2,2));
        localToWorld = localToWorld*osg::Matrixd::scale(sx,sy,sz);

        //Just track the center
        if (_settings->getTetherMode() == TETHER_CENTER)
        {
            _centerRotation = _centerLocalToWorld.getRotate();
        }
        //Track all rotations
        else if (_settings->getTetherMode() == TETHER_CENTER_AND_ROTATION)
        {
            _centerRotation = localToWorld.getRotate();
        }
        else if (_settings->getTetherMode() == TETHER_CENTER_AND_HEADING)
        {
            //Track just the heading
            osg::Matrixd localToFrame(localToWorld*osg::Matrixd::inverse( _centerLocalToWorld ));
            double azim = atan2(-localToFrame(0,1),localToFrame(0,0));
            osg::Quat nodeRotationRelToFrame, rotationOfFrame;
            nodeRotationRelToFrame.makeRotate(-azim,0.0,0.0,1.0);
            rotationOfFrame = _centerLocalToWorld.getRotate();
            _centerRotation = nodeRotationRelToFrame*rotationOfFrame;
        }
    }
}

bool
EarthManipulator::serviceTask()
{
    if ( _task.valid() && _task->_type != TASK_NONE )
    {
        double dt = _time_s_now - _task->_time_last_service;

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
            default: break;
        }

        _task->_duration_s -= dt;
        _task->_time_last_service = _time_s_now;

        if ( _task->_duration_s <= 0.0 )
        {
            _task->_type = TASK_NONE;
        }
    }

    // returns true if the task is still running.
    return _task.valid() && _task->_type != TASK_NONE;
}

bool
EarthManipulator::isMouseMoving()
{
    if (_ga_t0.get()==NULL || _ga_t1.get()==NULL) return false;

    static const float velocity = 0.1f;

    float dx = _ga_t0->getXnormalized()-_ga_t1->getXnormalized();
    float dy = _ga_t0->getYnormalized()-_ga_t1->getYnormalized();
    float len = sqrtf(dx*dx+dy*dy);
    //float dt = _ga_t0->getTime()-_ga_t1->getTime();

    return len > _delta_t * velocity;
}

bool
EarthManipulator::isMouseClick( const osgGA::GUIEventAdapter* mouse_up_event ) const
{
    if ( mouse_up_event == NULL || _mouse_down_event == NULL ) return false;

    static const float velocity = 0.1f;

    float dx = mouse_up_event->getXnormalized() - _mouse_down_event->getXnormalized();
    float dy = mouse_up_event->getYnormalized() - _mouse_down_event->getYnormalized();
    float len = sqrtf( dx*dx + dy*dy );
    float dt = mouse_up_event->getTime( ) - _mouse_down_event->getTime();

    return len < dt * velocity;
}

void
EarthManipulator::flushMouseEventStack()
{
    _ga_t1 = NULL;
    _ga_t0 = NULL;
    //_touchPointQueue.clear();
}


void
EarthManipulator::addMouseEvent(const osgGA::GUIEventAdapter& ea)
{
    _ga_t1 = _ga_t0;
    _ga_t0 = &ea;
    //_touchPointQueue.clear();
}

void
EarthManipulator::addTouchEvents(const osgGA::GUIEventAdapter& ea)
{
    _ga_t1 = _ga_t0;
    _ga_t0 = &ea;
    
    // first, push the old event to the back of the queue.
    while ( _touchPointQueue.size() > 1 )
        _touchPointQueue.pop_front();

    // queue any new events.
    if ( ea.isMultiTouchEvent() )
    {
        osgGA::GUIEventAdapter::TouchData* data = ea.getTouchData();

        _touchPointQueue.push_back(MultiTouchPoint());
        MultiTouchPoint& ev = _touchPointQueue.back();

        for( unsigned i=0; i<data->getNumTouchPoints(); ++i )
        {
            osgGA::GUIEventAdapter::TouchData::TouchPoint tp = data->get(i);
            ev.push_back(tp);
        }
    }
}

bool
EarthManipulator::parseTouchEvents( TouchEvents& output )
{
    const float sens = 0.005f;    
        
    if (_touchPointQueue.size() == 2 )
    {
        if (_touchPointQueue[0].size()   == 2 &&     // two fingers
            _touchPointQueue[1].size()   == 2)       // two fingers
        {
            MultiTouchPoint& p0 = _touchPointQueue[0];
            MultiTouchPoint& p1 = _touchPointQueue[1];

            if (p0[0].phase != osgGA::GUIEventAdapter::TOUCH_ENDED &&
                p1[0].phase != osgGA::GUIEventAdapter::TOUCH_ENDED &&
                p0[1].phase == osgGA::GUIEventAdapter::TOUCH_MOVED &&
                p1[1].phase == osgGA::GUIEventAdapter::TOUCH_MOVED)
            {
                // gather information about what happened:
                float dx[2], dy[2];
                for( int i=0; i<2; ++i )
                {
                    dx[i] = p1[i].x - p0[i].x;
                    dy[i] = p1[i].y - p0[i].y;
                }
                osg::Vec2f vec0 = osg::Vec2f(p0[1].x,p0[1].y)-osg::Vec2f(p0[0].x,p0[0].y);
                osg::Vec2f vec1 = osg::Vec2f(p1[1].x,p1[1].y)-osg::Vec2f(p1[0].x,p1[0].y);
                float deltaDistance = vec1.length() - vec0.length();
                
                float angle[2];
                angle[0] = atan2(p0[0].y - p0[1].y, p0[0].x - p0[1].x);
                angle[1] = atan2(p1[0].y - p1[1].y, p1[0].x - p1[1].x);
                float da = angle[0] - angle[1];

                float dragThres = 2.0f;         

                // now see if that corresponds to any touch events:
                
                if (osg::equivalent( vec0.x(), vec1.x(), dragThres) && 
                    osg::equivalent( vec0.y(), vec1.y(), dragThres))
                {                    
                    // two-finger drag.
                    output.push_back(TouchEvent());
                    TouchEvent& ev = output.back();
                    ev._eventType = EVENT_MULTI_DRAG;
                    ev._dx = 0.5 * (dx[0]+dx[1]) * sens;
                    ev._dy = 0.5 * (dy[0]+dy[1]) * sens;
                }                                                
                else
                {                                 
                    // otherwise it's a pinch and/or a zoom.  You can do them together.
                    if (fabs(deltaDistance) > 1.0)
                    {
                        // distance between the fingers changed: a pinch.
                        output.push_back(TouchEvent());
                        TouchEvent& ev = output.back();
                        ev._eventType = EVENT_MULTI_PINCH;
                        ev._dx = 0.0, ev._dy = deltaDistance * -sens;
                    }

                    if (fabs(da) > 0.01)
                    {
                        // angle between vectors changed: a twist.
                        output.push_back(TouchEvent());
                        TouchEvent& ev = output.back();
                        ev._eventType = EVENT_MULTI_TWIST;                    
                        ev._dx = da;
                        //ev._dy = 0.5 * (dy[0]+dy[1]) * sens;
                        ev._dy = 0.0;
                    }
                }             
            }
        }

        else if (_touchPointQueue[0].size() >= 1 &&     // one finger
                 _touchPointQueue[1].size() >= 1)       // one finger
        {
            MultiTouchPoint& p0 = _touchPointQueue[0];
            MultiTouchPoint& p1 = _touchPointQueue[1];

            if (p1[0].tapCount == 2)
            {
                // double tap
                output.push_back(TouchEvent());
                TouchEvent& ev = output.back();
                ev._eventType = EVENT_MOUSE_DOUBLE_CLICK;
                ev._mbmask = osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON;
                ev._dx = 0.0;
                ev._dy = 0.0;
            }
            else if ((p0[0].phase != osgGA::GUIEventAdapter::TOUCH_ENDED &&
                      p1[0].phase == osgGA::GUIEventAdapter::TOUCH_MOVED ))
            {
                output.push_back(TouchEvent());
                TouchEvent& ev = output.back();
                ev._eventType = EVENT_MOUSE_DRAG;
                ev._mbmask = osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON;
                ev._dx =  (p1[0].x - p0[0].x) * sens;
                ev._dy = -(p1[0].y - p0[0].y) * sens;
            }
        }
    }

    return output.size() > 0;
}

void
EarthManipulator::setByMatrix(const osg::Matrixd& matrix)
{
    osg::Vec3d lookVector(- matrix(2,0),-matrix(2,1),-matrix(2,2));
    osg::Vec3d eye(matrix(3,0),matrix(3,1),matrix(3,2));

    _centerRotation = makeCenterRotation(_center);

    osg::ref_ptr<osg::Node> safeNode = _node.get();

    if ( !safeNode.valid() )
    {
        setCenter( eye + lookVector );
        setDistance( lookVector.length() );
        _rotation = matrix.getRotate().inverse() * _centerRotation.inverse();	
        return;
    }

    // need to reintersect with the terrain
    const osg::BoundingSphere& bs = safeNode->getBound();
    float distance = (eye-bs.center()).length() + safeNode->getBound().radius();
    osg::Vec3d start_segment = eye;
    osg::Vec3d end_segment = eye + lookVector*distance;
    
    osg::Vec3d ip;
    bool hitFound = false;
    if (intersect(start_segment, end_segment, ip))
    {
        setCenter( ip );
        _centerRotation = makeCenterRotation(_center);
        setDistance( (eye-ip).length());

        osg::Matrixd rotation_matrix = osg::Matrixd::translate(0.0,0.0,-_distance)*
                                       matrix*
                                       osg::Matrixd::translate(-_center);
        _rotation = rotation_matrix.getRotate() * _centerRotation.inverse();
        hitFound = true;
    }

    if (!hitFound)
    {
        osg::CoordinateFrame eyeCoordFrame;
        createLocalCoordFrame( eye, eyeCoordFrame );

        osg::Vec3d eyeUp = getUpVector(eyeCoordFrame);

        if (intersect(eye + eyeUp*distance, eye - eyeUp*distance, ip))
        {
            setCenter( ip );
            _centerRotation = makeCenterRotation(_center);
            setDistance((eye-ip).length());
            _rotation.set(0,0,0,1);
            hitFound = true;
        }
    }

    //osg::CoordinateFrame coordinateFrame;
    //createLocalCoordFrame( _center, coordinateFrame );
    _previousUp = getUpVector(_centerLocalToWorld);

    recalculateRoll();
    //recalculateLocalPitchAndAzimuth();
}

osg::Matrixd
EarthManipulator::getMatrix() const
{
    return osg::Matrixd::translate(-_offset_x,-_offset_y,_distance)*
           osg::Matrixd::rotate(_rotation)*
           osg::Matrixd::rotate(_centerRotation)*
           osg::Matrixd::translate(_center);
}

osg::Matrixd
EarthManipulator::getInverseMatrix() const
{
    return osg::Matrixd::translate(-_center)*
           osg::Matrixd::rotate(_centerRotation.inverse() ) *
           osg::Matrixd::rotate(_rotation.inverse())*
           osg::Matrixd::translate(_offset_x,_offset_y,-_distance);
}

void
EarthManipulator::setByLookAt(const osg::Vec3d& eye,const osg::Vec3d& center,const osg::Vec3d& up)
{
    osg::ref_ptr<osg::Node> safeNode = _node.get();

    if ( !safeNode.valid() ) return;

    // compute rotation matrix
    osg::Vec3d lv(center-eye);
    setDistance( lv.length() );
    setCenter( center );

    if (_node.valid())
    {
        bool hitFound = false;

        double distance = lv.length();
        double maxDistance = distance+2*(eye-safeNode->getBound().center()).length();
        osg::Vec3d farPosition = eye+lv*(maxDistance/distance);
        osg::Vec3d endPoint = center;
        for(int i=0;
            !hitFound && i<2;
            ++i, endPoint = farPosition)
        {
            // compute the intersection with the scene.s
            
            osg::Vec3d ip;
            if (intersect(eye, endPoint, ip))
            {
                setCenter( ip );
                setDistance( (ip-eye).length() );
                hitFound = true;
            }
        }
    }

    // note LookAt = inv(CF)*inv(RM)*inv(T) which is equivalent to:
    // inv(R) = CF*LookAt.

    osg::Matrixd rotation_matrix = osg::Matrixd::lookAt(eye,center,up);

    _centerRotation = getRotation( _center ).getRotate().inverse();
    _rotation = rotation_matrix.getRotate().inverse() * _centerRotation.inverse();	
    
    _previousUp = getUpVector(_centerLocalToWorld);

    recalculateRoll();
}


void
EarthManipulator::recalculateCenter( const osg::CoordinateFrame& frame )
{
    osg::ref_ptr<osg::Node> safeNode = _node.get();
    if ( safeNode.valid() )
    {
        bool hitFound = false;

        //osg::Vec3d eye = getMatrix().getTrans();

        // need to reintersect with the terrain
        double ilen = safeNode->getBound().radius()*0.25f;

        osg::Vec3d up = getUpVector(frame);

        osg::Vec3d ip1;
        osg::Vec3d ip2;
        // extend coordonate to fall on the edge of the boundingbox see http://www.osgearth.org/ticket/113
        bool hit_ip1 = intersect(_center - up * ilen * 0.1, _center + up * ilen, ip1);
        bool hit_ip2 = intersect(_center + up * ilen * 0.1, _center - up * ilen, ip2);
        if (hit_ip1)
        {
            if (hit_ip2)
            {
                setCenter( (_center-ip1).length2() < (_center-ip2).length2() ? ip1 : ip2 );
                hitFound = true;
            }
            else
            {
                setCenter( ip1 );
                hitFound = true;
            }
        }
        else if (hit_ip2)
        {
            setCenter( ip2 );
            hitFound = true;
        }

        if (hitFound)
        {
#if 0
            // recalculate the distance based on the current eyepoint:
            double oldDistance = _distance;
            double newDistance = (eye-_center).length();
            setDistance( newDistance );
            OE_NOTICE << "OLD = " << oldDistance << ", NEW = " << newDistance << std::endl;
#endif
        }

        else // if (!hitFound)
        {
            // ??
            //OE_DEBUG<<"EarthManipulator unable to intersect with terrain."<<std::endl;
        }
    }
}


void
EarthManipulator::pan( double dx, double dy )
{
    if (!_tether_node.valid())
    {
        double scale = -0.3f*_distance;
        double old_azim;
        getLocalEulerAngles( &old_azim );

        osg::Matrixd rotation_matrix;
        rotation_matrix.makeRotate( _rotation * _centerRotation  );

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
        osg::CoordinateFrame oldCenterLocalToWorld = _centerLocalToWorld;

        // move the cente rpoint:
        setCenter( _center + dv );

        // need to recompute the intersection point along the look vector.
        osg::ref_ptr<osg::Node> safeNode = _node.get();
        if (safeNode.valid())
        {
            recalculateCenter( oldCenterLocalToWorld );

            osg::Vec3d new_localUp = getUpVector( _centerLocalToWorld );

            osg::Quat pan_rotation;
            pan_rotation.makeRotate( localUp, new_localUp );

            if ( !pan_rotation.zeroRotation() )
            {
                _centerRotation = _centerRotation * pan_rotation;
                _previousUp = new_localUp;
            }
            else
            {
                //OE_DEBUG<<"New up orientation nearly inline - no need to rotate"<<std::endl;
            }

            if ( _settings->getLockAzimuthWhilePanning() )
            {
                double new_azim;
                getLocalEulerAngles( &new_azim );

                double delta_azim = new_azim - old_azim;
                //OE_NOTICE << "DeltaAzim" << delta_azim << std::endl;

                osg::Quat q;
                q.makeRotate( delta_azim, new_localUp );
                if ( !q.zeroRotation() )
                {
                    _centerRotation = _centerRotation * q;
                }
            }
        }

        //recalculateLocalPitchAndAzimuth();
    }
    else
    {
        double scale = _distance;
        _offset_x += dx * scale;
        _offset_y += dy * scale;

        //Clamp values within range
        if (_offset_x < -_settings->getMaxXOffset()) _offset_x = -_settings->getMaxXOffset();
        if (_offset_y < -_settings->getMaxYOffset()) _offset_y = -_settings->getMaxYOffset();
        if (_offset_x > _settings->getMaxXOffset()) _offset_x = _settings->getMaxXOffset();
        if (_offset_y > _settings->getMaxYOffset()) _offset_y = _settings->getMaxYOffset();
    }
}

void
EarthManipulator::rotate( double dx, double dy )
{
    //OE_NOTICE << "rotate " << dx <<", " << dy << std::endl;
    // clamp the local pitch delta; never allow the pitch to hit -90.

    bool tether = _tether_node.valid();
    double minp = osg::DegreesToRadians( osg::clampAbove(_settings->getMinPitch(), -89.9) );
    double maxp = osg::DegreesToRadians( osg::clampBelow(_settings->getMaxPitch(), tether? 89.9 : -1.0) );

#if 0
    OE_NOTICE << LC 
        << "LocalPitch=" << osg::RadiansToDegrees(_local_pitch)
        << ", dy=" << osg::RadiansToDegrees(dy)
        << ", dy+lp=" << osg::RadiansToDegrees(_local_pitch+dy)
        << ", limits=" << osg::RadiansToDegrees(minp) << "," << osg::RadiansToDegrees(maxp)
        << std::endl;
#endif

    // clamp pitch range:
    double oldPitch;
    getLocalEulerAngles( 0L, &oldPitch );

    if ( dy + oldPitch > maxp || dy + oldPitch < minp )
        dy = 0;

    osg::Matrix rotation_matrix;
    rotation_matrix.makeRotate(_rotation);

    osg::Vec3d lookVector = -getUpVector(rotation_matrix);
    osg::Vec3d sideVector = getSideVector(rotation_matrix);
    osg::Vec3d upVector = getFrontVector(rotation_matrix);

    osg::Vec3d localUp(0.0f,0.0f,1.0f);

    osg::Vec3d forwardVector = localUp^sideVector;
    sideVector = forwardVector^localUp;

    forwardVector.normalize();
    sideVector.normalize();

    osg::Quat rotate_elevation;
    rotate_elevation.makeRotate(dy,sideVector);

    osg::Quat rotate_azim;
    rotate_azim.makeRotate(-dx,localUp);

    _rotation = _rotation * rotate_elevation * rotate_azim;
}

void
EarthManipulator::zoom( double dx, double dy )
{    
    double scale = 1.0f + dy;
    setDistance( _distance * scale );    
}


namespace
{
    // osg::View::getCameraContainingPosition has a bug in it. If the camera's current event
    // state is not up to date (after a window resize, for example), it still uses that event
    // state to get the window's current size instead of using the Viewport.
    //
    // This version works around that

    const osg::Camera*
    getCameraContainingPosition(osgViewer::View* view, float x, float y, float& out_local_x, float& out_local_y)
    {
        osg::Camera* camera = view->getCamera();
        osg::Viewport* viewport = camera->getViewport();

        if ( camera->getGraphicsContext() && viewport )
        {
            double new_x = x;
            double new_y = y;
            
            const double epsilon = 0.5;

            if (
                new_x >= (viewport->x()-epsilon) && new_y >= (viewport->y()-epsilon) &&
                new_x < (viewport->x()+viewport->width()-1.0+epsilon) && new_y <= (viewport->y()+viewport->height()-1.0+epsilon) )
            {
                out_local_x = new_x;
                out_local_y = new_y;
                return camera;
            }
        }

        return view->getCameraContainingPosition(x, y, out_local_x, out_local_y);
    }
}


bool
EarthManipulator::screenToWorld(float x, float y, osg::View* theView, osg::Vec3d& out_coords ) const
{
    osgViewer::View* view = dynamic_cast<osgViewer::View*>( theView );
    if ( !view || !view->getCamera() )
        return false;

    osg::NodePath nodePath;
    _csnObserverPath.getNodePath(nodePath);
    if ( nodePath.empty() )
        return false;

    float local_x, local_y = 0.0;
    const osg::Camera* camera = getCameraContainingPosition(view, x, y, local_x, local_y);
    if ( !camera )
        return false;

    osg::Matrixd matrix;
    if (nodePath.size()>1)
    {
        osg::NodePath prunedNodePath(nodePath.begin(),nodePath.end()-1);
        matrix = osg::computeLocalToWorld(prunedNodePath);
    }

    matrix.postMult(camera->getViewMatrix());
    matrix.postMult(camera->getProjectionMatrix());

    double zNear = -1.0;
    double zFar = 1.0;
    if (camera->getViewport())
    {
        matrix.postMult(camera->getViewport()->computeWindowMatrix());
        zNear = 0.0;
        zFar = 1.0;
    }

    osg::Matrixd inverse;
    inverse.invert(matrix);

    osg::Vec3d startVertex = osg::Vec3d(local_x,local_y,zNear) * inverse;
    osg::Vec3d endVertex = osg::Vec3d(local_x,local_y,zFar) * inverse;

	osg::ref_ptr<osgUtil::LineSegmentIntersector> picker = NULL;

	picker = new osgEarth::DPLineSegmentIntersector(osgUtil::Intersector::MODEL, startVertex, endVertex);	
	//picker = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::MODEL, startVertex, endVertex);	

    osgUtil::IntersectionVisitor iv(picker.get());
    iv.setTraversalMask(_intersectTraversalMask);
    nodePath.back()->accept(iv);

    if ( picker->containsIntersections() )
    {
        osgUtil::LineSegmentIntersector::Intersections& results = picker->getIntersections();
        out_coords = results.begin()->getWorldIntersectPoint();
        return true;
    }

    return false;
}


void
EarthManipulator::setDistance( double distance )
{
    _distance = osg::clampBetween( distance, _settings->getMinDistance(), _settings->getMaxDistance() );
}

void
EarthManipulator::dumpActionInfo( const EarthManipulator::Action& action, osg::NotifySeverity level ) const
{
    osgEarth::notify(level) << "action: " << s_actionNames[action._type] << "; options: ";
    for( ActionOptions::const_iterator i = action._options.begin(); i != action._options.end(); ++i )
    {
        const ActionOption& option = *i;
        std::string val;
        if ( s_actionOptionTypes[option.option()] == 0 )
            val = option.boolValue() ? "true" : "false";
        else
            val = toString<double>(option.doubleValue());

        osgEarth::notify(level)
            << s_actionOptionNames[option.option()] << "=" << val << ", ";
    }
    osgEarth::notify(level) << std::endl;        
}

void
EarthManipulator::handleMovementAction( const ActionType& type, double dx, double dy, osg::View* view )
{
    switch( type )
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

    case ACTION_EARTH_DRAG:
        if (_thrown)
          pan(dx*0.5, dy*0.5);  //TODO: create proper drag throwing instead of panning trick
        else
          drag( dx, dy, view );
        break;
    default:break;
    }
}

bool
EarthManipulator::handlePointAction( const Action& action, float mx, float my, osg::View* view )
{
    //Exit early if the action is null
    if (action._type == ACTION_NULL)
        return true;

    osg::Vec3d point;
    if ( screenToWorld( mx, my, view, point ))
    {
        switch( action._type )
        {
            case ACTION_GOTO:
            {
                Viewpoint here = getViewpoint();

                if ( !here.getSRS() )
                    return false;

                osg::Vec3d pointVP;
                here.getSRS()->transformFromWorld(point, pointVP);

                //OE_NOTICE << "X=" << pointVP.x() << ", Y=" << pointVP.y() << std::endl;

                here.setFocalPoint( pointVP );

                double duration_s = action.getDoubleOption(OPTION_DURATION, 1.0);
                double range_factor = action.getDoubleOption(OPTION_GOTO_RANGE_FACTOR, 1.0);

                here.setRange( here.getRange() * range_factor );

                setViewpoint( here, duration_s );
            }
            break;
            default:
            break;
        }
    }
    return true;
}

void
EarthManipulator::handleContinuousAction( const Action& action, osg::View* view )
{
    double t_factor = (_time_s_now - _last_continuous_action_time)/0.016666666;
    _last_continuous_action_time = _time_s_now;
    handleMovementAction( action._type, _continuous_dx * t_factor, _continuous_dy * t_factor, view );
}

void
EarthManipulator::applyOptionsToDeltas( const Action& action, double& dx, double& dy )
{
    dx *= action.getDoubleOption( OPTION_SCALE_X, 1.0 );
    dy *= action.getDoubleOption( OPTION_SCALE_Y, 1.0 );

    if ( action.getBoolOption( OPTION_SINGLE_AXIS, false ) == true )
    {
        if ( osg::absolute(dx) > osg::absolute(dy) )
            dy = 0.0;
        else
            dx = 0.0;
    }
}

bool
EarthManipulator::handleMouseAction( const Action& action, osg::View* view )
{
    // return if less then two events have been added.
    if (_ga_t0.get()==NULL || _ga_t1.get()==NULL) return false;

    //if ( osgEarth::getNotifyLevel() > osg::INFO )
    //    dumpActionInfo( action, osg::DEBUG_INFO );

    double dx = _ga_t0->getXnormalized()-_ga_t1->getXnormalized();
    double dy = _ga_t0->getYnormalized()-_ga_t1->getYnormalized();

    // return if there is no movement.
    if (dx==0 && dy==0) return false;

    // here we adjust for action scale, global sensitivy
    dx *= _settings->getMouseSensitivity();
    dy *= _settings->getMouseSensitivity();

    applyOptionsToDeltas( action, dx, dy );

    // in "continuous" mode, we accumulate the deltas each frame - thus
    // the deltas act more like speeds.
    if ( _continuous )
    {
        _continuous_dx += dx * 0.01;
        _continuous_dy += dy * 0.01;
    }
    else
    {
        
        _dx = dx;
        _dy = dy;
        handleMovementAction( action._type, dx, dy, view );
    }

    return true;
}

bool
EarthManipulator::handleMouseClickAction( const Action& action )
{
    //TODO.
    return false;
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
    default: break;
    }

    dx *= _settings->getKeyboardSensitivity();
    dy *= _settings->getKeyboardSensitivity();

    applyOptionsToDeltas( action, dx, dy );

    return handleAction( action, dx, dy, duration );
}

bool
EarthManipulator::handleScrollAction( const Action& action, double duration )
{
    const double scrollFactor = 1.5;

    double dx = 0, dy = 0;

    switch( action._dir )
    {
    case DIR_LEFT:  dx =  1; break;
    case DIR_RIGHT: dx = -1; break;
    case DIR_UP:    dy = -1; break;
    case DIR_DOWN:  dy =  1; break;
    default: break;
    }

    dx *= scrollFactor * _settings->getScrollSensitivity();
    dy *= scrollFactor * _settings->getScrollSensitivity();

    applyOptionsToDeltas( action, dx, dy );

    return handleAction( action, dx, dy, duration );
}

bool
EarthManipulator::handleAction( const Action& action, double dx, double dy, double duration )
{
    bool handled = true;

    //if ( osgEarth::getNotifyLevel() > osg::INFO )
    //    dumpActionInfo( action, osg::DEBUG_INFO );

    //OE_NOTICE << "action=" << action << ", dx=" << dx << ", dy=" << dy << std::endl;

    switch( action._type )
    {
    case ACTION_HOME:
        if ( _homeViewpoint.isSet() )
        {
            setViewpoint( _homeViewpoint.value(), _homeViewpointDuration );
        }
        break;


    case ACTION_PAN:
    case ACTION_PAN_LEFT:
    case ACTION_PAN_RIGHT:
    case ACTION_PAN_UP:
    case ACTION_PAN_DOWN:
        _task->set( TASK_PAN, dx, dy, duration, _time_s_now );
        break;

    case ACTION_ROTATE:
    case ACTION_ROTATE_LEFT:
    case ACTION_ROTATE_RIGHT:
    case ACTION_ROTATE_UP:
    case ACTION_ROTATE_DOWN:
        _task->set( TASK_ROTATE, dx, dy, duration, _time_s_now );
        break;

    case ACTION_ZOOM:
    case ACTION_ZOOM_IN:
    case ACTION_ZOOM_OUT:
        _task->set( TASK_ZOOM, dx, dy, duration, _time_s_now );
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
    rotation_matrix.makeRotate(_centerRotation);

    osg::Vec3d lookVector = -getUpVector(rotation_matrix);
    osg::Vec3d upVector = getFrontVector(rotation_matrix);

    osg::Vec3d localUp = getUpVector(_centerLocalToWorld);

    osg::Vec3d sideVector = lookVector ^ localUp;

    if (sideVector.length()<0.1)
    {
        //OE_INFO<<"Side vector short "<<sideVector.length()<<std::endl;

        sideVector = upVector^localUp;
        sideVector.normalize();

    }

    osg::Vec3d newUpVector = sideVector^lookVector;
    newUpVector.normalize();

    osg::Quat rotate_roll;
    rotate_roll.makeRotate(upVector,newUpVector);

    if (!rotate_roll.zeroRotation())
    {
        _centerRotation = _centerRotation * rotate_roll;
    }
}

void
EarthManipulator::getLocalEulerAngles( double* out_azim, double* out_pitch ) const
{
    osg::Matrix m = getMatrix() * osg::Matrixd::inverse(_centerLocalToWorld);
    osg::Vec3d look = -getUpVector( m );
    osg::Vec3d up   =  getFrontVector( m );
    
    look.normalize();
    up.normalize();

    if ( out_azim )
    {
        if ( look.z() < -0.9 )
            *out_azim = atan2( up.x(), up.y() );
        else if ( look.z() > 0.9 )
            *out_azim = atan2( -up.x(), -up.y() );
        else
            *out_azim = atan2( look.x(), look.y() );

        *out_azim = normalizeAzimRad( *out_azim );
    }

    if ( out_pitch )
    {
        *out_pitch = asin( look.z() );
    }
}


void
EarthManipulator::setHomeViewpoint( const Viewpoint& vp, double duration_s )
{
    _homeViewpoint = vp;
    _homeViewpointDuration = duration_s;
}


namespace // Utility functions for drag()
{
    // Find the point on a line, specified by p1 and v, closest to another
    // point.
    osg::Vec3d closestPtOnLine(const osg::Vec3d& p1, const osg::Vec3d& v,
                               const osg::Vec3d& p)
    {
        double u = (p - p1) * v / v.length2();
        return p1 + v * u;
    }

    // Intersection of line and plane
    bool findIntersectionWithPlane(const osg::Vec3d& normal, const osg::Vec3d& pt,
                                   const osg::Vec3d& p1, const osg::Vec3d& v,
                                   osg::Vec3d& result)
    {
        double denom = normal * v;
        if (osg::equivalent(0, denom))
            return false;
        double u = normal * (pt - p1) / denom;
        result = p1 + v * u;
        return true;
    }

    // Circle of intersection of two spheres. The circle is in the plane
    // normal to the line between the centers.
    bool sphereInterection(const osg::Vec3d& p0, double r0,
                           const osg::Vec3d& p1, double r1,
                           osg::Vec3d& resultCenter, double& r)
    {
        using namespace osg;
        Vec3d ptvec = (p1 - p0);
        double d = ptvec.normalize();
        if (d > r0 + r1)
            return false;               // spheres are too far apart
        else if (d < fabs(r0 - r1))
            return false;               // One sphere is contained in the other
        else if (equivalent(0, d) && equivalent(r0, r1))
        {
            resultCenter = p0;
            r = r0;
            return true;              // circles are coincident.
        }
        // distance from p0 to the line through the interection points
        double a = (r0 * r0 - r1 * r1 + d * d) / (2 * d);
        // distance from bisection of that line to the intersections
        resultCenter = p0 + ptvec * a;
        r = sqrt(r0 * r0 - a * a);
        return true;
    }

    // Find a point on the sphere (center, radius) through which the tangent
    // through pt passes. The point lies in the plane defined by
    //the line pt->center and ray.
    osg::Vec3d calcTangentPoint(const osg::Vec3d& pt, const osg::Vec3d& center,
                                double radius, const osg::Vec3d& ray)
    {
        using namespace osg;
        // new sphere with center at midpoint between pt and input sphere
        Vec3d center2 = (pt + center) / 2.0;
        double rad2 = (pt - center2).length();
        Vec3d resCtr;
        double resRad;
        // Use Thales' theorem, which states that a triangle inscribed in
        // a circle, with two points on a diameter of the circle and the
        // third on the circle, is a right triangle. Since one endpoint is
        // the center of the original sphere (the earth) and the other is
        // pt, we can get our tangent from that.
        bool valid = sphereInterection(center, radius, center2, rad2, resCtr,
                                       resRad);
        if (!valid)
            return Vec3d(0.0, 0.0, 0.0);
        // Get the tangent point that lies in the plane of the ray and the
        // center line. The sequence of cross products gives us the point
        // that is closest to the ray, rather than the one on the other
        // side of the sphere.
        Vec3d toCenter = center - pt;
        toCenter.normalize();
        Vec3d normal = ray ^ toCenter;
        normal.normalize();
        Vec3d radial = toCenter ^ normal;
        radial = radial * resRad;
        Vec3d result = resCtr + radial;
        return result;
        
    }
    // Calculate a pointer click in eye coordinates
    osg::Vec3d getWindowPoint(osgViewer::View* view, float x, float y)
    {
        float local_x, local_y;
        const osg::Camera* camera
            = view->getCameraContainingPosition(x, y, local_x, local_y);
        if (!camera)
            camera = view->getCamera();
        osg::Matrix winMat;
        if (camera->getViewport())
            winMat = camera->getViewport()->computeWindowMatrix();
        osg::Matrix projMat = camera->getProjectionMatrix();
        // ray from eye through pointer in camera coordinate system goes
        // from origin through transformed pointer coordinates
        osg::Matrix win2camera = projMat * winMat;
        win2camera.invert(win2camera);
        osg::Vec4d winpt4 = osg::Vec4d(x, y, 0.0, 1.0) * win2camera;
        winpt4 = winpt4 / winpt4.w();
        return osg::Vec3d(winpt4.x(), winpt4.y(), winpt4.z());
    }

    // Decompose  _center and _centerRotation into a longitude rotation
    // and a latitude rotation + translation in the longitudinal plane.

    void decomposeCenter(const osg::Vec3d& center, const osg::Quat& centerRotation,
                         osg::Matrix& Me, osg::Matrix& Mlon)
    {
        using namespace osg;
        Mlon.makeIdentity();
        Matrix Mtotal(centerRotation);
        Mtotal.setTrans(center);
        // Use the X axis to determine longitude rotation. Due to the
        // OpenGL camera rotation, this axis will be the Y axis of the
        // longitude matrix.
        Mlon(1, 0) = Mtotal(0, 0);  Mlon(1, 1) = Mtotal(0, 1);
        // X axis is rotated 90 degrees, obviously
        Mlon(0, 0) = Mlon(1, 1);  Mlon(0, 1) = -Mlon(1, 0);
        Matrix MlonInv = Matrixd::inverse(Mlon);
        Me = Mtotal * MlonInv;
    }

    osg::Matrixd rotateAroundPoint(const osg::Vec3d& pt, double theta,
                                   const osg::Vec3d& axis)
    {
        return (osg::Matrixd::translate(pt)
                * osg::Matrixd::rotate(theta, axis)
                * osg::Matrixd::translate(pt * -1.0));
    }
}

// Theory of operation for the manipulator drag motion
//
// The mouse drag is transformed to a vector on the surface of the
// earth i.e., in the surface plane at the start of the drag. This is
// treated as a displacement along the arc of a great circle. The
// earth will be rotated by the equivalent rotation around the axis of
// the circle. However, the manipulator controls the camera, not the
// earth, so the camera's placement matrix (inverse view matrix)
// should be rotated by the inverse of the calculated
// rotation. EarthManipulator represents the placement matrix as the
// concatenation of 4 transformations: distance from focal point,
// local heading and pitch, rotation to frame of focal point, focal
// point. To change the camera placement we rotate the frame rotation
// (_centerRotation) and focal point (_center).
//
// When the start or end drag click is not on the earth, we choose the
// nearest tangent point on the earth to the ray from the eye and
// proceed.

void
EarthManipulator::drag(double dx, double dy, osg::View* theView)
{
    using namespace osg;
    const osg::Vec3d zero(0.0, 0.0, 0.0);
    if (_last_action._type != ACTION_EARTH_DRAG)
        _lastPointOnEarth = zero;

    ref_ptr<osg::CoordinateSystemNode> csnSafe = _csn.get();
    double radiusEquator = csnSafe.valid() ? csnSafe->getEllipsoidModel()->getRadiusEquator() : 6378137.0;

    osgViewer::View* view = dynamic_cast<osgViewer::View*>(theView);
    float x = _ga_t0->getX(), y = _ga_t0->getY();
    float local_x, local_y;
    const osg::Camera* camera
        = view->getCameraContainingPosition(x, y, local_x, local_y);
    if (!camera)
        camera = view->getCamera();
    osg::Matrix viewMat = camera->getViewMatrix();
    osg::Matrix viewMatInv = camera->getInverseViewMatrix();
    if (!_ga_t1.valid())
        return;
    osg::Vec3d worldStartDrag;
    // drag start in camera coordinate system.
    bool onEarth;
    if ((onEarth = screenToWorld(_ga_t1->getX(), _ga_t1->getY(),
                                  view, worldStartDrag)))
    {
        if (_lastPointOnEarth == zero)
            _lastPointOnEarth = worldStartDrag;
        else
            worldStartDrag = _lastPointOnEarth;
    }
    else if (_is_geocentric)
    {
        if (_lastPointOnEarth != zero)
        {
            worldStartDrag =_lastPointOnEarth;
        }
        else if (csnSafe.valid())
        {
            const osg::Vec3d startWinPt = getWindowPoint(view, _ga_t1->getX(),
                                                         _ga_t1->getY());
            const osg::Vec3d startDrag = calcTangentPoint(
                zero, zero * viewMat, radiusEquator,
                startWinPt);
            worldStartDrag = startDrag * viewMatInv;
        }
    }
    else
        return;
    // ray from eye through pointer in camera coordinate system goes
    // from origin through transformed pointer coordinates
    const osg::Vec3d winpt = getWindowPoint(view, x, y);
    // Find new point to which startDrag has been moved
    osg::Vec3d worldEndDrag;
    osg::Quat worldRot;
    bool endOnEarth = screenToWorld(x, y, view, worldEndDrag);
    if (endOnEarth)
    {
        // OE_WARN << "end drag: " << worldEndDrag << "\n";
    }
    else
    {
        Vec3d earthOrigin = zero * viewMat;
        const osg::Vec3d endDrag = calcTangentPoint(
            zero, earthOrigin, radiusEquator, winpt);
        worldEndDrag = endDrag * viewMatInv;
        //OE_INFO << "tangent: " << worldEndDrag << "\n";
    }

#if 0
    if (onEarth != endOnEarth)
    {
        std::streamsize oldPrecision = osgEarth::notify(INFO).precision(10);
        OE_INFO << (onEarth ? "leaving earth\n" : "entering earth\n");
        OE_INFO << "start drag: " << worldStartDrag.x() << " "
                << worldStartDrag.y() << " "
                << worldStartDrag.z() << "\n";
        OE_INFO << "end drag: " << worldEndDrag.x() << " "
                << worldEndDrag.y() << " "
                << worldEndDrag.z() << "\n";
        osgEarth::notify(INFO).precision(oldPrecision);
    }
#endif

    if (_is_geocentric)
    {
        worldRot.makeRotate(worldStartDrag, worldEndDrag);
        // Move the camera by the inverse rotation
        Quat cameraRot = worldRot.conj();
        // Derive manipulator parameters from the camera matrix. We
        // can't use _center, _centerRotation, and _rotation directly
        // from the manipulator because they may have been updated
        // already this frame while the camera view matrix,
        // used to do the terrain intersection, has not. This happens
        // when several mouse movement events arrive in a frame. there
        // will be bad stuttering artifacts if we use the updated
        // manipulator parameters.
        Matrixd Mmanip = Matrixd::translate(_offset_x, _offset_y, -_distance)
            * viewMatInv;
        Vec3d center = Mmanip.getTrans();
        Quat centerRotation = makeCenterRotation(center);
        Matrixd Mrotation = (Mmanip * Matrixd::translate(center * -1)
                             * Matrixd::rotate(centerRotation.inverse()));
        Matrixd Me = Matrixd::rotate(centerRotation)
            * Matrixd::translate(center) * Matrixd::rotate(cameraRot);
        // In order for the Viewpoint settings to make sense, the
        // inverse camera matrix must not have a roll component, which
        // implies that its x axis remains parallel to the
        // z = 0 plane. The strategy for doing that is different if
        // the azimuth is locked.
        // Additionally, the part of the camera rotation defined by
        // _centerRotation must be oriented with the local frame of
        // _center on the ellipsoid. For the purposes of the drag
        // motion this is nearly identical to the frame obtained by
        // the trackball motion, so we just fix it up at the end.
        if (_settings->getLockAzimuthWhilePanning())
        {
            // The camera needs to be rotated that _centerRotation
            // is a rotation only around the global Z axis and the
            // camera frame X axis. We don't change _rotation, so that
            // azimuth and pitch will stay constant, but the drag must
            // still be correct i.e.,  the point dragged must remain
            // under the cursor. Therefore the rotation must be around the
            // point that was dragged, worldEndDrag.
            //
            // Rotate Me so that its x axis is parallel to the z=0
            // plane. 
            // Find cone with worldEndDrag->center axis and x
            // axis of coordinate frame as generator of the conical
            // surface.
            Vec3d coneAxis = worldEndDrag * -1;
            coneAxis.normalize();
            Vec3d xAxis(Me(0, 0), Me(0, 1), Me(0, 2));
            // Center of disk: project xAxis onto coneAxis
            double diskDist = xAxis * coneAxis;
            Vec3d P1 = coneAxis * diskDist;
            // Basis of disk equation:
            // p = P1 + R * r * cos(theta) + S * r * sin(theta)
            Vec3d R = xAxis - P1;
            Vec3d S = R ^ coneAxis;
            double r = R.normalize();
            S.normalize();
            // Solve for angle that rotates xAxis into z = 0 plane.
            // soln to 0 = P1.z + r cos(theta) R.z + r sin(theta) S.z
            double temp1 = r * (square(S.z()) + square(R.z()));
            if (equivalent(temp1, 0.0))
                return;
            double radical = r * temp1 - square(P1.z());
            if (radical < 0)
                return;
            double temp2 = R.z() * sqrt(radical) / temp1;
            double temp3 = S.z() * P1.z() / temp1;
            double sin1 = temp2 + temp3;
            double sin2 = temp2 - temp3;
            double theta1 = DBL_MAX;
            double theta2 = DBL_MAX;
            Matrixd cm1, cm2;
            if (fabs(sin1) <= 1.0)
            {
                theta1 = -asin(sin1);
                Matrixd m = rotateAroundPoint(worldEndDrag, -theta1, coneAxis);
                cm1 = Me * m;
            }
            if (fabs(sin2) <= 1.0)
            {
                theta2 = asin(sin2);
                Matrix m = rotateAroundPoint(worldEndDrag, -theta2, coneAxis);
                cm2 = Me * m;
            }
            if (theta1 == DBL_MAX && theta2 == DBL_MAX)
                return;
            Matrixd* CameraMat = 0;
            if (theta1 != DBL_MAX && cm1(1, 2) >= 0.0)
                CameraMat = &cm1;
            else if (theta2 != DBL_MAX && cm2(1, 2) >= 0.0)
                CameraMat = &cm2;
            else
                return;

            setCenter( CameraMat->getTrans() );
        }
        else
        {
            // The camera matrix must be rotated around the local Z axis so
            // that the X axis is parallel to the global z = 0
            // plane. Then, _rotation is rotated by the inverse
            // rotation to preserve the total transformation.
            double theta = atan2(-Me(0, 2), Me(1, 2));
            double s = sin(theta), c = cos(theta);
            if (c * Me(1, 2) - s * Me(0, 2) < 0.0)
            {
                s = -s;
                c = -c;
            }
            Matrixd m(c, s, 0, 0,
                      -s, c, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1);
            Matrixd CameraMat = m * Me;
            setCenter( CameraMat.getTrans() );
            // It's not necessary to include the translation
            // component, but it's useful for debugging.
            Matrixd headMat
                = (Matrixd::translate(-_offset_x, -_offset_y, _distance)
           * Mrotation);
            headMat = headMat * Matrixd::inverse(m);
            _rotation = headMat.getRotate();
            //recalculateLocalPitchAndAzimuth();
        }
        _centerRotation = makeCenterRotation(_center);

        _previousUp = getUpVector(_centerLocalToWorld);
    }
    else
    {
        // This is obviously not correct.
        setCenter( _center + (worldStartDrag - worldEndDrag) );
    }
}

