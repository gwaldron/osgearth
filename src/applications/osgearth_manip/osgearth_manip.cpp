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

#include <string>

#include <osg/Notify>
#include <osg/Timer>
#include <osg/ShapeDrawable>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/GeoMath>
#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Viewpoint>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthAnnotation/LocalGeometryNode>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthSymbology/Style>

using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Annotation;

#define D2R (osg::PI/180.0)
#define R2D (180.0/osg::PI)

namespace
{
    /**
     * Builds our help menu UI.
     */
    Control* createHelp( osgViewer::View* view )
    {
        const char* text[] =
        {
            "left mouse :",        "pan",
            "middle mouse :",      "rotate",
            "right mouse :",       "continuous zoom",
            "double-click :",      "zoom to point",
            "scroll wheel :",      "zoom in/out",
            "arrows :",            "pan",
            "1-6 :",               "fly to preset viewpoints",
            "shift-right-mouse :", "locked panning",
            "u :",                 "toggle azimuth lock",
            "c :",                 "toggle perspective/ortho",
            "t :",                 "toggle tethering",
            "a :",                 "toggle viewpoint arcing",
            "z :",                 "toggle throwing"
        };

        Grid* g = new Grid();
        for( unsigned i=0; i<sizeof(text)/sizeof(text[0]); ++i )
        {
            unsigned c = i % 2;
            unsigned r = i / 2;
            g->setControl( c, r, new LabelControl(text[i]) );
        }

        VBox* v = new VBox();
        v->addControl( g );

        return v;
    }


    /**
     * Some preset viewpoints to show off the setViewpoint function.
     */
    static Viewpoint VPs[] = {
        Viewpoint( "Africa",        osg::Vec3d(    0.0,   0.0, 0.0 ), 0.0, -90.0, 10e6 ),
        Viewpoint( "California",    osg::Vec3d( -121.0,  34.0, 0.0 ), 0.0, -90.0, 6e6 ),
        Viewpoint( "Europe",        osg::Vec3d(    0.0,  45.0, 0.0 ), 0.0, -90.0, 4e6 ),
        Viewpoint( "Washington DC", osg::Vec3d(  -77.0,  38.0, 0.0 ), 0.0, -90.0, 1e6 ),
        Viewpoint( "Australia",     osg::Vec3d(  135.0, -20.0, 0.0 ), 0.0, -90.0, 2e6 ),
        Viewpoint( "Boston",        osg::Vec3d( -71.096936, 42.332771, 0 ), 0.0, -90, 1e5 )
    };


    /**
     * Handler that demonstrates the "viewpoint" functionality in 
     *  osgEarthUtil::EarthManipulator. Press a number key to fly to a viewpoint.
     */
    struct FlyToViewpointHandler : public osgGA::GUIEventHandler 
    {
        FlyToViewpointHandler( EarthManipulator* manip ) : _manip(manip) { }

        bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
        {
            if ( ea.getEventType() == ea.KEYDOWN && ea.getKey() >= '1' && ea.getKey() <= '6' )
            {
                _manip->setViewpoint( VPs[ea.getKey()-'1'], 4.0 );
                aa.requestRedraw();
            }
            return false;
        }

        osg::observer_ptr<EarthManipulator> _manip;
    };


    /**
     * Handler to toggle "azimuth locking", which locks the camera's relative Azimuth
     * while panning. For example, it can maintain "north-up" as you pan around. The
     * caveat is that when azimuth is locked you cannot cross the poles.
     */
    struct LockAzimuthHandler : public osgGA::GUIEventHandler
    {
        LockAzimuthHandler(char key, EarthManipulator* manip)
            : _key(key), _manip(manip) { }

        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
        {
            if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == _key)
            {
                bool lockAzimuth = _manip->getSettings()->getLockAzimuthWhilePanning();
                _manip->getSettings()->setLockAzimuthWhilePanning(!lockAzimuth);
                aa.requestRedraw();
                return true;
            }
            return false;
        }

        void getUsage(osg::ApplicationUsage& usage) const
        {
            using namespace std;
            usage.addKeyboardMouseBinding(string(1, _key), string("Toggle azimuth locking"));
        }

        char _key;
        osg::ref_ptr<EarthManipulator> _manip;
    };


    /**
     * Handler to toggle "viewpoint transtion arcing", which causes the camera to "arc"
     * as it travels from one viewpoint to another.
     */
    struct ToggleArcViewpointTransitionsHandler : public osgGA::GUIEventHandler
    {
        ToggleArcViewpointTransitionsHandler(char key, EarthManipulator* manip)
            : _key(key), _manip(manip) { }

        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
        {
            if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == _key)
            {
                bool arc = _manip->getSettings()->getArcViewpointTransitions();
                _manip->getSettings()->setArcViewpointTransitions(!arc);
                aa.requestRedraw();
                return true;
            }
            return false;
        }

        void getUsage(osg::ApplicationUsage& usage) const
        {
            using namespace std;
            usage.addKeyboardMouseBinding(string(1, _key), string("Arc viewpoint transitions"));
        }

        char _key;
        osg::ref_ptr<EarthManipulator> _manip;
    };


    /**
     * Toggles the projection matrix between perspective and orthographic.
     */
    struct ToggleProjectionHandler : public osgGA::GUIEventHandler
    {
        ToggleProjectionHandler(char key, EarthManipulator* manip)
            : _key(key), _manip(manip)
        {
        }

        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
        {
            if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == _key)
            {
                if ( _manip->getSettings()->getCameraProjection() == EarthManipulator::PROJ_PERSPECTIVE )
                    _manip->getSettings()->setCameraProjection( EarthManipulator::PROJ_ORTHOGRAPHIC );
                else
                    _manip->getSettings()->setCameraProjection( EarthManipulator::PROJ_PERSPECTIVE );
                aa.requestRedraw();
                return true;
            }
            return false;
        }

        void getUsage(osg::ApplicationUsage& usage) const
        {
            using namespace std;
            usage.addKeyboardMouseBinding(string(1, _key), string("Toggle projection type"));
        }

        char _key;
        osg::ref_ptr<EarthManipulator> _manip;
    };


    /**
     * Toggles the throwing feature.
     */
    struct ToggleThrowingHandler : public osgGA::GUIEventHandler
    {
        ToggleThrowingHandler(char key, EarthManipulator* manip)
            : _key(key), _manip(manip)
        {
        }

        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
        {
            if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == _key)
            {
                bool throwing = _manip->getSettings()->getThrowingEnabled();
                _manip->getSettings()->setThrowingEnabled( !throwing );
                aa.requestRedraw();
                return true;
            }
            return false;
        }

        void getUsage(osg::ApplicationUsage& usage) const
        {
            using namespace std;
            usage.addKeyboardMouseBinding(string(1, _key), string("Toggle throwing"));
        }

        char _key;
        osg::ref_ptr<EarthManipulator> _manip;
    };


    /**
     * A simple simulator that moves an object around the Earth. We use this to
     * demonstrate/test tethering.
     */
    struct Simulator : public osgGA::GUIEventHandler
    {
        Simulator( osg::Group* root, EarthManipulator* manip, MapNode* mapnode )
            : _manip(manip), _mapnode(mapnode), _lat0(55.0), _lon0(45.0), _lat1(-55.0), _lon1(-45.0)
        {
            osg::Node* geode = AnnotationUtils::createSphere( 25.0, osg::Vec4(1,.7,.4,1) );
            
            _xform = new osg::MatrixTransform();
            _xform->addChild( geode );

            _cam = new osg::Camera();
            _cam->setRenderOrder( osg::Camera::NESTED_RENDER, 1 );
            _cam->addChild( _xform );

            Style style;
            style.getOrCreate<TextSymbol>()->size() = 32.0f;
            style.getOrCreate<TextSymbol>()->declutter() = false;
            _label = new LabelNode(_mapnode, GeoPoint(), "Hello World", style);
            _label->setDynamic( true );
            _cam->addChild( _label );

            root->addChild( _cam.get() );
        }

        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
        {
            if ( ea.getEventType() == ea.FRAME )
            {
                double t = fmod( osg::Timer::instance()->time_s(), 600.0 ) / 600.0;
                double lat, lon;
                GeoMath::interpolate( D2R*_lat0, D2R*_lon0, D2R*_lat1, D2R*_lon1, t, lat, lon );
                GeoPoint p( SpatialReference::create("wgs84"), R2D*lon, R2D*lat, 25000.0, ALTMODE_ABSOLUTE );
                osg::Vec3d world;
                p.toWorld( world );
                _xform->setMatrix( osg::Matrix::translate(world) );
                _label->setPosition( p );
            }
            else if ( ea.getEventType() == ea.KEYDOWN && ea.getKey() == 't' )
            {
                _manip->setTetherNode( _manip->getTetherNode() ? 0L : _xform.get() );
                if ( _manip->getTetherNode() )
                {
                    _manip->getSettings()->setArcViewpointTransitions( false );
                    _manip->setViewpoint(Viewpoint(osg::Vec3d(0,0,0), 45, -25, 250000));
                    _manip->getSettings()->setArcViewpointTransitions( true );
                }
                return true;
            }
            return false;
        }

        MapNode*                           _mapnode;
        EarthManipulator*                  _manip;
        osg::ref_ptr<osg::Camera>          _cam;
        osg::ref_ptr<osg::MatrixTransform> _xform;
        double                             _lat0, _lon0, _lat1, _lon1;
        LabelNode*                         _label;
    };
}


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );

    osgViewer::Viewer viewer(arguments);

    // install the programmable manipulator.
    EarthManipulator* manip = new EarthManipulator();
    viewer.setCameraManipulator( manip );

    // UI:
    Control* help = createHelp(&viewer);

    osg::Node* earthNode = MapNodeHelper().load( arguments, &viewer, help );
    if (!earthNode)
    {
        OE_WARN << "Unable to load earth model." << std::endl;
        return -1;
    }

    osg::Group* root = new osg::Group();
    root->addChild( earthNode );

    osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( earthNode );
    if ( mapNode )
    {
        if ( mapNode )
            manip->setNode( mapNode->getTerrainEngine() );

        if ( mapNode->getMap()->isGeocentric() )
        {
            manip->setHomeViewpoint( 
                Viewpoint( osg::Vec3d( -90, 0, 0 ), 0.0, -90.0, 5e7 ) );
        }
    }

    // Simulator for tethering:
    viewer.addEventHandler( new Simulator(root, manip, mapNode) );
    manip->getSettings()->getBreakTetherActions().push_back( EarthManipulator::ACTION_PAN );
    manip->getSettings()->getBreakTetherActions().push_back( EarthManipulator::ACTION_GOTO );


    viewer.setSceneData( root );

    manip->getSettings()->bindMouse(
        EarthManipulator::ACTION_EARTH_DRAG,
        osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON,
        osgGA::GUIEventAdapter::MODKEY_SHIFT );

    manip->getSettings()->setArcViewpointTransitions( true );
    
    viewer.addEventHandler(new FlyToViewpointHandler( manip ));
    viewer.addEventHandler(new LockAzimuthHandler('u', manip));
    viewer.addEventHandler(new ToggleProjectionHandler('c', manip));
    viewer.addEventHandler(new ToggleArcViewpointTransitionsHandler('a', manip));
    viewer.addEventHandler(new ToggleThrowingHandler('z', manip));

    return viewer.run();
}
