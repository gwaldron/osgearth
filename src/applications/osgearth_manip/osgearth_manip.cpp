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

#include <string>

#include <osg/Notify>
#include <osg/Switch>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/MapNode>
#include <osgEarth/Viewpoint>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/Controls>

using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

namespace
{
    class SwitchHandler : public osgGA::GUIEventHandler
    {
    public:
        SwitchHandler(char key = 0) : _key(key) {}

        bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa, osg::Object* object, osg::NodeVisitor* /*nv*/)
        {
            if (ea.getHandled() || ea.getEventType() != osgGA::GUIEventAdapter::KEYDOWN )
                return false;

            if ( ea.getKey() == _key )
            {
                ControlCanvas* canvas = ControlCanvas::get(aa.asView());
                if ( canvas )
                    canvas->setNodeMask( canvas->getNodeMask() ^ 0xFFFFFFFF );
            }
            return false;
        }

    protected:
        char _key;
    };

    osg::Node* createHelp( osgViewer::View* view )
    {
        static char s_help[] = 
            "left mouse: pan \n"
            "middle mouse: tilt/slew \n"
            "right mouse: zoom in/out continuous \n"
            "double-click: zoom in \n"
            "scroll wheel: zoom in/out \n"
            "arrows: pan\n"
            "1-6 : fly to preset viewpoints \n"
            "shift-right-mouse: locked panning\n"
            "u : toggle azimuth locking\n"
            "h : toggle this help\n";

        VBox* v = new VBox();
        v->addControl( new LabelControl( "EarthManipulator", osg::Vec4f(1,1,0,1) ) );
        v->addControl( new LabelControl( s_help ) );
        ControlCanvas* canvas = ControlCanvas::get( view );
        canvas->addControl( v );
        canvas->setEventCallback(new SwitchHandler('h'));    

        return canvas;
    }

    // some preset viewpoints.
    static Viewpoint VPs[] = {
        Viewpoint( "Africa",        osg::Vec3d(    0.0,   0.0, 0.0 ), 0.0, -90.0, 10e6 ),
        Viewpoint( "California",    osg::Vec3d( -121.0,  34.0, 0.0 ), 0.0, -90.0, 6e6 ),
        Viewpoint( "Europe",        osg::Vec3d(    0.0,  45.0, 0.0 ), 0.0, -90.0, 4e6 ),
        Viewpoint( "Washington DC", osg::Vec3d(  -77.0,  38.0, 0.0 ), 0.0, -90.0, 1e6 ),
        Viewpoint( "Australia",     osg::Vec3d(  135.0, -20.0, 0.0 ), 0.0, -90.0, 2e6 ),
        Viewpoint( "Boston",        osg::Vec3d( -71.096936, 42.332771, 0 ), 0.0, -90, 1e5 )
    };

    // a simple handler that demonstrates the "viewpoint" functionality in 
    // osgEarthUtil::EarthManipulator. Press a number key to fly to a viewpoint.
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

    struct LockAzimuthHandler : public osgGA::GUIEventHandler
    {
        LockAzimuthHandler(char key, EarthManipulator* manip)
            : _key(key), _manip(manip)
        {
        }

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

}


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);       
    osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );

    // install the programmable manipulator.
    EarthManipulator* manip = new EarthManipulator();

    osg::Node* earthNode = osgDB::readNodeFiles( arguments );
    if (!earthNode)
    {
        OE_WARN << "Unable to load earth model." << std::endl;
        return -1;
    }

    osgViewer::Viewer viewer(arguments);

    osg::Group* root = new osg::Group();
    root->addChild( earthNode );
    root->addChild( createHelp(&viewer) );

    osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( earthNode );
    if ( mapNode )
    {
        if ( mapNode )
            manip->setNode( mapNode->getTerrainEngine() );

        if ( mapNode->getMap()->isGeocentric() )
        {
            manip->setHomeViewpoint( 
                Viewpoint( osg::Vec3d( -90, 0, 0 ), 0.0, -90.0, 5e7 ) );

            // add a handler that will automatically calculate good clipping planes
            viewer.getCamera()->addCullCallback( new AutoClipPlaneCullCallback(mapNode->getMap()) );
        }
    }

    viewer.setSceneData( root );
    viewer.setCameraManipulator( manip );

    manip->getSettings()->bindMouse(
        EarthManipulator::ACTION_EARTH_DRAG,
        osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON,
        osgGA::GUIEventAdapter::MODKEY_SHIFT );

    manip->getSettings()->setArcViewpointTransitions( true );
    
    viewer.addEventHandler(new FlyToViewpointHandler( manip ));
    viewer.addEventHandler(new LockAzimuthHandler('u', manip));


    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));
    //viewer.addEventHandler(new osgViewer::RecordCameraPathHandler());

    return viewer.run();
}
