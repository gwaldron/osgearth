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
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Viewpoint>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/Graticule>

#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/GeometrySymbol>

#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osgEarthDrivers/agglite/AGGLiteOptions>

#include <osgUtil/IncrementalCompileOperation>

using namespace osgEarth::Drivers;
using namespace osgEarth::Symbology;
using namespace osgEarth::Features;
using namespace osgEarth::Util;

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
        }
        return false;
    }

    osg::observer_ptr<EarthManipulator> _manip;
};

// a simple handler that toggles a node mask on/off
struct NodeToggleHandler : public osgGA::GUIEventHandler 
{
    NodeToggleHandler( osg::Node* node, char key, const char* nodeName )
        : _node(node), _key(key), _nodeName(nodeName) { }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if ( ea.getEventType() == ea.KEYDOWN && ea.getKey() == _key )
        {
            _node->setNodeMask( _node->getNodeMask() == 0 ? ~0 : 0 );
        }
        return false;
    }

    void getUsage(osg::ApplicationUsage& usage) const
    {
        using namespace std;
        usage.addKeyboardMouseBinding(string(1, _key),
                                      string("Toggle ") + _nodeName);
    }
    
    osg::observer_ptr<osg::Node> _node;
    char _key;
    std::string _nodeName;
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
            bool lockAzimuth
                = _manip->getSettings()->getLockAzimuthWhilePanning();
            _manip->getSettings()->setLockAzimuthWhilePanning(!lockAzimuth);
            return true;
        }
        return false;
    }

    void getUsage(osg::ApplicationUsage& usage) const
    {
        using namespace std;
        usage.addKeyboardMouseBinding(string(1, _key),
                                      string("Toggle azimuth locking"));
    }

    char _key;
    osg::ref_ptr<EarthManipulator> _manip;
    
};

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

    osg::Group* root = new osg::Group();
    root->addChild( earthNode );

    osgViewer::Viewer viewer(arguments);

    Graticule* graticule = 0L;

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
            viewer.addEventHandler( new AutoClipPlaneHandler() );
        }

        // create a graticle, and start it in the OFF position
        graticule = new Graticule( mapNode->getMap() );
        graticule->setNodeMask(0);
        root->addChild( graticule );
    }

    viewer.setSceneData( root );
    viewer.setCameraManipulator( manip );

    // bind "double-click" to the zoom-to function in the EarthManipulator
    manip->getSettings()->bindMouseDoubleClick(
        EarthManipulator::ACTION_GOTO,
        osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON );

    manip->getSettings()->bindMouse(
        EarthManipulator::ACTION_EARTH_DRAG,
        osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON,
        osgGA::GUIEventAdapter::MODKEY_SHIFT );
    
    // add our fly-to handler
    viewer.addEventHandler(new FlyToViewpointHandler( manip ));

    // add a handler to toggle the graticle
    if ( graticule )
        viewer.addEventHandler(new NodeToggleHandler( graticule, 'g', "graticule"));
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
