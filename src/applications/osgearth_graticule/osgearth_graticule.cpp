/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osg/Notify>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgEarth/MapNode>
#include <osgEarth/GLUtils>

#include <osgEarth/EarthManipulator>
#include <osgEarth/MouseCoordsTool>
#include <osgEarth/MGRSFormatter>
#include <osgEarth/LatLongFormatter>

#include <osgEarth/GeodeticGraticule>
#include <osgEarth/MGRSGraticule>
#include <osgEarth/UTMGraticule>
#include <osgEarth/GARSGraticule>

using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Contrib;

int
usage( const std::string& msg )
{
    OE_NOTICE
        << msg << std::endl
        << "USAGE: osgearth_graticule [options] file.earth" << std::endl
        << "   --geodetic            : display a Lat/Long graticule" << std::endl
        << "   --utm                 : display a UTM graticule" << std::endl
        << "   --mgrs                : display an MGRS graticule" << std::endl
        << "   --gars                : display a GARS graticule" << std::endl;
    return -1;
}

//------------------------------------------------------------------------

int
main(int argc, char** argv)
{
    osgEarth::initialize();

    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);

    // parse command line:
    bool isUTM = arguments.read("--utm");
    bool isMGRS = arguments.read("--mgrs");
    bool isGeodetic = arguments.read("--geodetic");
    bool isGARS = arguments.read("--gars");

    // load the .earth file from the command line.
    MapNode* mapNode = MapNode::load( arguments );
    if ( !mapNode )
        return usage( "Failed to load a map from the .earth file" );

    // install our manipulator:
    viewer.setCameraManipulator( new EarthManipulator() );

    // initialize the top level state
    GLUtils::setGlobalDefaults(viewer.getCamera()->getOrCreateStateSet());

    // root scene graph:
    osg::Group* root = new osg::Group();
    root->addChild( mapNode );

    Formatter* formatter = 0L;
    if ( isUTM )
    {
        UTMGraticule* gr = new UTMGraticule();
        mapNode->getMap()->addLayer(gr);
        formatter = new MGRSFormatter();
    }
    else if ( isMGRS )
    {
        MGRSGraticule* gr = new MGRSGraticule();
        mapNode->getMap()->addLayer(gr);
        formatter = new MGRSFormatter();
    }
    else if ( isGARS )
    {
        GARSGraticule* gr = new GARSGraticule();
        mapNode->getMap()->addLayer(gr);
        formatter = new LatLongFormatter();
    }
    else // if ( isGeodetic )
    {
        GeodeticGraticule* gr = new GeodeticGraticule();
        mapNode->getMap()->addLayer(gr);
        formatter = new LatLongFormatter();
    }

    // mouse coordinate readout:
    ControlCanvas* canvas = new ControlCanvas();
    root->addChild( canvas );
    VBox* vbox = new VBox();
    canvas->addControl( vbox );

    LabelControl* readout = new LabelControl();
    vbox->addControl( readout );

    MouseCoordsTool* tool = new MouseCoordsTool( mapNode );
    tool->addCallback( new MouseCoordsLabelCallback(readout, formatter) );
    viewer.addEventHandler( tool );


    // disable the small-feature culling
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    // set a near/far ratio that is smaller than the default. This allows us to get
    // closer to the ground without near clipping. If you need more, use --logdepth
    viewer.getCamera()->setNearFarRatio(0.0001);


    // finalize setup and run.
    viewer.setSceneData( root );
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    return viewer.run();
}
