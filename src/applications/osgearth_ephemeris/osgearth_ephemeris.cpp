/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarth/NodeUtils>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/Ephemeris>
#include <osgEarth/Sky>
#include <osgEarth/LatLongFormatter>
#include <osgEarth/Controls>
#include <osgEarth/PlaceNode>
#include <osgDB/ReadFile>

#define LC "[osgearth_ephemeris] "

using namespace osgEarth;
using namespace osgEarth::Util;

namespace ui = osgEarth::Util::Controls;

int
usage(const char* name)
{
    OE_NOTICE
        << "\nUsage: " << name << " <file.earth> --sky" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}

struct App
{
    App() {
        _playing = false;
        readout = new ui::LabelControl();
        readout->setVertAlign(ui::Control::ALIGN_CENTER);
    }

    osg::ref_ptr<PlaceNode> sunPos;
    osg::ref_ptr<PlaceNode> moonPos;
    SkyNode* sky;
    ui::LabelControl* readout;

    void play() { _playing = true; }
    void stop() { _playing = false; }

    void tick() {
        if (_playing) {
            TimeStamp t = sky->getDateTime().asTimeStamp() + 1;
            sky->setDateTime(DateTime(t));
        }
        readout->setText(sky->getDateTime().asRFC1123());
    }

    bool _playing;
};

struct Play : public ui::ControlEventHandler {
    Play(App& app) : _app(app) { }
    void onClick(ui::Control*) { _app.play(); }
    App& _app;
};

struct Stop : public ui::ControlEventHandler {
    Stop(App& app) : _app(app) { }
    void onClick(ui::Control*) { _app.stop(); }
    App& _app;
};

ui::Container* createUI(App& app)
{
    ui::HBox* vcr = new ui::HBox();
    vcr->addControl(new ui::ButtonControl("Play", new Play(app)));
    vcr->addControl(new ui::ButtonControl("Stop", new Stop(app)));
    vcr->addControl(app.readout);
    return vcr;
}

int
main(int argc, char** argv)
{
    osgEarth::initialize();

    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    if ( arguments.find("--sky") < 0 )
        return usage(argv[0]);

    osgViewer::Viewer viewer(arguments);

    EarthManipulator* em = new EarthManipulator(arguments);
    em->getSettings()->setMinMaxPitch(-89, 89);
    viewer.setCameraManipulator( em );

    osg::ref_ptr<osg::Image> mark = osgDB::readRefImageFile("../data/placemark32.png");

    App app;

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags
    auto node = MapNodeHelper().load( arguments, &viewer );
    if (node.valid())
    {
        osg::Group* root = new osg::Group();
        root->addChild( node );

        MapNode* mapNode = MapNode::get(node);
        if (!mapNode)
            return usage(argv[0]);

        app.sunPos = new PlaceNode("Sun", Style(), mark.get());
        app.sunPos->setDynamic(true);
        mapNode->addChild( app.sunPos.get() );

        app.moonPos = new PlaceNode("Moon", Style(), mark.get());
        app.moonPos->setDynamic(true);
        mapNode->addChild( app.moonPos.get() );

        app.sky = osgEarth::findTopMostNodeOfType<SkyNode>(node);
        const Ephemeris* ephemeris = 0L;
        if ( app.sky )
        {
            ephemeris = app.sky->getEphemeris();
        }

        LatLongFormatter llf;
        llf.setOptions( LatLongFormatter::Options(llf.FORMAT_DEGREES_MINUTES_SECONDS) );
        llf.setPrecision( 8 );

        viewer.setSceneData( root );

        ui::ControlCanvas* container = ui::ControlCanvas::getOrCreate(&viewer);
        container->addChild(createUI(app));

        while(!viewer.done())
        {
            viewer.frame();

            if ( ephemeris )
            {
                const DateTime& dt = app.sky->getDateTime();

                CelestialBody sun = ephemeris->getSunPosition(dt);
                GeoPoint sunPos;
                sunPos.fromWorld(mapNode->getMapSRS(), sun.geocentric);
                sunPos.alt() = 0.0;
                app.sunPos->setPosition( sunPos );
                app.sunPos->setText( "Sun\n" + llf.format(sunPos) );

                CelestialBody moon = ephemeris->getMoonPosition(dt);
                GeoPoint moonPos;
                moonPos.fromWorld(mapNode->getMapSRS(), moon.geocentric);
                moonPos.alt() = 0.0;
                app.moonPos->setPosition( moonPos );
                app.moonPos->setText( "Moon\n" + llf.format(moonPos) );
            }

            app.tick();
        }
    }
    else
    {
        return usage(argv[0]);
    }
}
