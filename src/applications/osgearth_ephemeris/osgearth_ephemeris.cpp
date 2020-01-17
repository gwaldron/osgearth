/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarth/NodeUtils>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/Ephemeris>
#include <osgEarthUtil/Sky>
#include <osgEarthUtil/LatLongFormatter>
#include <osgEarthUtil/Controls>
#include <osgEarthAnnotation/PlaceNode>

#define LC "[osgearth_ephemeris] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Annotation;

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

    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    osg::ref_ptr<osg::Image> mark = osgDB::readRefImageFile("../data/placemark32.png");
    
    App app;

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load( arguments, &viewer );
    if ( node )
    {
        osg::Group* root = new osg::Group();
        root->addChild( node );

        MapNode* mapNode = MapNode::get(node);

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
