/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2015 Pelican Mapping
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
    osg::ref_ptr<PlaceNode> sunPos;
    osg::ref_ptr<PlaceNode> moonPos;
    ui::HSliderControl* year;
    ui::HSliderControl* month;
    ui::HSliderControl* date;
    SkyNode* sky;

    void set(const DateTime& dt)
    {
        year->setValue(dt.year());
        month->setValue(dt.month());
        date->setValue(dt.day());
    }

    void update()
    {
        DateTime oldDT = sky->getDateTime();
        DateTime dt( (int)year->getValue(), (int)month->getValue(), (int)date->getValue(), oldDT.hours() );
        sky->setDateTime( dt );
    }
};

struct Update : public ui::ControlEventHandler
{
    App& _app;
    Update(App& app) : _app(app) { }

    void onValueChanged(ui::Control*)
    {
        _app.update();
    }
};

ui::Container* makeUI(App& app)
{
    Update* update = new Update(app);

    VBox* box = new VBox();

    Grid* g = box->addControl(new Grid());

    g->setControl(0, 0, new LabelControl("Year"));
    app.year = g->setControl(1, 0, new HSliderControl(1990, 2020, 2015, update));
    app.year->setHorizFill(true, 200);
    g->setControl(2, 0, new LabelControl(app.year));

    g->setControl(0, 1, new LabelControl("Month"));
    app.month = g->setControl(1, 1, new HSliderControl(1, 13, 1, update));
    app.month->setHorizFill(true, 200);
    g->setControl(2, 1, new LabelControl(app.month));

    g->setControl(0, 2, new LabelControl("Date"));
    app.date = g->setControl(1, 2, new HSliderControl(1, 32, 1, update));
    app.date->setHorizFill(true, 200);
    g->setControl(2, 2, new LabelControl(app.date));
    
    return box;
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

    osg::ref_ptr<osg::Image> mark = osgDB::readImageFile("../data/placemark32.png");
    
    App app;
    ui::Container* gui = makeUI(app);

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load( arguments, &viewer, gui );
    if ( node )
    {
        osg::Group* root = new osg::Group();
        root->addChild( node );

        MapNode* mapNode = MapNode::get(node);

        app.sunPos = new PlaceNode(mapNode, GeoPoint(), mark.get(), "Sun");
        app.sunPos->setDynamic(true);
        root->addChild( app.sunPos.get() );

        app.moonPos = new PlaceNode(mapNode, GeoPoint(), mark.get(), "Moon");
        app.moonPos->setDynamic(true);

        root->addChild( app.moonPos.get() );        


        app.sky = osgEarth::findTopMostNodeOfType<SkyNode>(node);        
        const Ephemeris* ephemeris = 0L;
        if ( app.sky )
        {
            ephemeris = app.sky->getEphemeris();
            app.set( DateTime() );
        }

        LatLongFormatter llf;
        llf.setOptions( LatLongFormatter::Options(llf.FORMAT_DEGREES_MINUTES_SECONDS) );
        llf.setPrecision( 4 );

        viewer.setSceneData( root );

        while(!viewer.done())
        {
            viewer.frame();

            if ( ephemeris )
            {
                const DateTime& dt = app.sky->getDateTime();

                osg::Vec3d sunECEF = ephemeris->getSunPositionECEF(dt);
                GeoPoint sun;
                sun.fromWorld(mapNode->getMapSRS(), sunECEF);
                sun.alt() = 0.0;
                app.sunPos->setPosition( sun );
                app.sunPos->setText( "Sun\n" + llf.format(sun) );

                osg::Vec3d moonECEF = ephemeris->getMoonPositionECEF(dt);
                GeoPoint moon;
                moon.fromWorld(mapNode->getMapSRS(), moonECEF);
                moon.alt() = 0.0;
                app.moonPos->setPosition( moon );
                app.moonPos->setText( "Moon\n" + llf.format(moon) );
            }
        }
    }
    else
    {
        return usage(argv[0]);
    }
}
