/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
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

/**
 * Demonstrates how to place an object and control its position and
 * orientation by combining a GeoTransform and a PositionAttitudeTransform.
 */

#include <osg/PositionAttitudeTransform>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/Controls>
#include <osgEarth/GeoTransform>
#include <osgEarth/MapNode>

#define LC "[osgearth_transform] "


using namespace osgEarth;
using namespace osgEarth::Util;

namespace ui = osgEarth::Util::Controls;

int
usage(const char* name)
{
    OE_NOTICE 
        << "\nUsage: " << name << " file.earth" << std::endl;

    return 0;
}

struct App
{
    const osgEarth::SpatialReference* srs;
    osgEarth::GeoTransform*           geo;
    osg::PositionAttitudeTransform*   pat;

    ui::HSliderControl* uiLat;
    ui::HSliderControl* uiLon;
    ui::HSliderControl* uiAlt;
    ui::HSliderControl* uiHeading;
    ui::HSliderControl* uiPitch;
    ui::HSliderControl* uiRoll;

    void apply()
    {
        GeoPoint pos(
            srs,
            uiLon->getValue(), uiLat->getValue(), uiAlt->getValue(),
            ALTMODE_ABSOLUTE);

        geo->setPosition( pos );

        osg::Quat ori =
            osg::Quat(osg::DegreesToRadians(uiRoll->getValue()),    osg::Vec3(0,1,0)) *
            osg::Quat(osg::DegreesToRadians(uiPitch->getValue()),   osg::Vec3(1,0,0)) *
            osg::Quat(osg::DegreesToRadians(uiHeading->getValue()), osg::Vec3(0,0,-1));

        pat->setAttitude( ori );
    }
};

struct Apply : public ui::ControlEventHandler
{
    Apply(App& app) : _app(app) { }
    void onValueChanged(ui::Control* control) {
        _app.apply();
    }
    App& _app;
};

ui::Control* makeUI(App& app)
{
    ui::Grid* grid = new ui::Grid();

    grid->setControl(0, 0, new ui::LabelControl("Lat:"));
    grid->setControl(0, 1, new ui::LabelControl("Long:"));
    grid->setControl(0, 2, new ui::LabelControl("Alt:"));
    grid->setControl(0, 3, new ui::LabelControl("Heading:"));
    grid->setControl(0, 4, new ui::LabelControl("Pitch:"));
    grid->setControl(0, 5, new ui::LabelControl("Roll:"));

    app.uiLat = grid->setControl(1, 0, new ui::HSliderControl(-80.0f, 80.0f, 42.0f, new Apply(app)));
    app.uiLon = grid->setControl(1, 1, new ui::HSliderControl(-180.0f, 180.0f, 7.0f, new Apply(app)));
    app.uiAlt = grid->setControl(1, 2, new ui::HSliderControl(50000.0f, 500000.0f, 250000.0f, new Apply(app)));
    app.uiHeading = grid->setControl(1, 3, new ui::HSliderControl(-180.0f, 180.0f, 0.0f, new Apply(app)));
    app.uiPitch   = grid->setControl(1, 4, new ui::HSliderControl(-90.0f, 90.0f, 0.0f, new Apply(app)));
    app.uiRoll    = grid->setControl(1, 5, new ui::HSliderControl(-180.0f, 180.0f, 0.0f, new Apply(app)));

    app.uiLat->setHorizFill(true, 300.0f);
    return grid;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    osgViewer::Viewer viewer(arguments);
    EarthManipulator* em = new EarthManipulator();
    viewer.setCameraManipulator( em );

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* earth = MapNodeHelper().load( arguments, &viewer );
    MapNode* mapNode = MapNode::get(earth);
    if (!mapNode)
        return usage(argv[0]);

    // load the model file into the local coordinate frame, which will be
    // +X=east, +Y=north, +Z=up.
    osg::Node* model = osgDB::readNodeFile("../data/axes.osgt.(1000).scale");
    if ( !model )
        return usage(argv[0]);

    osg::Group* root = new osg::Group();
    root->addChild( earth );
    
    App app;
    app.srs = mapNode->getMapSRS();
    app.geo = new GeoTransform();
    app.geo->setTerrain( mapNode->getTerrain() );
    app.pat = new osg::PositionAttitudeTransform();
    app.pat->addChild( model );
    app.geo->addChild( app.pat );

    root->addChild( app.geo );
    
    viewer.setSceneData( root );
    viewer.getCamera()->setNearFarRatio(0.00002);
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    ui::ControlCanvas::getOrCreate(&viewer)->addControl( makeUI(app) );
    app.apply();

    em->setTetherNode( app.geo );

    osgEarth::Viewpoint vp;
    vp.setNode( app.geo );
    vp.heading()->set( -45.0, Units::DEGREES );
    vp.pitch()->set( -20.0, Units::DEGREES );
    vp.range()->set( model->getBound().radius()*10.0, Units::METERS );
    em->setViewpoint( vp );

    return viewer.run();
}
