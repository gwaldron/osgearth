/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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

/**
 * This sample shows how to apply a detail texture to osgEarth's terrain.
 */
#include <osg/Notify>
#include <osgViewer/Viewer>
#include <osgEarth/URI>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/DetailTexture>
#include <osg/Image>

using namespace osgEarth;
using namespace osgEarth::Util;
namespace ui = osgEarth::Util::Controls;

int 
usage(const char* msg)
{
    OE_NOTICE << msg << std::endl;
    OE_NOTICE <<
        "Usage:\n"
        "    --image <filename>                  : detail texture image file\n"
        "    --lod <lod>                         : LOD at which to start detail texturing\n"
        "    --uniform oe_dtex_intensity 0 1     : control the intensity of the detail texture\n"
        << std::endl;
    return 0;
}


struct App
{
    DetailTexture* dt;
};


// Build a slider to adjust the vertical scale
ui::Control* createUI( App& app )
{
    struct SetIntensity : public ui::ControlEventHandler {
        App& _app;
        SetIntensity(App& app) : _app(app) {}
        void onValueChanged(ui::Control*, float value) {
            _app.dt->setIntensity(value);
        }
    };

    ui::VBox* vbox = new VBox();

    vbox->addControl( new LabelControl(Stringify() << "Detail texture starts at LOD: " << app.dt->getStartLOD()) );

    ui::HBox* hbox = vbox->addControl( new ui::HBox() );
    hbox->setChildVertAlign( ui::Control::ALIGN_CENTER );
    hbox->addControl( new ui::LabelControl("Intensity:") );
    ui::HSliderControl* intensity = hbox->addControl( new ui::HSliderControl(0.0, 1.0, 0.25, new SetIntensity(app)) );
    intensity->setHorizFill( true, 200 );
    hbox->addControl( new ui::LabelControl(intensity) );

    return vbox;
}


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);

    if (arguments.read("--help"))
        return usage(argv[0]);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator( new EarthManipulator() );

    // Set up the app and read options:
    App app;
    app.dt = new DetailTexture();

    std::string filename( "../data/noise3.png" );
    arguments.read( "--image", filename );
    osg::Image* image = URI(filename).getImage();
    if ( !image )
        return usage( "Failed to load image" );
    app.dt->setImage( image );

    unsigned startLOD;
    if ( arguments.read("--lod", startLOD) )
        app.dt->setStartLOD(startLOD);

    // Create the UI:
    ui::Control* demoui = createUI(app);

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load( arguments, &viewer, demoui );
    if ( node )
    {
        MapNode* mapNode = MapNode::findMapNode(node);
        if ( !mapNode )
            return -1;

        // install our detail texturer.
        TerrainEngineNode* terrain = mapNode->getTerrainEngine();
        terrain->addEffect( app.dt );

        viewer.setSceneData( node );
        viewer.run();
    }
    else
    {
        return usage("no earth file");
    }

    return 0;
}
