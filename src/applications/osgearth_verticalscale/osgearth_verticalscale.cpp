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

/**
 * This sample shows how to use osgEarth's built-in elevation data attributes
 * to adjust the terrain's vertical scale in real time.
 */
#include <osg/Notify>
#include <osgViewer/Viewer>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Registry>
#include <osgEarth/TerrainEngineNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/VerticalScale>

using namespace osgEarth;
using namespace osgEarth::Util;

//-------------------------------------------------------------------------


using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Symbology;
namespace ui = osgEarth::Util::Controls;


int 
usage(const char* msg)
{
    OE_NOTICE << msg << std::endl;
    return 0;
}


struct App
{
    VerticalScale* scaler;
};


struct SetScale : public ui::ControlEventHandler {
    App& _app;
    SetScale(App& app) : _app(app) {}
    void onValueChanged(ui::Control*, float value) {
        _app.scaler->setScale(value);
    }
};



// Build a slider to adjust the vertical scale
ui::Control* createUI( App& app )
{
    ui::VBox* vbox = new VBox();
    vbox->setAbsorbEvents( true );

    vbox->addControl( new LabelControl(Stringify() << "Vertical Scale Example", Color::Yellow) );

    Grid* grid = vbox->addControl( new Grid() );

    int r = 0;
    grid->setControl( 0, r, new ui::LabelControl("Scale:") );
    grid->setControl( 1, r, new ui::HSliderControl(0.0, 10.0, 1.0, new SetScale(app)) );
    grid->setControl( 2, r, new ui::LabelControl(grid->getControl(1, r)) );

    grid->getControl(1, r)->setHorizFill( true, 200 );

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

    // Set up the app and read available options:
    App app;

    // Create the UI:
    ui::Control* demo_ui = createUI(app);

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load( arguments, &viewer, demo_ui );
    if ( node )
    {
        MapNode* mapNode = MapNode::get(node);
        if ( !mapNode )
            return -1;

        // attach the effect to the terrain.
        app.scaler = new VerticalScale();
        mapNode->getTerrainEngine()->addEffect( app.scaler );

        viewer.setSceneData( node );
        viewer.run();
    }
    else
    {
        return usage("no earth file");
    }

    return 0;
}
