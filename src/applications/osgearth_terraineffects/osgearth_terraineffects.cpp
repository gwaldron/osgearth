/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
 * This sample tests all the various built-in TerrainEffect classes and
 * lets you toggle them and try them together.
 */
#include <osgEarth/VirtualProgram>
#include <osgEarth/Registry>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/MapNode>

#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/Controls>

#include <osgEarthUtil/ContourMap>
#include <osgEarthUtil/VerticalScale>

#include <osgEarthSymbology/Color>

#include <osg/Notify>
#include <osg/Fog>
#include <osgViewer/Viewer>

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
    TerrainEngineNode*           engine;

    osg::ref_ptr<ContourMap>     contourMap;
    osg::ref_ptr<VerticalScale>  verticalScale;

    App()
    {
        contourMap = new ContourMap();
        verticalScale = new VerticalScale();
    }
};


#define SET_FLOAT( effect, func ) \
struct func : public ui::ControlEventHandler { \
    App& _app; \
    func (App& app) : _app(app) { } \
    void onValueChanged(ui::Control*, float value) { \
        _app. effect -> func (value); \
    } \
};

#define TOGGLE( effect ) \
struct Toggle : public ui::ControlEventHandler { \
    App& _app; \
    Toggle (App& app) : _app(app) { } \
    void onValueChanged(ui::Control*, bool value) { \
        if ( value ) _app.engine->addEffect( _app. effect .get() ); \
        else _app.engine->removeEffect( _app. effect .get() ); \
    } \
};

struct ContourMapController {
    TOGGLE   ( contourMap );
    SET_FLOAT( contourMap, setOpacity );
    ContourMapController(App& app, ui::Grid* grid) {
        int r = grid->getNumRows();
        grid->setControl(0, r, new ui::LabelControl("ContourMap"));
        grid->setControl(1, r, new ui::CheckBoxControl(false, new Toggle(app)));
        ++r;
        grid->setControl(0, r, new ui::LabelControl("   opacity:"));
        grid->setControl(1, r, new ui::HSliderControl(0.0, 1.0, 0.5, new setOpacity(app)));
        grid->setControl(2, r, new ui::LabelControl(grid->getControl(1, r)));
    }
};

struct LODBlendingController {
    TOGGLE   ( lodBlending );
    SET_FLOAT( lodBlending, setDelay );
    SET_FLOAT( lodBlending, setDuration );
    SET_FLOAT( lodBlending, setVerticalScale );
    LODBlendingController(App& app, ui::Grid* grid) {
        int r = grid->getNumRows();
        grid->setControl(0, r, new ui::LabelControl("LOD Blending"));
        grid->setControl(1, r, new ui::CheckBoxControl(false, new Toggle(app)));
        ++r;
        grid->setControl(0, r, new ui::LabelControl("   delay:"));
        grid->setControl(1, r, new ui::HSliderControl(0.0, 2.0, 1.0, new setDelay(app)));
        grid->setControl(2, r, new ui::LabelControl(grid->getControl(1, r)));
        ++r;
        grid->setControl(0, r, new ui::LabelControl("   duration:"));
        grid->setControl(1, r, new ui::HSliderControl(0.0, 5.0, 1.0, new setDuration(app)));
        grid->setControl(2, r, new ui::LabelControl(grid->getControl(1, r)));
        ++r;
        grid->setControl(0, r, new ui::LabelControl("   vertical scale:"));
        grid->setControl(1, r, new ui::HSliderControl(0.0, 5.0, 1.0, new setVerticalScale(app)));
        grid->setControl(2, r, new ui::LabelControl(grid->getControl(1, r)));
    }
};

struct VerticalScaleController {
    TOGGLE   ( verticalScale );
    SET_FLOAT( verticalScale, setScale );
    VerticalScaleController(App& app, ui::Grid* grid) {
        int r = grid->getNumRows();
        grid->setControl(0, r, new ui::LabelControl("VerticalScale"));
        grid->setControl(1, r, new ui::CheckBoxControl(false, new Toggle(app)));
        ++r;
        grid->setControl(0, r, new ui::LabelControl("   scale:"));
        grid->setControl(1, r, new ui::HSliderControl(0.0, 10.0, 1.0, new setScale(app)));
        grid->setControl(2, r, new ui::LabelControl(grid->getControl(1, r)));
    }
};


// Build a slider to adjust the vertical scale
ui::Control* createUI( App& app )
{
    Grid* grid = new Grid();
    grid->setAbsorbEvents( true );
    grid->setControl(0, 0, new LabelControl("Terrain Effects", Color::Yellow) );
    grid->setControl(1, 0, new LabelControl(""));
    grid->getControl(1, 0)->setHorizFill( true, 250 );

    ContourMapController    (app, grid);
    LODBlendingController   (app, grid);
    VerticalScaleController (app, grid);

    return grid;
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

    // Set up the app create the UI:
    App app;
    ui::Control* demo_ui = createUI(app);

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load( arguments, &viewer, demo_ui );
    if ( node )
    {
        MapNode* mapNode = MapNode::get(node);

        if ( !mapNode )
            return -1;

        app.engine = mapNode->getTerrainEngine();

        viewer.setSceneData( node );
        viewer.run();
    }
    else
    {
        return usage("no earth file");
    }

    return 0;
}
