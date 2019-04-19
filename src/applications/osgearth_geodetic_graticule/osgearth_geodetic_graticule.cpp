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

#include <osg/Notify>
#include <osgViewer/Viewer>
#include <osgEarth/MapNode>
#include <osgEarth/GLUtils>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/GeodeticGraticule>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthSymbology/Style>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Symbology;
namespace ui = osgEarth::Util::Controls;

int
usage( char** argv, const std::string& msg )
{
    OE_NOTICE 
        << msg << std::endl
        << "USAGE: " << argv[0] << " file.earth" << std::endl;
    return -1;
}

const osg::Vec4 colors[4] = { osg::Vec4(1,1,1,1), osg::Vec4(1,0,0,1), osg::Vec4(1,1,0,1), osg::Vec4(0,1,0,1) };

struct App
{
    GeodeticGraticule* graticule;

    int gridColorIndex;
    int edgeColorIndex;

    App(GeodeticGraticule* g)
    {
        graticule = g;
        gridColorIndex = 0;
        edgeColorIndex = 1;
    }

    void cycleStyles()
    {
        // Could also use the get/setGridLabelStyle API here, but this demonstrates
        // changing the options and calling dirty() or apply().
        Style gridLabelStyle = graticule->options().gridLabelStyle().get();
        gridColorIndex = (gridColorIndex+1)%4;
        gridLabelStyle.getOrCreate<TextSymbol>()->fill()->color() = colors[gridColorIndex];
        graticule->options().gridLabelStyle() = gridLabelStyle;

        Style edgeLabelStyle = graticule->options().edgeLabelStyle().get(); //graticule->getEdgeLabelStyle();
        edgeColorIndex = (edgeColorIndex+1)%4;
        edgeLabelStyle.getOrCreate<TextSymbol>()->fill()->color() = colors[edgeColorIndex];
        graticule->options().edgeLabelStyle() = edgeLabelStyle;

        graticule->dirty();
    }

    void toggleGridLabels()
    {
        bool vis = graticule->getGridLabelsVisible();
        graticule->setGridLabelsVisible(!vis);
    }

    void toggleEdgeLabels()
    {
        bool vis = graticule->getEdgeLabelsVisible();
        graticule->setEdgeLabelsVisible(!vis);
    }

    void toggleGrid()
    {
        bool vis = graticule->getGridLinesVisible();
        graticule->setGridLinesVisible(!vis);
    }

    void toggleLayer()
    {
        bool vis = graticule->getVisible();
        graticule->setVisible(!vis);
    }
};

OE_UI_HANDLER(cycleStyles);
OE_UI_HANDLER(toggleGridLabels);
OE_UI_HANDLER(toggleEdgeLabels);
OE_UI_HANDLER(toggleGrid);
OE_UI_HANDLER(toggleLayer);

ui::Control* makeUI(App& app)
{
    ui::VBox* b = new ui::VBox();
    b->addChild(new ui::ButtonControl("Change styles", new cycleStyles(app)));
    b->addChild(new ui::ButtonControl("Toggle grid labels", new toggleGridLabels(app)));
    b->addChild(new ui::ButtonControl("Toggle edge labels", new toggleEdgeLabels(app)));
    b->addChild(new ui::ButtonControl("Toggle grid visibility", new toggleGrid(app)));
    b->addChild(new ui::ButtonControl("Toggle layer visibility", new toggleLayer(app)));
    return b;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);

    GLUtils::setGlobalDefaults(viewer.getCamera()->getOrCreateStateSet());
    viewer.setRealizeOperation(new GL3RealizeOperation());
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    // load the .earth file from the command line.
    MapNode* mapNode = MapNode::load(arguments);
    if ( !mapNode )
        return usage( argv, "Failed to load a map from the .earth file" );

    viewer.setSceneData(mapNode);
    viewer.setCameraManipulator( new EarthManipulator() );

    GeodeticGraticule* graticule = new GeodeticGraticule();
    mapNode->getMap()->addLayer(graticule);

    App app(graticule);
    ui::ControlCanvas* canvas = ui::ControlCanvas::getOrCreate(&viewer);
    canvas->addControl(makeUI(app));

    return viewer.run();
}
