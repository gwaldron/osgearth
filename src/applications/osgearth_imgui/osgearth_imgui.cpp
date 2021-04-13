/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
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

#include <osgEarth/ImGui/ImGui>

#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/Threading>
#include <osgEarth/Geocoder>
#include <osgEarth/NodeUtils>

#include <iostream>

#include <osgEarth/Metrics>


#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::GUI;

struct App
{
    osgViewer::View* _view;
    osg::ref_ptr<MapNode> _mapNode;
    osg::ref_ptr<EarthManipulator> _manip;
};

struct AppGUI : public OsgImGuiHandler
{
    AppGUI(App& app) : _app(app),
        _layers_show(true),
        _search_show(true),
        _netmon_show(false),
        _scenegraph_show(true),
        _viewpoints_show(true),
        _texinspector_show(true)
    {
    }

    void drawUi(osg::RenderInfo& renderInfo) override
    {
        _layers.draw(renderInfo, _app._mapNode, _app._view->getCamera(), _app._manip);
        _search.draw(_app._manip);
        _netmon.draw(&_netmon_show);
        _scenegraph.draw(_app._view->getCamera(), renderInfo, _app._manip.get(), _app._mapNode.get(), &_scenegraph_show);
        _viewpoints.draw(_app._mapNode.get(), _app._manip.get());
        _texinspector.draw(renderInfo, _app._view->getCamera(), &_texinspector_show);
    }
    
    App& _app;
    LayersGUI _layers;
    SearchGUI _search;
    NetworkMonitorGUI _netmon;
    SceneGraphGUI _scenegraph;
    ViewpointsGUI _viewpoints;
    TextureInspectorGUI _texinspector;

    bool _layers_show;
    bool _search_show;
    bool _netmon_show;
    bool _scenegraph_show;
    bool _viewpoints_show;
    bool _texinspector_show;
};

int
usage(const char* name)
{
    OE_NOTICE
        << "\nUsage: " << name << " file.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;
    return 0;
}


int
main(int argc, char** argv)
{
    osgEarth::initialize();

    osg::ArgumentParser arguments(&argc, argv);

    // help?
    if (arguments.read("--help"))
        return usage(argv[0]);

    osgViewer::Viewer viewer(arguments);

    App app;
    app._view = &viewer;
    app._manip = new EarthManipulator(arguments);

    viewer.setCameraManipulator(app._manip);

    // Setup the viewer for imgui
    viewer.setRealizeOperation(new AppGUI::RealizeOperation);
    viewer.realize();

    // Load an earth file
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if (node)
    {
        app._mapNode = MapNode::get(node);
        if (app._mapNode)
        {
            viewer.setSceneData(node);
            viewer.addEventHandler(new AppGUI(app));
            viewer.addEventHandler(new SelectNodeHandler());
        }

        return viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }
}
