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

#include <osgEarth/ImGuiUtils>
#include <osgEarth/OsgImGuiHandler.hpp>
#include <imgui_internal.h>

#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/Threading>
#include <osgEarth/Geocoder>
#include <osgEarth/NodeUtils>
#include <osgEarth/StateTransition>

#include <iostream>

#include <osgEarth/Metrics>


#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Contrib;

class ImGuiDemo : public OsgImGuiHandler
{
public:
    ImGuiDemo(osgViewer::View* view, MapNode* mapNode, EarthManipulator* earthManip) :
        _mapNode(mapNode),
        _earthManip(earthManip),
        _view(view)
    {
    }

protected:
    void drawUi() override
    {
        // ImGui code goes here...
        //ImGui::ShowDemoWindow();
        _layers.draw(_mapNode.get(), _view->getCamera(), _earthManip.get());
    }

    osg::ref_ptr< MapNode > _mapNode;
    osg::ref_ptr<EarthManipulator> _earthManip;
    osgViewer::View* _view;
    LayersGUI _layers;
};

// An event handler that will print out the elevation at the clicked point
struct StateTransitionHandler : public osgGA::GUIEventHandler
{
    StateTransitionHandler()
    {
    }

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());

        if (ea.getEventType() == ea.PUSH && ea.getButton() == ea.LEFT_MOUSE_BUTTON)
        {
            osg::Vec3d world;
            osgUtil::LineSegmentIntersector::Intersections hits;
            if (view->computeIntersections(ea.getX(), ea.getY(), hits))
            {
                StateTransition* stateTransition = 0;
                for (auto& i: hits.begin()->nodePath)
                {
                    stateTransition = dynamic_cast<StateTransition*>(i);
                    if (stateTransition)
                    {
                        std::vector< std::string > states = stateTransition->getStates();
                        if (!states.empty())
                        {
                            stateTransition->transitionToState(states[0]);
                        }
                        return true;
                    }
                }
            }
        }
        return false;
    }
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
    ImGuiNotifyHandler* notifyHandler = new ImGuiNotifyHandler();
    osg::setNotifyHandler(notifyHandler);
    osgEarth::setNotifyHandler(notifyHandler);

    osgEarth::initialize();

    osg::ArgumentParser arguments(&argc, argv);

    // help?
    if (arguments.read("--help"))
        return usage(argv[0]);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // install our default manipulator (do this before calling load)
    EarthManipulator* manip = new EarthManipulator(arguments);
    viewer.setCameraManipulator(manip);

    // Setup the viewer for imgui
    viewer.setRealizeOperation(new ImGuiDemo::RealizeOperation);

    viewer.realize();

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if (node)
    {
        MapNode* mapNode = MapNode::findMapNode(node);
        if (mapNode)
        {
            viewer.getEventHandlers().push_front(new ImGuiDemo(&viewer, mapNode, manip));
        }

        viewer.addEventHandler(new StateTransitionHandler());

        viewer.setSceneData(node);
        return viewer.run();// return Metrics::run(viewer);
    }
    else
    {
        return usage(argv[0]);
    }
}
