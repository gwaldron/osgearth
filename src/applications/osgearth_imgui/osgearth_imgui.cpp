/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2018 Pelican Mapping
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
#include <osgEarth/NetworkMonitor>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Profiler>

#include <iostream>

#include <osgEarth/Metrics>


#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Contrib;


class NetworkMonitorGUI
{
public:
    NetworkMonitorGUI() :
        _showOnlyActiveRequests(false),
        filter("")
    {

    }

    void draw()
    {
        ImGui::Begin("Network Monitor");
        NetworkMonitor::Requests requests;
        NetworkMonitor::getRequests(requests);
        ImGui::Checkbox("Show active requests", &_showOnlyActiveRequests);
        ImGui::InputText("Filter", filter, 128);
        ImGui::Columns(3, "requests");
        ImGui::Separator();
        ImGui::Text("Path"); ImGui::NextColumn();
        ImGui::Text("Duration (ms)"); ImGui::NextColumn();
        ImGui::Text("Status"); ImGui::NextColumn();
        ImGui::Separator();
        for (auto itr = requests.begin(); itr != requests.end(); ++itr)
        {
            if (!_showOnlyActiveRequests || !itr->second.isComplete)
            {
                if (strlen(filter) > 0)
                {
                    if (strstr(itr->second.uri.c_str(), filter) == NULL)
                    {
                        continue;
                    }
                }
                ImVec4 color = itr->second.isComplete ? ImVec4(0.0, 1.0, 0.0, 1.0) : ImVec4(1.0, 1.0, 1.0, 1.0);
                ImGui::TextColored(color, itr->second.uri.c_str());  ImGui::NextColumn();
                ImGui::Text("%f", itr->second.getDuration()); ImGui::NextColumn();
                ImGui::Text(itr->second.status.c_str()); ImGui::NextColumn();
            }
        }
        ImGui::Columns(1);
        ImGui::Separator();
        ImGui::End();
    }

    bool _showOnlyActiveRequests;
    char filter[128];
};

class ProfilerGUI
{
public:
    ProfilerGUI()
    {

    }

    void draw()
    {
        ImGui::Begin("Profiler");

        std::map<unsigned int, ThreadZones> zones;
        Profiler::getZones(zones);

        for (auto threadItr = zones.begin(); threadItr != zones.end(); ++threadItr)
        {
            std::string header = Stringify() << "Thread " << threadItr->first;
            if (ImGui::CollapsingHeader(header.c_str()))
            {
                const ZoneList& zones = threadItr->second._zones;
                drawZones(zones);
            }
        }
        ImGui::End();
        _id = 0;
    }

    void drawZones(const ZoneList& zones)
    {
        for (ZoneList::const_iterator zoneItr = zones.begin(); zoneItr != zones.end(); ++zoneItr)
        {
            ImGui::PushID(_id++);
            OE_NOTICE << "Duration " << zoneItr->duration() << std::endl;
            if (ImGui::TreeNode((const void*)(&zoneItr), "%s %lf", zoneItr->_name.c_str(), zoneItr->duration()))
            {
                if (ImGui::IsItemHovered() && !zoneItr->_zoneText.empty())
                {
                    ImGui::BeginTooltip();
                    for (auto textItr = zoneItr->_zoneText.begin(); textItr != zoneItr->_zoneText.end(); ++textItr)
                    {
                        ImGui::Text(textItr->c_str());
                    }
                    ImGui::EndTooltip();
                }
                drawZones(zoneItr->_children);
                ImGui::TreePop();
            }         
            ImGui::PopID();
        }
    }

    int _id = 0;
};

class ImGuiDemo : public OsgImGuiHandler
{
public:
    ImGuiDemo(MapNode* mapNode):
        _mapNode(mapNode)        
    {
    }

protected:
    void drawUi() override
    {
        // ImGui code goes here...
        ImGui::ShowDemoWindow();

        ImageLayerVector imageLayers;
        _mapNode->getMap()->getLayers(imageLayers);

        ImGui::Begin("Layer Timing");

        const ImVec2 p = ImGui::GetCursorScreenPos();
        float x = p.x;
        float y = p.y;       

        double totalTime = 0.0f;
        
        float width = 100.0f;
        for (unsigned int i = 0; i < imageLayers.size(); i++)
        {
            totalTime += imageLayers[i].get()->getInternalTime();
        }

        float totalHeight = 100.0f;

        for (unsigned int i = 0; i < imageLayers.size(); i++)
        {
            if (i >= colors.size())
            {
                float r = (float)rand() / (float)RAND_MAX;
                float g = (float)rand() / (float)RAND_MAX;
                float b = (float)rand() / (float)RAND_MAX;
                colors.push_back(ImVec4(r, g, b, 1.0f));
            }                    
            float h = totalHeight * imageLayers[i].get()->getInternalTime() / totalTime;
            std::string name = imageLayers[i]->getName();
            ImGui::RenderFrame(ImVec2(x, y + totalHeight), ImVec2(x + width, y+totalHeight-h), ImGui::GetColorU32(colors[i]));                        
            ImGui::RenderText(ImVec2(x, y + totalHeight), name.c_str());
            x += width;
            /*
            ImGui::ProgressBar(h);
            ImGui::SameLine();
            ImGui::Text(name.c_str());            
            */
        }

        ImGui::End();

        _networkMonitor.draw();
        _layers.draw(_mapNode);
        _profiler.draw();
    }

    NetworkMonitorGUI _networkMonitor;
    ProfilerGUI _profiler;
    LayersGUI _layers;
    MapNode *_mapNode;    
    std::vector< ImVec4 > colors;
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
    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // Tell the database pager to not modify the unref settings
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy( true, false );

    // thread-safe initialization of the OSG wrapper manager. Calling this here
    // prevents the "unsupported wrapper" messages from OSG
    osgDB::Registry::instance()->getObjectWrapperManager()->findWrapper("osg::Image");

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator( new EarthManipulator(arguments) );

    // disable the small-feature culling
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    // set a near/far ratio that is smaller than the default. This allows us to get
    // closer to the ground without near clipping. If you need more, use --logdepth
    viewer.getCamera()->setNearFarRatio(0.0001);

    // Setup the viewer for imgui
    viewer.setRealizeOperation(new GlewInitOperation);
    
    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if ( node )
    {
        MapNode* mapNode = MapNode::findMapNode(node);
        if (mapNode)
        {
            viewer.addEventHandler(new ImGuiDemo(mapNode));
        }

        viewer.setSceneData( node );
        return Metrics::run(viewer);
    }
    else
    {
        return usage(argv[0]);
    }
}