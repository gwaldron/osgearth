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
#include <iostream>
#include <osgEarth/Metrics>

#include <osgEarthProcedural/BiomeLayer>
#include <osgEarthProcedural/BiomeManager>

#define LC "[osgearth_biome] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Contrib;
using namespace osgEarth::Procedural;

int
usage(const char* name)
{
    OE_NOTICE
        << "\nUsage: " << name << " file.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;
    return 0;
}

struct App
{
    const Map* _map;
    MapNode* _mapNode;
    EarthManipulator* _manip;
    osgViewer::View* _view;
};

struct BiomeGUI
{
    App _app;
    BiomeGUI(App& app) : _app(app)
    {
        BiomeLayer* biolayer = _app._map->getLayer<BiomeLayer>();
        OE_HARD_ASSERT(biolayer != nullptr, __func__);
    }

    void draw()
    {
        ImGui::Begin("Biomes", NULL, ImGuiWindowFlags_MenuBar);

        if (ImGui::CollapsingHeader("Active Biomes", ImGuiTreeNodeFlags_DefaultOpen))
        {
            auto biocat = _app._map->getLayer<BiomeLayer>()->getBiomeCatalog();
            auto& bioman = _app._map->getLayer<BiomeLayer>()->getBiomeManager();
            auto biomes = bioman.getActiveBiomes();
            for (auto biome : biomes)
            {
                if (ImGui::TreeNode(biome->name()->c_str()))
                {
                    for (auto cat : biome->modelCategories())
                    {
                        if (ImGui::TreeNode(cat.name()->c_str()))
                        {
                            for (auto& member : cat.members())
                            {
                                if (ImGui::TreeNode(member.asset->name()->c_str()))
                                {
                                    drawModelAsset(member.asset);
                                    ImGui::TreePop();
                                }
                            }
                            ImGui::TreePop();
                        }
                    }
                    ImGui::TreePop();
                }
            }
        }

        if (ImGui::CollapsingHeader("Resident Assets", ImGuiTreeNodeFlags_DefaultOpen))
        {
            auto biocat = _app._map->getLayer<BiomeLayer>()->getBiomeCatalog();
            auto& bioman = _app._map->getLayer<BiomeLayer>()->getBiomeManager();

            auto assets = bioman.getResidentAssets();
            for (auto& entry : assets)
            {
                const ModelAsset* asset = entry.first;
                const auto& data = entry.second;

                if (ImGui::TreeNode(asset->name()->c_str()))
                {
                    drawModelAsset(asset);
                    ImGui::TreePop();
                }
            }
        }

        ImGui::End();
    }

    void drawModelAsset(const ModelAsset* asset) //, const ModelAssetData::Pointer& data)
    {
        if (asset->modelURI().isSet())
            ImGui::Text("Model: %s", asset->modelURI()->base().c_str());
        if (asset->sideBillboardURI().isSet())
            ImGui::Text("Side BB: %s", asset->sideBillboardURI()->base().c_str());
        if (asset->topBillboardURI().isSet())
            ImGui::Text("Top BB: %s", asset->topBillboardURI()->base().c_str());
    }
};

class MainGUI : public OsgImGuiHandler
{
public:
    MainGUI(App& app) : _app(app), _biomes(app)
    {
    }

protected:
    void drawUi() override
    {
        _layers.draw(_app._mapNode, _app._view->getCamera(), _app._manip);
        _biomes.draw();
    }

    App& _app;
    BiomeGUI _biomes;
    LayersGUI _layers;
};

int
main(int argc, char** argv)
{
    //ImGuiNotifyHandler* notifyHandler = new ImGuiNotifyHandler();
    //osg::setNotifyHandler(notifyHandler);
    //osgEarth::setNotifyHandler(notifyHandler);

    osgEarth::initialize();

    osg::ArgumentParser arguments(&argc, argv);
    if (arguments.read("--help"))
        return usage(argv[0]);

    osgViewer::Viewer viewer(arguments);

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if (node)
    {
        App app;

        app._mapNode = MapNode::get(node);
        if (!app._mapNode)
            return usage("No map node");

        app._map = app._mapNode->getMap();
        app._view = &viewer;
        app._manip = new EarthManipulator(arguments);
        viewer.setCameraManipulator(app._manip);
        viewer.setRealizeOperation(new MainGUI::RealizeOperation());
        viewer.realize();
        viewer.getEventHandlers().push_front(new MainGUI(app));

        viewer.setSceneData(node);
        return viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }
}
