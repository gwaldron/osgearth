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

#include <osgEarth/Notify>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Metrics>
#include <osgEarth/Lighting>
#include <osgEarth/NodeUtils>
#include <osgEarth/PhongLightingEffect>

#include <osgEarthProcedural/BiomeLayer>
#include <osgEarthProcedural/BiomeManager>
#include <osgEarthProcedural/LifeMapLayer>
#include <osgEarthProcedural/VegetationLayer>

#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/Uniform>
#include <iostream>

#define LC "[osgearth_biome] "

using namespace osgEarth;
using namespace osgEarth::Util;
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
    EventRouter* _router;
    osgViewer::View* _view;
    osg::Light* _light;
};

struct LifeMapGUI : public GUI::BaseGUI
{
    App _app;
    LifeMapLayer* lifemap;

    LifeMapGUI(App& app) : GUI::BaseGUI("Life Map"), _app(app)
    {
        lifemap = _app._map->getLayer<LifeMapLayer>();
        OE_HARD_ASSERT(lifemap != nullptr);
    }

    void draw(osg::RenderInfo& ri) override
    {
        ImGui::Begin("LifeMap Tweaks");

        LifeMapLayer::Options& o = lifemap->options();
        
        ImGui::Checkbox("Use landcover data", &o.useLandCover().mutable_value());
        ImGui::Checkbox("Use terrain data", &o.useTerrain().mutable_value());
        if (o.useTerrain() == true)
        {
            ImGui::SliderFloat("Terrain weight", &o.terrainWeight().mutable_value(), 0.0f, 1.0f);
        }

        ImGui::SliderFloat(
            "Slope intensity",
            &o.slopeIntensity().mutable_value(),
            1.0f, 10.0f);

        if (ImGui::Button("Apply"))
        {
            _app._mapNode->getTerrainEngine()->invalidateRegion(
                { lifemap },
                GeoExtent::INVALID);
        }

        ImGui::End();
    }
};

struct TextureSplattingGUI : public GUI::BaseGUI
{
    App _app;
    bool _installed;
    float _blend_start;
    float _blend_end;
    float _blend_rgbh_mix;
    float _blend_normal_mix;
    float _depth;
    bool _lifemap_direct;
    float _rugged_power;
    float _dense_power;
    float _lush_power;
    float _normal_power;
    float _ao_power;
    float _brightness;
    float _contrast;
    float _snow;
    float _snow_min_elev;
    float _snow_max_elev;

    TextureSplattingGUI(App& app) : GUI::BaseGUI("Texture Splatting"), _app(app)
    {
        _installed = false;
        _blend_start = 2500.0f;
        _blend_end = 500.0f;
        _blend_rgbh_mix = 0.85f;
        _blend_normal_mix = 0.72f;
        _depth = 0.02f;
        _lifemap_direct = false;
        _rugged_power = 1.0f;
        _dense_power = 1.0f;
        _lush_power = 1.0f;
        _normal_power = 1.0f;
        _ao_power = 1.0f;
        _brightness = 1.0f;
        _contrast = 1.0f;
        _snow = 0.0f;
        _snow_min_elev = 0.0f;
        _snow_max_elev = 3500.0f;
    }

    void draw(osg::RenderInfo& ri) override
    {
        ImGui::Begin("Texture Splatting");

        if (!_installed)
        {
            // activate tweakable uniforms
            ri.getCurrentCamera()->getOrCreateStateSet()->setDefine("OSGEARTH_SPLAT_TWEAKS");
        }

        // uniforms
        ImGui::SliderFloat("Level blend start (m)", &_blend_start, 0.0f, 5000.0f);
        _app._mapNode->getOrCreateStateSet()->addUniform(new osg::Uniform("oe_splat_blend_start", _blend_start));

        ImGui::SliderFloat("Level blend end (m)", &_blend_end, 0.0f, 5000.0f);
        _app._mapNode->getOrCreateStateSet()->addUniform(new osg::Uniform("oe_splat_blend_end", _blend_end));

        ImGui::SliderFloat("RGBH mix", &_blend_rgbh_mix, 0.0f, 1.0f);
        _app._mapNode->getOrCreateStateSet()->addUniform(new osg::Uniform("oe_splat_blend_rgbh_mix", _blend_rgbh_mix));

        ImGui::SliderFloat("Normal mix", &_blend_normal_mix, 0.0f, 1.0f);
        _app._mapNode->getOrCreateStateSet()->addUniform(new osg::Uniform("oe_splat_blend_normal_mix", _blend_normal_mix));

        ImGui::SliderFloat("Displacement depth", &_depth, 0.001f, 0.3f);
        _app._mapNode->getOrCreateStateSet()->addUniform(new osg::Uniform("oe_depth", _depth));

        if (ImGui::Checkbox("LifeMap Direct Set", &_lifemap_direct)) {
            _app._mapNode->getOrCreateStateSet()->setDefine("OSGEARTH_LIFEMAP_DIRECT");
        }
        else {
            _app._mapNode->getOrCreateStateSet()->removeDefine("OSGEARTH_LIFEMAP_DIRECT");
        }
        float lm_max = _lifemap_direct ? 1.0f : 4.0f;

        ImGui::SliderFloat("Dense power", &_dense_power, 0.0f, lm_max);
        _app._mapNode->getOrCreateStateSet()->addUniform(new osg::Uniform("dense_power", _dense_power));

        ImGui::SliderFloat("Rugged power", &_rugged_power, 0.0f, lm_max);
        _app._mapNode->getOrCreateStateSet()->addUniform(new osg::Uniform("rugged_power", _rugged_power));

        ImGui::SliderFloat("Lush power", &_lush_power, 0.0f, lm_max);
        _app._mapNode->getOrCreateStateSet()->addUniform(new osg::Uniform("lush_power", _lush_power));


        ImGui::SliderFloat("Normal power", &_normal_power, 0.0f, 4.0f);
        _app._mapNode->getOrCreateStateSet()->addUniform(new osg::Uniform("normal_power", _normal_power));

        ImGui::SliderFloat("AO power", &_ao_power, 0.0f, 16.0f);
        _app._mapNode->getOrCreateStateSet()->addUniform(new osg::Uniform("ao_power", _ao_power));

        ImGui::SliderFloat("Global brightness", &_brightness, 0.0f, 4.0f);
        _app._mapNode->getOrCreateStateSet()->addUniform(new osg::Uniform("brightness", _brightness));

        ImGui::SliderFloat("Global contrast", &_contrast, 0.0f, 4.0f);
        _app._mapNode->getOrCreateStateSet()->addUniform(new osg::Uniform("contrast", _contrast));

        ImGui::SliderFloat("Snow", &_snow, 0.0f, 1.0f);
        _app._mapNode->getOrCreateStateSet()->addUniform(new osg::Uniform("oe_snow", _snow));

        ImGui::SliderFloat("Snow bottom elev", &_snow_min_elev, 0.0f, 2500.0f);
        _app._mapNode->getOrCreateStateSet()->addUniform(new osg::Uniform("oe_snow_min_elev", _snow_min_elev));

        ImGui::SliderFloat("Snow top elev", &_snow_max_elev, 2500.0f, 5000.0f);
        _app._mapNode->getOrCreateStateSet()->addUniform(new osg::Uniform("oe_snow_max_elev", _snow_max_elev));

        ImGui::End();
    }
};

struct BiomeGUI : public GUI::BaseGUI
{
    using Clock = std::chrono::steady_clock;

    App _app;
    float _sse;
    bool _auto_sse;
    std::queue<float> _times;
    float _total;
    float _sse_target_ms;
    bool _forceGenerate;
    Clock::time_point _lastVisit;
    osg::ref_ptr<osg::Uniform> _sseUni;
    osg::observer_ptr<BiomeLayer> _biolayer;
    osg::observer_ptr<VegetationLayer> _veglayer;

    BiomeGUI(App& app) : GUI::BaseGUI("Biomes"),
        _app(app),
        _sse(100.0f),
        _auto_sse(false),
        _forceGenerate(false),
        _sse_target_ms(15.0f)
    {
        //nop
        for (int i = 0; i < 60; ++i) _times.push(0.0f);
        _total = 0.0f;
    }

    void load(const Config& conf) override
    {
        conf.get("SSE", _sse);
        conf.get("AUTO_SSE", _auto_sse);
    }

    void save(Config& conf) override
    {
        conf.set("SSE", _sse);
        conf.set("AUTO_SSE", _auto_sse);
    }

    void draw(osg::RenderInfo& ri) override
    {
        if (!findLayerOrHide(_veglayer, ri))
            return;
        if (!findLayerOrHide(_biolayer, ri))
            return;

        Clock::time_point now = Clock::now();
        float elapsed_ms = (float)(std::chrono::duration_cast<std::chrono::milliseconds>(now - _lastVisit).count());
        _lastVisit = now;
        _times.push(elapsed_ms);
        _total += _times.back();
        _total -= _times.front();
        _times.pop();
        float t = _total / (float)_times.size();

        ImGui::Begin("Biomes", NULL, ImGuiWindowFlags_MenuBar);
        {
            if (ImGui::SliderFloat("SSE", &_sse, 1.0f, 1000.0f)) {
                _veglayer->setMaxSSE(_sse);
                dirtySettings();
            }
            
            ImGui::SameLine();
            if (ImGui::Checkbox("Auto", &_auto_sse)) {
                dirtySettings();
            }

            if (_auto_sse)
            {
                if (t > _sse_target_ms*2.0f)
                {
                    _sse = clamp(_sse + 4.0f, 1.0f, 1000.0f);
                    _veglayer->setMaxSSE(_sse);
                }
                else if (t > _sse_target_ms*1.01f)
                {
                    _sse = clamp(_sse+0.25f, 1.0f, 1000.0f);
                    _veglayer->setMaxSSE(_sse);
                }
                else if (t < _sse_target_ms*0.8f)
                {
                    _sse = clamp(_sse-0.1f, 1.0f, 1000.0f);
                    _veglayer->setMaxSSE(_sse);
                }
            }

            //if (ImGui::Checkbox("Always generate", &_forceGenerate)) {
            //    _app._map->getLayer<VegetationLayer>()->setGenerate(_forceGenerate);
            //}

            if (ImGui::CollapsingHeader("Active Biomes", ImGuiTreeNodeFlags_DefaultOpen))
            {
                auto biocat = _app._map->getLayer<BiomeLayer>()->getBiomeCatalog();
                auto& bioman = _app._map->getLayer<BiomeLayer>()->getBiomeManager();
                auto biomes = bioman.getActiveBiomes();
                for (auto biome : biomes)
                {
                    if (ImGui::TreeNode(biome->name()->c_str()))
                    {
                        for(int group=0; group<NUM_ASSET_GROUPS; ++group)
                        {
                            std::string groupName = group == 0 ? "Trees" : "Undergrowth";

                            if (ImGui::TreeNode(groupName.c_str()))
                            {
                                for(auto& pointer : biome->assetPointers(group))
                                {
                                    if (ImGui::TreeNode(pointer.asset->name()->c_str()))
                                    {
                                        drawModelAsset(pointer.asset);
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
        }
        ImGui::End();
    }

    void drawModelAsset(const ModelAsset* asset)
    {
        if (asset->modelURI().isSet())
            ImGui::Text("Model: %s", asset->modelURI()->base().c_str());
        if (asset->sideBillboardURI().isSet())
            ImGui::Text("Side BB: %s", asset->sideBillboardURI()->base().c_str());
        if (asset->topBillboardURI().isSet())
            ImGui::Text("Top BB: %s", asset->topBillboardURI()->base().c_str());
    }
};

class MainGUI : public GUI::ApplicationGUI
{
public:
    MainGUI(osg::ArgumentParser& args, App& app) : 
        _app(app), 
        _lifemap(app),
        _biomes(app),
        _splatting(app)
    {
        addAllBuiltInTools(&args);

        add("Procedural", new LifeMapGUI(app), true);
        add("Procedural", new BiomeGUI(app), true);
        add("Procedural", new TextureSplattingGUI(app), true);
    }

    App& _app;
    LifeMapGUI _lifemap;
    BiomeGUI _biomes;
    TextureSplattingGUI _splatting;
};

osg::Vec4
worldToVec4(const osg::Vec3d& ecef)
{
    osg::Vec4 result(0.0f, 0.0f, 0.0f, 1.0f);
    osg::Vec3d d = ecef;
    while (d.length() > 1e6)
    {
        d *= 0.1;
        result.w() *= 0.1;
    }
    return osg::Vec4(d.x(), d.y(), d.z(), result.w());
}

// For testing the splatting normal maps
void
setupMouseLight(App& app)
{
    SkyNode* sky = osgEarth::findTopMostNodeOfType<SkyNode>(app._view->getSceneData());
    if (!sky)
    {
        PhongLightingEffect* phong = new PhongLightingEffect();
        phong->attach(app._view->getSceneData()->getOrCreateStateSet());
    }

    app._light = new osg::Light(sky ? 1 : 0);
    app._light->setAmbient(osg::Vec4(0.2, 0.2, 0.2, 1));
    app._light->setDiffuse(osg::Vec4(1, 1, 1, 1));
    osg::LightSource* ls = new osg::LightSource();
    ls->setLight(app._light);
    app._mapNode->addChild(ls);
    GenerateGL3LightingUniforms gen;
    ls->accept(gen);
    app._light = ls->getLight();

    app._router->onMove([&](osg::View* view, float x, float y) {
        osg::Vec3d world;
        if (app._mapNode->getTerrain()->getWorldCoordsUnderMouse(view, x, y, world)) {
            osg::Vec3d n = world;
            n.normalize();
            world += n * 1.0;
            app._light->setPosition(worldToVec4(world));
        }
    });
}

int
encodeTexture(osg::ArgumentParser& args)
{
    std::string infile;
    if (args.read("--encode-texture", infile))
    {
        std::size_t pos = infile.find("_Color.", 0);
        if (pos >= 0)
        {
            std::string base = infile.substr(0, pos);
            std::string ext = osgDB::getFileExtension(infile);
            osg::ref_ptr<osg::Image> image;
            image = osgDB::readRefImageFile(base + ".oe_splat_rgbh");
            if (image.valid())
                osgDB::writeImageFile(*image.get(), base + ".oe_splat_rgbh");
            image = osgDB::readRefImageFile(base + ".oe_splat_nnra");
            if (image.valid())
                osgDB::writeImageFile(*image.get(), base + ".oe_splat_nnra");
        }
    }
    return 0;
}

int
main(int argc, char** argv)
{
    osgEarth::initialize();

    osg::ArgumentParser arguments(&argc, argv);
    if (arguments.read("--help"))
        return usage(argv[0]);

    if (arguments.find("--encode-texture") >= 0)
        return encodeTexture(arguments);

    osgViewer::Viewer viewer(arguments);
    viewer.setThreadingModel(viewer.SingleThreaded);
    viewer.setRealizeOperation(new MainGUI::RealizeOperation());

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags
    osg::Node* node = MapNodeHelper().loadWithoutControls(arguments, &viewer);
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
        viewer.getEventHandlers().push_front(new MainGUI(arguments, app));
        viewer.setSceneData(node);

        return viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }
}
