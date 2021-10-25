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
    osg::observer_ptr<LifeMapLayer> _lifemap;
    bool _lifemap_direct;
    float _rugged_power;
    float _dense_power;
    float _lush_power;

    LifeMapGUI(App& app) : GUI::BaseGUI("Life Map"), _app(app),
        _lifemap_direct(false),
        _rugged_power(1.0f),
        _dense_power(1.0f),
        _lush_power(1.0f)
    {
        //nop
    }

    void draw(osg::RenderInfo& ri) override
    {
        if (!findLayerOrHide(_lifemap, ri))
            return;

        ImGui::Begin("LifeMap");

        // lifemap adjusters

        ImGui::TextColored(ImVec4(1, 1, 0, 1), "Runtime Adjustments");
        ImGui::Indent();

        if (ImGui::Checkbox("LifeMap Direct Set", &_lifemap_direct))
        {
            if (_lifemap_direct) {
                stateset(ri)->setDefine("OE_LIFEMAP_DIRECT", "1", 0x7);
                _rugged_power = std::min(_rugged_power, 1.0f);
                _dense_power = std::min(_dense_power, 1.0f);
                _lush_power = std::min(_lush_power, 1.0f);
            }
            else {
                stateset(ri)->setDefine("OE_LIFEMAP_DIRECT", "0", 0x7);
            }
        }

        float lm_max = _lifemap_direct ? 1.0f : 4.0f;

        ImGui::SliderFloat(_lifemap_direct ? "Dense" : "Dense multiplier", &_dense_power, 0.0f, lm_max);
        stateset(ri)->addUniform(new osg::Uniform("dense_power", _dense_power));

        ImGui::SliderFloat(_lifemap_direct ? "Rugged" : "Rugged multiplier", &_rugged_power, 0.0f, lm_max);
        stateset(ri)->addUniform(new osg::Uniform("rugged_power", _rugged_power));

        ImGui::SliderFloat(_lifemap_direct ? "Lush" : "Lush multiplier", &_lush_power, 0.0f, lm_max);
        stateset(ri)->addUniform(new osg::Uniform("lush_power", _lush_power));

        ImGui::Unindent();
        ImGui::Separator();

        ImGui::TextColored(ImVec4(1, 1, 0, 1), "Generator Settings");
        ImGui::Indent();

        LifeMapLayer::Options& o = _lifemap->options();

        ImGui::TextColored(ImVec4(1,1,0,1),"LifeMap contributions levels:");
        ImGui::Separator();
        ImGui::SliderFloat("Landcover contrib", &o.landCoverWeight().mutable_value(), 0.0f, 1.0f);
        ImGui::Indent();
        float value = o.landCoverBlur()->as(Units::METERS);
        if (ImGui::SliderFloat("Landcover blur (m)", &value, 0.0f, 100.0f))
            o.landCoverBlur()->set(value, Units::METERS);
        ImGui::Unindent();

        if (_lifemap->getColorLayer())
        {
            ImGui::SliderFloat("Imagery color contrib", &o.colorWeight().mutable_value(), 0.0f, 1.0f);
        }

        ImGui::SliderFloat("Terrain contrib", &o.terrainWeight().mutable_value(), 0.0f, 1.0f);
        ImGui::Indent();
        ImGui::SliderFloat("Slope contrib", &o.slopeIntensity().mutable_value(), 1.0f, 10.0f);
        ImGui::SliderFloat("Slope cutoff", &o.slopeCutoff().mutable_value(), 0.0f, 1.0f);
        ImGui::Unindent();

        if (_lifemap->getLandUseLayer())
        {
            ImGui::SliderFloat("Land Use contrib", &o.landUseWeight().mutable_value(), 0.0f, 1.0f);
        }

        ImGui::SliderFloat("Noise contrib", &o.noiseWeight().mutable_value(), 0.0f, 1.0f);

        if (ImGui::Button("Apply Changes"))
        {
            // make sure the cache is off
            _lifemap->setCachePolicy(CachePolicy::NO_CACHE);

            _app._mapNode->getTerrainEngine()->invalidateRegion(
                { _lifemap.get() },
                GeoExtent::INVALID);
        }

        ImGui::Unindent();

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
    float _normal_power;
    float _ao_power;
    float _brightness;
    float _contrast;
    float _dense_contrast;
    float _dense_brightness;
    float _snow;
    float _snow_min_elev;
    float _snow_max_elev;

    TextureSplattingGUI(App& app) : GUI::BaseGUI("Splatting"), _app(app)
    {
        _installed = false;
        _blend_start = 2500.0f;
        _blend_end = 500.0f;
        _blend_rgbh_mix = 0.85f;
        _blend_normal_mix = 0.85f;
        _depth = 0.02f;
        _normal_power = 1.0f;
        _ao_power = 1.0f;
        _brightness = 1.0f;
        _contrast = 1.0f;
        _dense_contrast = 0.0f;
        _dense_brightness = 0.0f;
        _snow = 0.0f;
        _snow_min_elev = 0.0f;
        _snow_max_elev = 3500.0f;
    }

    void load(const Config& conf) override
    {
        conf.get("brightness", _brightness);
        conf.get("contrast", _contrast);
        conf.get("dense_contrast", _dense_contrast);
        conf.get("dense_brightness", _dense_brightness);
        conf.get("snow", _snow);
        conf.get("snow_min_elev", _snow_min_elev);
        conf.get("snow_max_elev", _snow_max_elev);
    }

    void save(Config& conf) override
    {
        conf.set("brightness", _brightness);
        conf.set("contrast", _contrast);
        conf.set("dense_contrast", _dense_contrast);
        conf.set("dense_brightness", _dense_brightness);
        conf.set("snow", _snow);
        conf.set("snow_min_elev", _snow_min_elev);
        conf.set("snow_max_elev", _snow_max_elev);
    }

    void draw(osg::RenderInfo& ri) override
    {
        ImGui::Begin("Ground");

        if (!_installed)
        {
            // activate tweakable uniforms
            stateset(ri)->setDataVariance(osg::Object::DYNAMIC);
            stateset(ri)->setDefine("OE_SPLAT_TWEAKS", 0x7);
        }

        // uniforms

        ImGui::SliderFloat("Normal power", &_normal_power, 0.0f, 4.0f);
        stateset(ri)->addUniform(new osg::Uniform("normal_power", _normal_power));

        ImGui::SliderFloat("AO power", &_ao_power, 0.0f, 16.0f);
        stateset(ri)->addUniform(new osg::Uniform("ao_power", _ao_power));

        ImGui::SliderFloat("Displacement depth", &_depth, 0.001f, 0.3f);
        stateset(ri)->addUniform(new osg::Uniform("oe_depth", _depth));

        ImGui::SliderFloat("Level blend start (m)", &_blend_start, 0.0f, 5000.0f);
        stateset(ri)->addUniform(new osg::Uniform("oe_splat_blend_start", _blend_start));

        ImGui::SliderFloat("Level blend end (m)", &_blend_end, 0.0f, 5000.0f);
        stateset(ri)->addUniform(new osg::Uniform("oe_splat_blend_end", _blend_end));

        ImGui::SliderFloat("RGBH mix", &_blend_rgbh_mix, 0.0f, 1.0f);
        stateset(ri)->addUniform(new osg::Uniform("oe_splat_blend_rgbh_mix", _blend_rgbh_mix));

        ImGui::SliderFloat("Normal mix", &_blend_normal_mix, 0.0f, 1.0f);
        stateset(ri)->addUniform(new osg::Uniform("oe_splat_blend_normal_mix", _blend_normal_mix));

        ImGui::SliderFloat("Global brightness", &_brightness, 0.0f, 4.0f);
        stateset(ri)->addUniform(new osg::Uniform("brightness", _brightness));

        ImGui::SliderFloat("Global contrast", &_contrast, 0.0f, 4.0f);
        stateset(ri)->addUniform(new osg::Uniform("contrast", _contrast));

        ImGui::SliderFloat("Density contrast boost", &_dense_contrast, -1.0f, 1.0f);
        stateset(ri)->addUniform(new osg::Uniform("dense_contrast", _dense_contrast));

        ImGui::SliderFloat("Density brightness boost", &_dense_brightness, -1.0f, 1.0f);
        stateset(ri)->addUniform(new osg::Uniform("dense_brightness", _dense_brightness));

        ImGui::SliderFloat("Snow", &_snow, 0.0f, 1.0f);
        stateset(ri)->addUniform(new osg::Uniform("oe_snow", _snow));

        ImGui::SliderFloat("Snow bottom elev", &_snow_min_elev, 0.0f, 2500.0f);
        stateset(ri)->addUniform(new osg::Uniform("oe_snow_min_elev", _snow_min_elev));

        ImGui::SliderFloat("Snow top elev", &_snow_max_elev, 2500.0f, 5000.0f);
        stateset(ri)->addUniform(new osg::Uniform("oe_snow_max_elev", _snow_max_elev));

        ImGui::End();
    }
};

struct VegetationGUI : public GUI::BaseGUI
{
    using Clock = std::chrono::steady_clock;

    App _app;
    bool _first;
    float _sse;
    bool _auto_sse;
    std::queue<float> _times;
    float _total;
    float _sse_target_ms;
    bool _forceGenerate;
    bool _manualBiomes;
    std::unordered_map<const Biome*, bool> _isManualBiomeActive;
    Clock::time_point _lastVisit;
    osg::ref_ptr<osg::Uniform> _sseUni;
    osg::observer_ptr<BiomeLayer> _biolayer;
    osg::observer_ptr<VegetationLayer> _veglayer;
    float _maxMaxRanges[NUM_ASSET_GROUPS];

    VegetationGUI(App& app) : GUI::BaseGUI("Vegetation"),
        _app(app),
        _first(true),
        _sse(100.0f),
        _auto_sse(false),
        _forceGenerate(false),
        _manualBiomes(false),
        _sse_target_ms(15.0f)
    {
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

        if (_first)
        {
            _first = false;
            for (int i = 0; i < NUM_ASSET_GROUPS; ++i)
                _maxMaxRanges[i] = _veglayer->options().groups()[i].maxRange().get() * 2.0f;
        }

        Clock::time_point now = Clock::now();
        float elapsed_ms = (float)(std::chrono::duration_cast<std::chrono::milliseconds>(now - _lastVisit).count());
        _lastVisit = now;
        _times.push(elapsed_ms);
        _total += _times.back();
        _total -= _times.front();
        _times.pop();
        float t = _total / (float)_times.size();

        ImGui::Begin("Vegetation", nullptr, ImGuiWindowFlags_MenuBar);
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

            if (ImGui::Checkbox("Regenerate vegetation every frame", &_forceGenerate))
            {
                _veglayer->setAlwaysGenerate(_forceGenerate);
            }

            ImGui::Text("Num tiles: %d", _veglayer->getNumTilesRendered());

            ImGui::Text("Groups:");
            ImGui::Indent();
            for (int i = 0; i < NUM_ASSET_GROUPS; ++i)
            {
                ImGui::PushID(i);

                if (i > 0)
                    ImGui::Separator();

                AssetGroup::Type type = (AssetGroup::Type)i;

                ImGui::Checkbox(
                    AssetGroup::name(type).c_str(), 
                    &_veglayer->options().group(type).enabled().mutable_value());

                if (ImGui::SliderFloat(
                    "Range",
                    &_veglayer->options().group(type).maxRange().mutable_value(),
                    0.0f, _maxMaxRanges[i]))
                {
                    _veglayer->setMaxRange(type, _veglayer->options().group(type).maxRange().get());
                }

                ImGui::PopID();
            }
            ImGui::Unindent();

            if (ImGui::CollapsingHeader("All Biomes"))
            {
                auto layer = _app._map->getLayer<BiomeLayer>();
                auto biocat = layer->getBiomeCatalog();
                auto& bioman = layer->getBiomeManager();

                if (ImGui::Checkbox("Select biomes manually", &_manualBiomes))
                {
                    layer->setAutoBiomeManagement(!_manualBiomes);

                    if (_manualBiomes)
                    {
                        _isManualBiomeActive.clear();
                        for (auto biome : biocat->getBiomes())
                            _isManualBiomeActive[biome] = false;
                    }
                    else
                    {
                        stateset(ri)->setDefine("OE_BIOME_INDEX", "-1", 0x7);
                    }
                }

                ImGui::Separator();

                if (_manualBiomes)
                {
                    for (auto biome : biocat->getBiomes())
                    {
                        ImGui::PushID(biome);
                        char buf[255];
                        sprintf(buf, "[%s] %s", biome->id()->c_str(), biome->name()->c_str());
                        if (ImGui::Checkbox(buf, &_isManualBiomeActive[biome]))
                        {
                            if (_isManualBiomeActive[biome])
                            {
                                bioman.ref(biome);
                                stateset(ri)->setDefine("OE_BIOME_INDEX", std::to_string(biome->index()), 0x7);
                            }
                            else
                            {
                                bioman.unref(biome);
                            }
                        }
                        ImGui::PopID();
                    }
                }
                else
                {
                    for (auto biome : biocat->getBiomes())
                    {
                        char buf[255];
                        sprintf(buf, "[%s] %s", biome->id()->c_str(), biome->name()->c_str());
                        ImGui::Text(buf);
                    }
                }
            }

            if (ImGui::CollapsingHeader("Active Biomes", ImGuiTreeNodeFlags_DefaultOpen))
            {
                auto biocat = _app._map->getLayer<BiomeLayer>()->getBiomeCatalog();
                auto& bioman = _app._map->getLayer<BiomeLayer>()->getBiomeManager();
                auto biomes = bioman.getActiveBiomes();
                for (auto biome : biomes)
                {
                    char buf[255];
                    sprintf(buf, "[%s] %s", biome->id()->c_str(), biome->name()->c_str());
                    if (ImGui::TreeNode(buf))
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
        add("Procedural", new VegetationGUI(app), true);
        add("Procedural", new TextureSplattingGUI(app), true);
    }

    App& _app;
    LifeMapGUI _lifemap;
    VegetationGUI _biomes;
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
        osg::ref_ptr<osg::Image> image;

        image = osgDB::readRefImageFile(infile + ".oe_splat_rgbh");
        if (image.valid())
            osgDB::writeImageFile(*image.get(), infile + ".oe_splat_rgbh");

        image = osgDB::readRefImageFile(infile + ".oe_splat_nnra");
        if (image.valid())
            osgDB::writeImageFile(*image.get(), infile + ".oe_splat_nnra");
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
    
    App app;
    app._view = &viewer;
    app._manip = new EarthManipulator(arguments);
    viewer.setCameraManipulator(app._manip);

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags
    osg::ref_ptr<osg::Node> node = MapNodeHelper().loadWithoutControls(arguments, &viewer);
    if (node.valid())
    {
        app._mapNode = MapNode::get(node.get());
        if (!app._mapNode)
            return usage("No map node");

        app._map = app._mapNode->getMap();
        viewer.getEventHandlers().push_front(new MainGUI(arguments, app));
        viewer.setSceneData(node);

        return viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }
}
