/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#include "TextureSplattingLayer"
#include "TextureSplattingMaterials"
#include "ProceduralShaders"
#include "NoiseTextureFactory"

#include <osgEarth/TerrainEngineNode>
#include <osgEarth/TerrainResources>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Shaders>
#include <osgEarth/Capabilities>

#include <osgUtil/CullVisitor>
#include <osg/BlendFunc>
#include <osg/Drawable>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/FileNameUtils>

#include <cstdlib> // getenv

#define LC0 "[TextureSplattingLayer] "
#define LC LC0 << getName() << ": "

#define TEXTURE_ARENA_BINDING_POINT 5
#define RENDERPARAMS_BINDING_POINT  6

using namespace osgEarth::Procedural;

REGISTER_OSGEARTH_LAYER(proceduralimage, TextureSplattingLayer);
REGISTER_OSGEARTH_LAYER(procedural_image, TextureSplattingLayer);

//........................................................................

Config
TextureSplattingLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();
    conf.set("num_levels", numLevels());
    conf.set("use_hex_tiler", useHexTiler());
    return conf;
}

void
TextureSplattingLayer::Options::fromConfig(const Config& conf)
{
    numLevels().setDefault(1);
    useHexTiler().setDefault(false);
    conf.get("num_levels", numLevels());
    conf.get("use_hex_tiler", useHexTiler());
}

//........................................................................

BiomeLayer*
TextureSplattingLayer::getBiomeLayer() const
{
    return options().biomeLayer().getLayer();
}

LifeMapLayer*
TextureSplattingLayer::getLifeMapLayer() const
{
    return options().lifeMapLayer().getLayer();
}

void
TextureSplattingLayer::init()
{
    VisibleLayer::init();

    setRenderType(osgEarth::Layer::RENDERTYPE_TERRAIN_SURFACE);
}

void
TextureSplattingLayer::addedToMap(const Map* map)
{
    VisibleLayer::addedToMap(map);

    options().biomeLayer().addedToMap(map);
    if (getBiomeLayer() == nullptr)
    {
        BiomeLayer* layer = map->getLayer<BiomeLayer>();
        if (layer)
            options().biomeLayer().setLayer(layer);
    }

    options().lifeMapLayer().addedToMap(map);
    if (getLifeMapLayer() == nullptr)
    {
        LifeMapLayer* layer = map->getLayer<LifeMapLayer>();
        if (layer)
            options().lifeMapLayer().setLayer(layer);
    }

    buildStateSets();
}

void
TextureSplattingLayer::removedFromMap(const Map* map)
{
    VisibleLayer::removedFromMap(map);

    options().biomeLayer().removedFromMap(map);
    options().lifeMapLayer().removedFromMap(map);
}

void
TextureSplattingLayer::prepareForRendering(TerrainEngine* engine)
{
    VisibleLayer::prepareForRendering(engine);

    if (Capabilities::get().supportsInt64() == false)
    {
        setStatus(Status::ResourceUnavailable, "GLSL int64 support required but not available");
        return;
    }

    if (getLifeMapLayer() == nullptr)
    {
        // without a lifemap layer we can't do any splatting
        setStatus(Status::ResourceUnavailable, "No LifeMap data to splat");
        return;
    }

    // Install a general-purpose noise texture
    engine->getResources()->reserveTextureImageUnit(_noiseUnit, getName().c_str());
    if (_noiseUnit.valid())
    {
        NoiseTextureFactory ntf;
        osg::Texture* noise = ntf.create(256u, 4u);
        getOrCreateStateSet()->setTextureAttribute(_noiseUnit.unit(), noise);
        getOrCreateStateSet()->addUniform(new osg::Uniform("oe_noise", _noiseUnit.unit()));
    }

    // Since we're actually rendering, load the materials for splatting
    if (getBiomeLayer())
    {
        auto biome_cat = getBiomeLayer()->getBiomeCatalog();

        if (biome_cat && !biome_cat->getAssets().empty())
        {
            int numLevels = options().numLevels().get();

            const AssetCatalog& assets = biome_cat->getAssets();

            std::vector<GeoExtent> ex;
            ex.push_back(engine->getMap()->getProfile()->calculateExtent(14, 0, 0));
            ex.push_back(engine->getMap()->getProfile()->calculateExtent(19, 0, 0));

            std::vector<double> tile_height_m;
            tile_height_m.push_back(ex[0].height(Units::METERS));
            tile_height_m.push_back(ex[1].height(Units::METERS));

            // Function to load all material textures.
            auto loadMaterials = [assets, tile_height_m](Cancelable* c) -> Materials::Ptr
            {
                Materials::Ptr result = Materials::Ptr(new Materials);

                // contains the textures and their bindless handles
                result->_arena = new TextureArena();
                result->_arena->setName("TextureSplattingLayer");
                result->_arena->setBindingPoint(TEXTURE_ARENA_BINDING_POINT);

                // contains metadata about the textures
                result->_textureScales = new osg::Uniform();
                result->_textureScales->setName("oe_texScale");
                result->_textureScales->setType(osg::Uniform::FLOAT);
                result->_textureScales->setNumElements(
                    assets.getMaterials().size() * tile_height_m.size());

                int ptr0 = 0;
                int ptr1 = assets.getMaterials().size();

                for(auto& material : assets.getMaterials())
                {
                    auto t0 = std::chrono::steady_clock::now();

                    result->_assets.push_back(&material);

                    RGBH_NNRA_Loader::load(material.uri()->full(), result->_arena.get());

                    auto t1 = std::chrono::steady_clock::now();

                    OE_INFO << LC0 << "Loaded material " << material.uri()->base()
                        << ", t=" << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "ms"
                        << std::endl;

                    if (c && c->isCanceled())
                        return nullptr;

                    // Set up the texture scaling:
                    result->_textureScales->setElement(
                        ptr0++,
                        (float)(material.size().isSet() ? tile_height_m[0] / material.size()->as(Units::METERS) : 1.0f));

                    if (tile_height_m.size() > 1)
                    {
                        result->_textureScales->setElement(
                            ptr1++,
                            (float)(material.size().isSet() ? tile_height_m[1] / material.size()->as(Units::METERS) : 1.0f));
                    }
                }

                return result;
            };

            // Load material asynchronously
            _materialsJob = Job().dispatch<Materials::Ptr>(loadMaterials);
        }
    }
    else
    {
        // without a biome layer we have no textures to splat with
        setStatus(Status::ResourceUnavailable, "No Biome data to splat");
        return;
    }

    // default uniform values
    osg::StateSet* ss = getOrCreateStateSet();
    ss->addUniform(new osg::Uniform("oe_splat_blend_start", 2500.0f));
    ss->addUniform(new osg::Uniform("oe_splat_blend_end", 500.0f));
}

void
TextureSplattingLayer::update(osg::NodeVisitor& nv)
{
    // once the materials are loaded, install them and build the state set.
    if (_materials == nullptr && _materialsJob.isAvailable())
    {
        _materials = _materialsJob.release();
        buildStateSets();
    }
}

void
TextureSplattingLayer::buildStateSets()
{
    if (_materials != nullptr && getLifeMapLayer())
    {
        osg::StateSet* ss = getOrCreateStateSet();

        // Install the texture arena as a state attribute:
        ss->setAttribute(_materials->_arena);

        // Install the uniform holding constant texture scales
        ss->addUniform(_materials->_textureScales);

        // Install the texture splatting shader
        VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
        ProceduralShaders terrain_shaders;

        // Do the layer replacements BEFORE calling load. This way
        // the "oe_use_shared_layer" directive will be able to find the shared layer.
        terrain_shaders.replace(
            "OE_LIFEMAP_TEX",
            getLifeMapLayer()->getSharedTextureUniformName());

        terrain_shaders.replace(
            "OE_LIFEMAP_MAT",
            getLifeMapLayer()->getSharedTextureMatrixUniformName());

        terrain_shaders.load(
            vp, 
            terrain_shaders.TextureSplatting, 
            getReadOptions());

        // General purpose define indicating that this layer sets PBR values.
        ss->setDefine("OE_USE_PBR");
        Shaders shaders;
        shaders.load(vp, shaders.PBR);

        if (getBiomeLayer())
        {
            const auto& assets = getBiomeLayer()->getBiomeCatalog()->getAssets();
            ss->setDefine("OE_TEX_DIM_X", std::to_string(assets.getLifeMapMatrixWidth()));
            //ss->setDefine("OE_TEX_DIM_Y", std::to_string(assets.getLifeMapMatrixHeight()));
        }

        ss->setDefine(
            "OE_SPLAT_NUM_LEVELS",
            std::to_string(clamp(options().numLevels().get(), 1, 2)));

        if (options().useHexTiler() == true)
        {
            ss->setDefine("OE_SPLAT_HEX_TILER", "1");
        }
    }
}

void
TextureSplattingLayer::resizeGLObjectBuffers(unsigned maxSize)
{
    VisibleLayer::resizeGLObjectBuffers(maxSize);
}

void
TextureSplattingLayer::releaseGLObjects(osg::State* state) const
{
    VisibleLayer::releaseGLObjects(state);
}
