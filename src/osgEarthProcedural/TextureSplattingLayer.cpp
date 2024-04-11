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

#include <osgEarth/NoiseTextureFactory>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/TerrainResources>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Shaders>
#include <osgEarth/Capabilities>
#include <osgEarth/Registry>

#include <osgUtil/CullVisitor>
#include <osg/Drawable>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/FileNameUtils>

#include <cstdlib> // getenv

#define LC0 "[TextureSplattingLayer] "
#define LC LC0 << getName() << ": "

#define TEXTURE_ARENA_BINDING_POINT 5

using namespace osgEarth::Procedural;

REGISTER_OSGEARTH_LAYER(proceduralimage, TextureSplattingLayer);
//REGISTER_OSGEARTH_LAYER(procedural_image, TextureSplattingLayer);


//........................................................................

Config
TextureSplattingLayer::Options::getConfig() const
{
    Config conf = super::getConfig();
    conf.set("num_levels", numLevels());
    conf.set("use_hex_tiler", useHexTiler());
    conf.set("normalmap_power", normalMapPower());
    conf.set("lifemap_threshold", lifeMapMaskThreshold());
    conf.set("displacement_depth", displacementDepth());
    conf.set("max_texture_size", maxTextureSize());
    return conf;
}

void
TextureSplattingLayer::Options::fromConfig(const Config& conf)
{
    conf.get("num_levels", numLevels());
    conf.get("use_hex_tiler", useHexTiler());
    conf.get("normalmap_power", normalMapPower());
    conf.get("lifemap_threshold", lifeMapMaskThreshold());
    conf.get("displacement_depth", displacementDepth());
    conf.get("max_texture_size", maxTextureSize());
}

//........................................................................

BiomeLayer*
TextureSplattingLayer::getBiomeLayer() const
{
    return _biomeLayer.get();
}

LifeMapLayer*
TextureSplattingLayer::getLifeMapLayer() const
{
    return _lifeMapLayer.get();
}

void
TextureSplattingLayer::init()
{
    super::init();

    setRenderType(osgEarth::Layer::RENDERTYPE_TERRAIN_SURFACE);
}

void
TextureSplattingLayer::addedToMap(const Map* map)
{
    super::addedToMap(map);

    if (getBiomeLayer() == nullptr)
    {
        _biomeLayer = map->getLayer<BiomeLayer>();
    }

    if (getLifeMapLayer() == nullptr)
    {
        _lifeMapLayer = map->getLayer<LifeMapLayer>();
    }

    _mapProfile = map->getProfile();

    buildStateSets();
}

void
TextureSplattingLayer::removedFromMap(const Map* map)
{
    super::removedFromMap(map);
    _lifeMapLayer = nullptr;
    _biomeLayer = nullptr;
}

void
TextureSplattingLayer::prepareForRendering(TerrainEngine* engine)
{
    super::prepareForRendering(engine);

    if (Capabilities::get().supportsInt64() == false)
    {
        setStatus(Status::ResourceUnavailable, "GLSL int64 support required but not available");
        OE_WARN << LC << getStatus().message() << std::endl;
        return;
    }

    if (Capabilities::get().getGLSLVersion() < 4.6f || Capabilities::get().supportsNVGL() == false)
    {
        setStatus(Status::ResourceUnavailable, "GLSL 4.6+ required but not available");
        OE_WARN << LC << getStatus().message() << std::endl;
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
            ex.push_back(_mapProfile->calculateExtent(14, 0, 0));
            ex.push_back(_mapProfile->calculateExtent(19, 0, 0));

            std::vector<double> tile_height_m;
            tile_height_m.push_back(ex[0].height(Units::METERS));
            tile_height_m.push_back(ex[1].height(Units::METERS));

            osg::ref_ptr<const osgDB::Options> readOptions = getReadOptions();

            unsigned maxTextureSize = std::min(
                (int)options().maxTextureSize().get(),
                Registry::instance()->getMaxTextureSize());

            // Function to load all material textures.
            auto loadMaterials = [assets, tile_height_m, readOptions, maxTextureSize](Cancelable& c) -> Materials::Ptr
            {
                Materials::Ptr result = Materials::Ptr(new Materials);

                // contains the textures and their bindless handles
                result->_arena = new TextureArena();
                result->_arena->setName("TextureSplattingLayer");
                result->_arena->setBindingPoint(TEXTURE_ARENA_BINDING_POINT);
                result->_arena->setMaxTextureSize(maxTextureSize);

                // contains metadata about the textures
                result->_textureScales = new osg::Uniform();
                result->_textureScales->setName("oe_texScale");
                result->_textureScales->setType(osg::Uniform::FLOAT);
                result->_textureScales->setNumElements(
                    assets.getMaterials().size() * tile_height_m.size());

                int ptr0 = 0;
                int ptr1 = assets.getMaterials().size();

                for (auto& material : assets.getMaterials())
                {
                    auto t0 = std::chrono::steady_clock::now();

                    result->_assets.push_back(&material);

                    RGBH_NNRA_Loader::load(
                        material.uri()->full(),
                        result->_arena.get(),
                        readOptions.get());

                    auto t1 = std::chrono::steady_clock::now();

                    OE_INFO << LC0 << "Loaded material " << material.uri()->base()
                        << ", t=" << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "ms"
                        << std::endl;

                    if (c.canceled())
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
            _materialsJob = jobs::dispatch(loadMaterials);
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
    if (_materials == nullptr && _materialsJob.available())
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
        _materials->_arena->setMaxTextureSize(options().maxTextureSize().get());

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

        setUseHexTiler(options().useHexTiler().get());
        setNormalMapPower(options().normalMapPower().get());
        setLifeMapMaskThreshold(options().lifeMapMaskThreshold().get());
        setDisplacementDepth(options().displacementDepth().get());
    }
}

void
TextureSplattingLayer::setUseHexTiler(bool value)
{
    options().useHexTiler() = value;
    auto ss = getOrCreateStateSet();
    ss->removeDefine("OE_SPLAT_HEX_TILER");
    ss->setDefine("OE_SPLAT_HEX_TILER", value ? "1" : "0");
}

bool
TextureSplattingLayer::getUseHexTiler() const
{
    return options().useHexTiler().get();
}

void
TextureSplattingLayer::setNormalMapPower(float value)
{
    if (options().normalMapPower().get() != value)
        options().normalMapPower() = value;

    auto ss = getOrCreateStateSet();
    ss->getOrCreateUniform("oe_normal_power", osg::Uniform::FLOAT)->set(value);
}

float
TextureSplattingLayer::getNormalMapPower() const
{
    return options().normalMapPower().get();
}

void
TextureSplattingLayer::setLifeMapMaskThreshold(float value)
{
    if (getLifeMapMaskThreshold() != value)
        options().lifeMapMaskThreshold() = value;

    auto ss = getOrCreateStateSet();
    ss->getOrCreateUniform("oe_mask_alpha", osg::Uniform::FLOAT)->set(value);
}

float
TextureSplattingLayer::getLifeMapMaskThreshold() const
{
    return options().lifeMapMaskThreshold().get();
}

void
TextureSplattingLayer::setDisplacementDepth(float value)
{
    if (getDisplacementDepth() != value)
        options().displacementDepth() = value;

    auto ss = getOrCreateStateSet();
    ss->getOrCreateUniform("oe_displacement_depth", osg::Uniform::FLOAT)->set(value);
}

float
TextureSplattingLayer::getDisplacementDepth() const
{
    return options().displacementDepth().get();
}

void
TextureSplattingLayer::setMaxTextureSize(unsigned value)
{
    if (options().maxTextureSize().get() != value)
    {
        options().maxTextureSize() = value;
        if (_materials && _materials->_arena)
        {
            _materials->_arena->setMaxTextureSize(value);
        }
    }
}

unsigned
TextureSplattingLayer::getMaxTextureSize() const
{
    return options().maxTextureSize().value();
}

void
TextureSplattingLayer::resizeGLObjectBuffers(unsigned maxSize)
{
    super::resizeGLObjectBuffers(maxSize);

    if (_materials && _materials->_arena.valid())
        _materials->_arena->resizeGLObjectBuffers(maxSize);
}

void
TextureSplattingLayer::releaseGLObjects(osg::State* state) const
{
    super::releaseGLObjects(state);

    if (_materials && _materials->_arena.valid())
        _materials->_arena->releaseGLObjects(state);
}
