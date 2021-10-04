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
#include "ProceduralShaders"
#include "NoiseTextureFactory"
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/VirtualProgram>
#include <osgUtil/CullVisitor>
#include <osg/BlendFunc>
#include <osg/Drawable>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <cstdlib> // getenv

#define LC0 "[TextureSplattingLayer] "
#define LC LC0 << getName() << ": "

#define RENDERPARAMS_LAYOUT_BINDING_INDEX 6

using namespace osgEarth::Procedural;

REGISTER_OSGEARTH_LAYER(splat, TextureSplattingLayer);
REGISTER_OSGEARTH_LAYER(splatimage, TextureSplattingLayer);
REGISTER_OSGEARTH_LAYER(splat_image, TextureSplattingLayer);
REGISTER_OSGEARTH_LAYER(splat_imagery, TextureSplattingLayer);
REGISTER_OSGEARTH_LAYER(proceduralimage, TextureSplattingLayer);
REGISTER_OSGEARTH_LAYER(procedural_image, TextureSplattingLayer);

//........................................................................

Config
TextureSplattingLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();
    //NOP
    return conf;
}

void
TextureSplattingLayer::Options::fromConfig(const Config& conf)
{
    //NOP
}

//........................................................................

#if 0
TextureSplattingLayer::Materials::Ptr
TextureSplattingLayer::loadMaterials(
    const AssetCatalog& cat, 
    Cancelable* progress) const
{
    Materials::Ptr result = Materials::Ptr(new Materials);

    result->_arena = new TextureArena();

    if (cat.getLifeMapMatrixHeight() * cat.getLifeMapMatrixWidth() !=
        cat.getLifeMapTextures().size())
    {
        OE_WARN << LC0 << "Configuration error: LifeMapTextures count does not match width*height"
            << std::endl;
        return nullptr;
    }

    for (auto& tex : cat.getLifeMapTextures())
    {
        auto t0 = std::chrono::steady_clock::now();

        result->_assets.push_back(&tex);

        Texture::Ptr rgbh = Texture::create();
        rgbh->_uri = URI(tex.uri()->full() + ".oe_splat_rgbh");
        result->_arena->add(rgbh);

        // protect the NNRA from compression, b/c it confuses the normal maps
        Texture::Ptr nnra = Texture::create();
        nnra->_uri = URI(tex.uri()->full() + ".oe_splat_nnra");
        nnra->_compress = false;
        result->_arena->add(nnra);

        auto t1 = std::chrono::steady_clock::now();

        OE_INFO << LC0 << "Loaded texture " << tex.uri()->base()
            << ", t=" << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "ms"
            << std::endl;

        if (progress && progress->isCanceled())
            return nullptr;
    }

    // initialize the size of the rendering parameters to be
    // equal to the number of texture assets. We will set the
    // values later.
    result->_renderParams.setNumElements(result->_assets.size());
    result->_renderParams.setBindingIndex(RENDERPARAMS_LAYOUT_BINDING_INDEX);

    return result;
}
#endif

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

#define NUM_LEVELS 1

void
TextureSplattingLayer::prepareForRendering(TerrainEngine* engine)
{
    VisibleLayer::prepareForRendering(engine);

    if (getLifeMapLayer() == nullptr)
    {
        // without a lifemap layer we can't do any splatting
        setStatus(Status::ResourceUnavailable, "No LifeMap data to splat");
        return;
    }

    // Since we're actually rendering, load the materials for splatting
    if (getBiomeLayer())
    {
        auto biome_cat = getBiomeLayer()->getBiomeCatalog();

        if (biome_cat && !biome_cat->getAssets().empty())
        {
            const AssetCatalog& assets = biome_cat->getAssets();

            GeoExtent ex[NUM_LEVELS];
            ex[0] = engine->getMap()->getProfile()->calculateExtent(14, 0, 0);
            ex[1] = engine->getMap()->getProfile()->calculateExtent(19, 0, 0);

            double tile_height_m[NUM_LEVELS];
            tile_height_m[0] = ex[0].height(Units::METERS);
            tile_height_m[1] = ex[1].height(Units::METERS);

            // Function to load all material textures.
            auto loadMaterials = [assets, tile_height_m](Cancelable* c) -> Materials::Ptr
            {
                Materials::Ptr result = Materials::Ptr(new Materials);

                // contains the textures and their bindless handles
                result->_arena = new TextureArena();

                // contains metadata about the textures (size etc.)
                result->_renderParams.setNumElements(assets.getLifeMapTextures().size() * NUM_LEVELS);

                if (assets.getLifeMapMatrixHeight() * assets.getLifeMapMatrixWidth() !=
                    assets.getLifeMapTextures().size())
                {
                    OE_WARN << LC0 << "Configuration error: LifeMapTextures count does not match width*height"
                        << std::endl;
                    return nullptr;
                }

                int ptr0 = 0;
                int ptr1 = assets.getLifeMapTextures().size();

                for (auto& tex : assets.getLifeMapTextures())
                {
                    auto t0 = std::chrono::steady_clock::now();

                    result->_assets.push_back(&tex);

                    Texture::Ptr rgbh = Texture::create();
                    rgbh->_uri = URI(tex.uri()->full() + ".oe_splat_rgbh");
                    result->_arena->add(rgbh);

                    // protect the NNRA from compression, b/c it confuses the normal maps
                    Texture::Ptr nnra = Texture::create();
                    nnra->_uri = URI(tex.uri()->full() + ".oe_splat_nnra");
                    nnra->_compress = false;
                    result->_arena->add(nnra);

                    auto t1 = std::chrono::steady_clock::now();

                    OE_INFO << LC0 << "Loaded texture " << tex.uri()->base()
                        << ", t=" << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "ms"
                        << std::endl;

                    if (c && c->isCanceled())
                        return nullptr;

                    // Set up the texture scaling:
                    RenderParams& params0 = result->_renderParams[ptr0++];
                    params0._scaleV = tex.size().isSet() ? tile_height_m[0] / tex.size()->as(Units::METERS) : 1.0f;
                    params0._scaleU = params0._scaleV;

                    if (NUM_LEVELS > 1)
                    {
                        RenderParams& params1 = result->_renderParams[ptr1++];
                        params1._scaleV = tex.size().isSet() ? tile_height_m[1] / tex.size()->as(Units::METERS) : 1.0f;
                        params1._scaleU = params1._scaleV;
                    }

                    //OE_INFO << LC0 << "   size=" << tex.size()->as(Units::METERS) << "m  scale=" << params._scaleU << std::endl;
                }

                result->_renderParams.dirty();

                // bind the buffer to a layout index in the shader
                result->_renderParams.setBindingIndex(RENDERPARAMS_LAYOUT_BINDING_INDEX);

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

        // install the LUT buffer for per-texture render parameters.
        ss->setAttribute(new StateAttributeAdapter(&_materials->_renderParams));

        // Install the texture splatting shader
        VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
        TerrainShaders shaders;
        shaders.load(vp, shaders.TextureSplatting, getReadOptions());

        // Find the LifeMap layer and access its share
        ss->setDefine(
            "OE_LIFEMAP_TEX",
            getLifeMapLayer()->getSharedTextureUniformName());

        ss->setDefine(
            "OE_LIFEMAP_MAT",
            getLifeMapLayer()->getSharedTextureMatrixUniformName());

        if (getBiomeLayer())
        {
            const auto& assets = getBiomeLayer()->getBiomeCatalog()->getAssets();
            ss->setDefine("OE_TEX_DIM_X", std::to_string(assets.getLifeMapMatrixWidth()));
            ss->setDefine("OE_TEX_DIM_Y", std::to_string(assets.getLifeMapMatrixHeight()));
        }
    }
}

void
TextureSplattingLayer::resizeGLObjectBuffers(unsigned maxSize)
{
    VisibleLayer::resizeGLObjectBuffers(maxSize);

    // no need to process _arena because it's in the StateSet
    if (_materials)
        _materials->_renderParams.resizeGLObjectBuffers(maxSize);
}

void
TextureSplattingLayer::releaseGLObjects(osg::State* state) const
{
    VisibleLayer::releaseGLObjects(state);

    // no need to process _arena because it's in the StateSet
    if (_materials)
        _materials->_renderParams.releaseGLObjects(state);
}
