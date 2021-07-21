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
    return conf;
}

void
TextureSplattingLayer::Options::fromConfig(const Config& conf)
{
}

//........................................................................

namespace
{
    // Job for loading materials into an arena
    osg::ref_ptr<TextureArena>
    loadMaterials(const AssetCatalog& cat, Cancelable* progress)
    {
        osg::ref_ptr<TextureArena> arena = new TextureArena();

        if (cat.getLifeMapMatrixHeight() * cat.getLifeMapMatrixWidth() !=
            cat.getLifeMapTextures().size())
        {
            OE_WARN << LC0 << "Configuration error: LifeMapTextures count does not match width*height"
                << std::endl;
            return nullptr;
        }

        for (int i = 0; i < 2; ++i)
        {
            auto texList =
                i == 0 ? cat.getLifeMapTextures() :
                         cat.getSpecialTextures();

            for (auto& tex : texList)
            {
                auto t0 = std::chrono::steady_clock::now();

                Texture::Ptr rgbh = Texture::create();
                rgbh->_uri = URI(tex.uri()->full() + ".oe_splat_rgbh");
                arena->add(rgbh);

                // protect the NNRA from compression, b/c it confuses the normal maps
                Texture::Ptr nnra = Texture::create();
                nnra->_uri = URI(tex.uri()->full() + ".oe_splat_nnra");
                nnra->_compress = false;
                arena->add(nnra);

                auto t1 = std::chrono::steady_clock::now();

                OE_INFO << LC0 << "Loaded texture " << tex.uri()->base()
                    << ", t=" << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "ms"
                    << std::endl;

                if (progress && progress->isCanceled())
                    return nullptr;
            }
        }

        return arena;
    }
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
            _materials = Job().dispatch<osg::ref_ptr<TextureArena>>(
                [biome_cat](Cancelable* progress)
                {
                    return loadMaterials(biome_cat->getAssets(), progress);
                }
            );
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
    if (!_arena.valid() && _materials.isAvailable())
    {
        _arena = _materials.release();
        buildStateSets();
    }
}

void
TextureSplattingLayer::buildStateSets()
{
    if (_arena.valid() &&
        getLifeMapLayer())
    {
        osg::StateSet* ss = getOrCreateStateSet();

        // Install the texture arena as a state attribute:
        ss->setAttribute(_arena.get());

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
}

void
TextureSplattingLayer::releaseGLObjects(osg::State* state) const
{
    VisibleLayer::releaseGLObjects(state);

    // no need to process _arena because it's in the StateSet
}
