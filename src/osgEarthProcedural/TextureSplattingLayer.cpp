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
#include <cstdlib> // getenv

#define LC "[TextureSplattingLayer] " << getName() << ": "

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
    // OSG loader for reading an albedo texture and a heightmap texture
    // and merging them into a single RGBH texture
    struct RGBHPseudoLoader : public osgDB::ReaderWriter
    {
        RGBHPseudoLoader() {
            supportsExtension("oe_splat_rgbh", "RGB+H");
        }

        ReadResult readImage(const std::string& uri, const osgDB::Options* options) const override
        {
            std::string ext = osgDB::getLowerCaseFileExtension(uri);
            if (ext == "oe_splat_rgbh")
            {
                URI colorURI(
                    osgDB::getNameLessExtension(uri) + "_Color.jpg");

                osg::ref_ptr<osg::Image> color = colorURI.getImage(options);

                if (!color.valid())
                    return ReadResult::FILE_NOT_FOUND;

                URI heightURI(
                    osgDB::getNameLessExtension(uri) + "_Displacement.jpg");

                osg::ref_ptr<osg::Image> height = heightURI.getImage(options);

                osg::ref_ptr<osg::Image> rgbh = new osg::Image();
                rgbh->allocateImage(color->s(), color->t(), 1, GL_RGBA, GL_UNSIGNED_BYTE);

                ImageUtils::PixelReader readHeight(height.get());
                ImageUtils::PixelReader readColor(color.get());
                ImageUtils::PixelWriter writeRGBH(rgbh.get());
                osg::Vec4 temp, temp2;
                float minh = 1.0f, maxh = 0.0f;

                ImageUtils::ImageIterator iter(rgbh.get());
                iter.forEachPixel([&]()
                    {
                        readColor(temp, iter.s(), iter.t());
                        if (height.valid())
                        {
                            readHeight(temp2, iter.s(), iter.t());
                            temp.a() = temp2.r();
                        }
                        else
                        {
                            temp.a() = 0.0f;
                        }

                        minh = osg::minimum(minh, temp.a());
                        maxh = osg::maximum(maxh, temp.a());
                        writeRGBH(temp, iter.s(), iter.t());
                    });

                OE_INFO << heightURI.base() << ", MinH=" << minh << ", MaxH=" << maxh << std::endl;

                ImageUtils::compressImageInPlace(rgbh.get());

                return rgbh;
            }

            return ReadResult::FILE_NOT_HANDLED;
        }
    };

    #define DEFAULT_ROUGHNESS 0.75
    #define DEFAULT_AO 1.0

    // OSG load for reading normal, roughness, and AO textures and merging
    // them into a single encoded material texture.
    struct NNRAPseudoLoader : public osgDB::ReaderWriter
    {
        NNRAPseudoLoader() {
            supportsExtension("oe_splat_nnra", "Normal/Roughness/AO");
        }

        ReadResult readImage(const std::string& uri, const osgDB::Options* options) const override
        {
            std::string ext = osgDB::getLowerCaseFileExtension(uri);
            if (ext == "oe_splat_nnra")
            {
                URI normalsURI(
                    osgDB::getNameLessExtension(uri) + "_Normal.jpg");

                osg::ref_ptr<osg::Image> normals =
                    normalsURI.getImage(options);

                if (!normals.valid())
                    return ReadResult::FILE_NOT_FOUND;

                URI roughnessURI(
                    osgDB::getNameLessExtension(uri) + "_Roughness.jpg");

                osg::ref_ptr<osg::Image> roughness =
                    roughnessURI.getImage(options);

                URI aoURI(
                    osgDB::getNameLessExtension(uri) + "_AmbientOcclusion.jpg");

                osg::ref_ptr<osg::Image> ao =
                    aoURI.getImage(options);

                osg::ref_ptr<osg::Image> nnra = new osg::Image();
                nnra->allocateImage(normals->s(), normals->t(), 1, GL_RGBA, GL_UNSIGNED_BYTE);

                ImageUtils::PixelReader readNormals(normals.get());
                ImageUtils::PixelReader readAO(ao.get());
                ImageUtils::PixelReader readRoughness(roughness.get());
                ImageUtils::PixelWriter writeMat(nnra.get());

                osg::Vec3 normal3;
                osg::Vec4 normal;
                osg::Vec4 roughnessVal;
                osg::Vec4 aoVal;
                osg::Vec4 packed;

                ImageUtils::ImageIterator iter(nnra.get());
                iter.forEachPixel([&]()
                    {
                        readNormals(normal, iter.s(), iter.t());
                        normal3.set(normal.x()*2.0 - 1.0, normal.y()*2.0 - 1.0, normal.z()*2.0 - 1.0);
                        NormalMapGenerator::pack(normal3, packed);
                        if (roughness.valid())
                        {
                            readRoughness(roughnessVal, iter.s(), iter.t());
                            packed[2] = roughnessVal.r();
                        }
                        else packed[2] = DEFAULT_ROUGHNESS;

                        if (ao.valid())
                        {
                            readAO(aoVal, iter.s(), iter.t());
                            packed[3] = aoVal.r();
                        }
                        else packed[3] = DEFAULT_AO;

                        writeMat(packed, iter.s(), iter.t());
                    });

                return nnra;
            }

            return ReadResult::FILE_NOT_HANDLED;
        }
    };

    // Job for loading materials into an arena
    osg::ref_ptr<TextureArena>
    loadMaterials(const AssetCatalog* cat, Cancelable* progress)
    {
        osg::ref_ptr<TextureArena> arena = new TextureArena();

        for (const auto tex : cat->getTextures())
        {
            Texture* rgbh = new Texture();
            rgbh->_uri = URI(tex->uri()->full() + ".oe_splat_rgbh");
            arena->add(rgbh);

            Texture* nnra = new Texture();
            nnra->_uri = URI(tex->uri()->full() + ".oe_splat_nnra");
            arena->add(nnra);

            if (progress && progress->isCanceled())
                return nullptr;
        }

        return arena;
    }
}

REGISTER_OSGPLUGIN(oe_splat_rgbh, RGBHPseudoLoader);
REGISTER_OSGPLUGIN(oe_splat_nnra, NNRAPseudoLoader);

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
        const BiomeCatalog* cat = getBiomeLayer()->getBiomeCatalog();
        if (cat)
        {
            // Begin asynchronous loading of materials texture arena:
            osg::ref_ptr<const AssetCatalog> assets = cat->getAssets();
            if (assets.valid())
            {
                _materialsJob = MaterialsJob::dispatch(
                    [assets](Cancelable* progress)
                    {
                        return loadMaterials(assets.get(), progress);
                    }
                );
            }
        }
    }
    else
    {
        // without a biome layer we have no textures to splat with
        setStatus(Status::ResourceUnavailable, "No Biome data to splat");
        return;
    }

    engine->getResources()->reserveTextureImageUnitForLayer(_lifemap_res, this);
    if (_lifemap_res.valid() == false)
    {
        setStatus(Status::ResourceUnavailable, "Could not reserve a texture image unit");
        return;
    }
}

void
TextureSplattingLayer::update(osg::NodeVisitor& nv)
{
    if (!_arena.valid() && _materialsJob.isAvailable())
    {
        _arena = _materialsJob.release();
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
        shaders.load(vp, shaders.TextureSplatting2, getReadOptions());

        // Find the LifeMap layer and access its share
        ss->setDefine(
            "OE_LIFEMAP_TEX",
            getLifeMapLayer()->getSharedTextureUniformName());

        ss->setDefine(
            "OE_LIFEMAP_MAT",
            getLifeMapLayer()->getSharedTextureMatrixUniformName());
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
