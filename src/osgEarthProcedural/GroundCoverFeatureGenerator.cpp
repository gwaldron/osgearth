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
#include "GroundCoverFeatureGenerator"
#include "GroundCoverLayer"
#include "NoiseTextureFactory"
#include <osgEarth/ImageUtils>
#include <osg/ComputeBoundsVisitor>

using namespace osgEarth;
using namespace osgEarth::Procedural;

#define LC "[GroundCoverFeatureGenerator] "

//...................................................................

namespace
{
    // GLSL fract
    inline float fract(float x)
    {
        return fmodf(x, 1.0f);
    }

    // GLSL clamp
    inline float clamp(float x, float m0, float m1)
    {
        return osg::clampBetween(x, m0, m1);
    }

    // Sample a texture with a scale/bias matrix
    inline void sample(osg::Vec4f& output, ImageUtils::PixelReader& texture, const osg::Matrixf& matrix, float u, float v)
    {
        u = clamp(u*matrix(0, 0) + matrix(3, 0), 0.0f, 1.0f);
        v = clamp(v*matrix(1, 1) + matrix(3, 1), 0.0f, 1.0f);
        return texture(output, u, v);
    }

    // Noise channels
    const int NOISE_SMOOTH = 0;
    const int NOISE_RANDOM = 1;
    const int NOISE_RANDOM_2 = 2;
    const int NOISE_CLUMPY = 3;
}

//...................................................................

GroundCoverFeatureGenerator::GroundCoverFeatureGenerator() :
    _status(Status::ConfigurationError),
    _sizeCache("OE.GroundCoverFeatureGenerator.modelSizeCache")
{
    //nop
}

void
GroundCoverFeatureGenerator::setMap(const Map* map)
{
    _map = map;
    initialize();
}

void
GroundCoverFeatureGenerator::setFactory(TerrainTileModelFactory* value)
{
    _factory = value;
    initialize();
}

void
GroundCoverFeatureGenerator::setLayer(GroundCoverLayer* layer)
{
    _gclayer = layer;
    initialize();
}

void
GroundCoverFeatureGenerator::addAssetPropertyName(const std::string& name)
{
    _propNames.push_back(name);
}

const Status&
GroundCoverFeatureGenerator::getStatus() const
{
    return _status;
}

void
GroundCoverFeatureGenerator::initialize()
{
    if (!_map.valid())
    {
        _status.set(Status::ConfigurationError, "Missing required Map");
        return;
    }

    if (_map->getProfile() == NULL)
    {
        _status.set(Status::ConfigurationError, "Map has no Profile set");
        return;
    }

    if (!_factory.valid())
    {
        _status.set(Status::ConfigurationError, "Missing required TerrainTileModelFactory");
        return;
    }

    if (!_gclayer.valid())
    {
        _status.set(Status::ConfigurationError, "Missing required GroundCoverLayer");
        return;
    }

    // find a lifemap layer
    _lifemaplayer = _map->getLayer<LifeMapLayer>();
    if (!_lifemaplayer.valid())
    {
        _status.set(Status::ConfigurationError, "Missing required LifeMapLayer");
        return;
    }
    if (_lifemaplayer->open().isError())
    {
        _status.set(_lifemaplayer->getStatus().code(), Stringify() << "Opening life map layer: " << _gclayer->getStatus().message());
        return;
    }

    _biomelayer = _map->getLayer<BiomeLayer>();
    if (!_biomelayer.valid())
    {
        _status.set(Status::ConfigurationError, "Missing required BiomeLayer");
        return;
    }
    if (_biomelayer->open().isError())
    {
        _status.set(_biomelayer->getStatus().code(), Stringify() << "Opening biome layer: " << _gclayer->getStatus().message());
        return;
    }

    // open the groundcover layer
    if (_gclayer->open().isError())
    {
        _status.set(_gclayer->getStatus().code(), Stringify() << "Opening groundcover layer: " << _gclayer->getStatus().message());
        return;
    }

    // make sure the lifemap intersects the LOD of the GC layer
    if (_gclayer->getLOD() < _lifemaplayer->getMinLevel() ||
        _gclayer->getLOD() > _lifemaplayer->getMaxLevel())
    {
        _status.set(
            Status::ResourceUnavailable,
            "GC Layer LOD is outside the min/max LOD of your LifeMap layer");
        return;
    }

    // open the elevation layers
    ElevationLayerVector elevLayers;   
    _map->getLayers(elevLayers, [](const Layer* layer) { return layer->getEnabled(); });
    if (elevLayers.empty() == false)
    {
        for(ElevationLayerVector::iterator i = elevLayers.begin();
            i != elevLayers.end();
            ++i)
        {
            Layer* layer = i->get();
            Status s = layer->open();
            if (s.isError())
            {
                _status.set(s.code(),
                    Stringify() << "Opening elevation layer \""
                    << layer->getName() << "\" : " << s.message());
                return;
            }
        }
    }

    // disable texture compression for layers we intend to sample on CPU
    if (_lifemaplayer.valid())
        _lifemaplayer->options().textureCompression() = "none";
    if (_biomelayer.valid())
        _biomelayer->options().textureCompression() = "none";

    // create noise texture
    NoiseTextureFactory noise;
    _noiseTexture = noise.create(256u, 4u);

    // layers we're going to request
    if (_lifemaplayer.valid())
        _manifest.insert(_lifemaplayer.get());
    if (_biomelayer.valid())
        _manifest.insert(_biomelayer.get());
    if (_gclayer.valid()) 
        _manifest.insert(_gclayer.get());
    for (auto& i : elevLayers)
        _manifest.insert(i.get());

    _status.set(Status::NoError);
}

Status
GroundCoverFeatureGenerator::getFeatures(const GeoExtent& extent, FeatureList& output) const
{
    if (!_map.valid() || _map->getProfile()==NULL)
        return Status(Status::ConfigurationError, "No map, or profile not set");

    if (!_gclayer.valid())
        return Status(Status::ConfigurationError, "No GroundCoverLayer");

    if (extent.isInvalid())
        return Status(Status::ConfigurationError, "Invalid extent");

    std::vector<TileKey> keys;
    _map->getProfile()->getIntersectingTiles(extent, _gclayer->getLOD(), keys);
    if (keys.empty())
        return Status(Status::AssertionFailure, "No keys intersect extent");
    
    for(auto& key : keys)
    {
        Status s = getFeatures(key, output);

        if (s.isError())
            return s;
    }

    return Status::NoError;
}

Status
GroundCoverFeatureGenerator::getFeatures(const TileKey& key, FeatureList& output) const
{
    if (key.getLOD() != _gclayer->getLOD())
        return Status(Status::ConfigurationError, "TileKey LOD does not match GroundCoverLayer LOD");

    // Populate the model, falling back on lower-LOD keys as necessary
    osg::ref_ptr<TerrainTileModel> model = _factory->createStandaloneTileModel(_map.get(), key, _manifest, NULL, NULL);
    if (!model.valid())
    {
        OE_INFO << LC << "null model for key " << key.str() << std::endl;
        return Status::NoError;
    }

    // noise sampler:
    ImageUtils::PixelReader sampleNoise;
    sampleNoise.setTexture(_noiseTexture.get());

    // lifemap texture:
    osg::Texture* lifemapTex = nullptr;
    osg::Matrixf lifemapMat;
    if (_lifemaplayer.valid())
    {
        lifemapTex = model->getTexture(_lifemaplayer->getUID());
        osg::RefMatrixf* matptr = model->getMatrix(_lifemaplayer->getUID());
        if (matptr) lifemapMat = *matptr;
    }
    ImageUtils::PixelReader sampleLifemap;
    sampleLifemap.setTexture(lifemapTex);
    osg::Vec4 lifemap_value;

    // biome map:
    osg::Texture* biomeTex = nullptr;
    osg::Matrixf biomeMat;
    if (_biomelayer.valid())
    {
        biomeTex = model->getTexture(_biomelayer->getUID());
        osg::RefMatrixf* matptr = model->getMatrix(_biomelayer->getUID());
        if (matptr) biomeMat = *matptr;
    }
    ImageUtils::PixelReader sampleBiome;
    sampleBiome.setTexture(biomeTex);
    sampleBiome.setBilinear(false);
    osg::Vec4 biome_value;

    // elevation
    osg::Texture* elevTex = NULL;
    osg::Matrix elevMat;
    if (model->elevationModel().valid())
    {
        elevTex = model->getElevationTexture();
        osg::RefMatrixf* r = model->getElevationTextureMatrix();
        if (r) elevMat = *r;
    }
    ImageUtils::PixelReader elevSampler;
    elevSampler.setTexture(elevTex);
    osg::Vec4 elev_value;
    // because in the shader oe_terrain_getElevation adjusts the sampling
    // with scale coefficients:
    elevSampler.setSampleAsTexture(false);

    // calculate instance count based on tile extents
    unsigned lod = _gclayer->getLOD();
    unsigned tx, ty;
    _map->getProfile()->getNumTiles(lod, tx, ty);
    GeoExtent e = TileKey(lod, tx / 2, ty / 2, _map->getProfile()).getExtent();
    GeoCircle c = e.computeBoundingGeoCircle();
    double tileWidth_m = 2.0 * c.getRadius() / 1.4142;
    float spacing_m = _gclayer->getSpacing().as(Units::METERS);
    unsigned numInstances1D = tileWidth_m / spacing_m;
    if (numInstances1D & 0x01) numInstances1D += 1;

    // from here on out, we are mimicing the GroundCover.VS.glsl shader logic.
    osg::Vec2ui numWorkgroups(numInstances1D, numInstances1D);
    osg::Vec2f numWorkgroupsF((float)numInstances1D, (float)numInstances1D);
    //unsigned numInstancesX = numInstances1D;
    //unsigned numInstancesY = numInstances1D;

    osg::Vec2f offset, tilec, shift;
    osg::Vec4f noise(0,0,0,0);

    osg::Vec2f halfSpacing(
        0.5f / numWorkgroupsF.x(),
        0.5f / numWorkgroupsF.y());

    for(unsigned y = 0; y < numWorkgroups.y(); ++y)
    {
        for (unsigned x = 0; x < numWorkgroups.x(); ++x)
        {
            offset.set((float)x, (float)y);

            tilec.set(
                halfSpacing.x() + offset.x() / numWorkgroupsF.x(),
                halfSpacing.y() + offset.y() / numWorkgroupsF.y());

            sampleNoise(noise, tilec.x(), tilec.y());

            shift.set(
                fract(noise[NOISE_RANDOM] * 1.5)*2.0f - 1.0f,
                fract(noise[NOISE_RANDOM_2] * 1.5)*2.0f - 1.0f);

            tilec.x() += shift.x() * halfSpacing.x();
            tilec.y() += shift.y() * halfSpacing.y();

            // look up the lifemap:
            sampleLifemap(lifemap_value, tilec.x(), tilec.y());
            float fill = lifemap_value[0]; // density
            float lush = lifemap_value[1];
            lush = noise[NOISE_CLUMPY] * lush;

            if (noise[NOISE_SMOOTH] > fill)
                continue;
            else
                noise[NOISE_SMOOTH] /= fill;

            // look up the biome:
            sampleBiome(biome_value, tilec.x(), tilec.y());
            int biome_id = (int)(biome_value.r()*255.0f);
            const Biome* biome = _biomelayer->getBiome(biome_id);
            if (biome == nullptr)
            {
                return Status(
                    Status::ConfigurationError,
                    Stringify() << "Biome " << biome_id << " not found in the catalog");
            }

            // and the model category to use:
            const std::string mcatName = _gclayer->getModelCategoryName();
            const ModelCategory* mcat = biome->getModelCategory(mcatName);
            if (!mcat)
            {
                return Status(
                    Status::ConfigurationError,
                    Stringify() << "Model category " << mcatName << " not found in the catalog");
            }

            // randomly select an asset from the category:
            int assetCount = mcat->members().size();
            if (assetCount == 0)
            {
                return Status(
                    Status::ConfigurationError,
                    Stringify() << "Model category " << mcatName << " is empty");
            }

            int pickIndex = clamp(int(floor(lush * float(assetCount))), 0, assetCount - 1);

            const ModelCategory::Member& member = mcat->members()[pickIndex];
            const ModelAsset* asset = member.asset;
            if (!asset)
            {
                return Status(
                    Status::ConfigurationError,
                    Stringify() << "Asset #" << pickIndex << " in " << mcatName << " was null");
            }

            // clamp the asset to the elevation data:
            float z = 0.0;
            if (elevTex)
            {
                sample(elev_value, elevSampler, elevMat, tilec.x(), tilec.y());
                //if (fabs(elev.r()) > 30000.0f)
                //{
                //    OE_WARN << "That's a problem.. elevation value is " << elev.r() << std::endl;
                //    osg::Vec4f val;
                //    sample(val, elevSampler, elevMat, tilec.x(), tilec.y());
                //    OE_WARN << "val.r() = " << val.r() << std::endl;
                //    
                //}
                if (elev_value.r() != NO_DATA_VALUE)
                {
                    z = elev_value.r();
                }
            }

            // record the asset's position and properties as a point feature:
            Point* point = new Point();

            point->push_back(osg::Vec3d(
                key.getExtent().xMin() + tilec.x()*key.getExtent().width(),
                key.getExtent().yMin() + tilec.y()*key.getExtent().height(),
                0.0));

            osg::ref_ptr<Feature> feature = new Feature(point, key.getExtent().getSRS());
            feature->set("elevation", z);

            float width = 0.0f, height = 0.0f;
            if (asset->modelURI().isSet() &&
                (!asset->width().isSet() || !asset->height().isSet()))
            {
                ScopedMutexLock lock(_sizeCache);
                osg::BoundingBoxf* bbox;

                auto i = _sizeCache.find(asset);
                if (i == _sizeCache.end())
                {
                    bbox = &_sizeCache[asset];
                    osg::ref_ptr<osg::Node> node = asset->modelURI()->getNode();
                    if (node.valid())
                    {
                        osg::ComputeBoundsVisitor cbv;
                        node->accept(cbv);
                        *bbox = cbv.getBoundingBox();
                    }
                    else
                    {
                        bbox->set(0, 0, 0, 0, 0, 0);
                    }
                }
                else
                {
                    bbox = &i->second;
                }

                width = asset->width().isSet()
                    ? asset->width().get()
                    : std::max(bbox->xMax() - bbox->xMin(), bbox->yMax() - bbox->yMin());

                height = asset->height().isSet()
                    ? asset->height().get()
                    : bbox->zMax() - bbox->zMin();
            }

            // Resolve the symbol so we can add attributes   
            float sizeScale = 1.0 + asset->sizeVariation().get() * (noise[NOISE_RANDOM_2] * 2.0 - 1.0);
            feature->set("width", width * sizeScale);
            feature->set("height", height * sizeScale);

            // We can recover rotation but it's unnecessary at this time -gw

            // Store any pass-thru properties
            if (!_propNames.empty())
            {
                // TODO: slow. optimize by precomputing elsewhere.
                const Config& assetConfig = asset->getSourceConfig();

                for (std::vector<std::string>::const_iterator i = _propNames.begin();
                    i != _propNames.end();
                    ++i)
                {
                    std::string value = assetConfig.value(*i);
                    if (!value.empty())
                    {
                        feature->set(*i, value);
                    }
                }
            }

            output.push_back(feature.get());
        }
    }

    return Status::NoError;
}
