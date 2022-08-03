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
#include "VegetationFeatureGenerator"
#include "NoiseTextureFactory"
#include <osgEarth/ImageUtils>
#include <osgEarth/Math>
#include <osg/ComputeBoundsVisitor>

using namespace osgEarth;
using namespace osgEarth::Procedural;

#define LC "[VegetationFeatureGenerator] "

//...................................................................

namespace
{
    //// GLSL fract
    //inline float fract(float x)
    //{
    //    return fmodf(x, 1.0f);
    //}

    //// GLSL clamp
    //inline float clamp(float x, float m0, float m1)
    //{
    //    return osg::clampBetween(x, m0, m1);
    //}

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

VegetationFeatureGenerator::VegetationFeatureGenerator() :
    _status(Status::ConfigurationError),
    _sizeCache("OE.VegetationFeatureGenerator.modelSizeCache")
{
    //nop
}

void
VegetationFeatureGenerator::setMap(const Map* map)
{
    _map = map;
    initialize();
}

void
VegetationFeatureGenerator::setFactory(TerrainTileModelFactory* value)
{
    _factory = value;
    initialize();
}

void
VegetationFeatureGenerator::setLayer(VegetationLayer* layer)
{
    _veglayer = layer;
    initialize();
}

void
VegetationFeatureGenerator::addAssetPropertyName(const std::string& name)
{
    _propNames.push_back(name);
}

const Status&
VegetationFeatureGenerator::getStatus() const
{
    return _status;
}

void
VegetationFeatureGenerator::initialize()
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

    if (!_veglayer.valid())
    {
        _status.set(Status::ConfigurationError, "Missing VegetationLayer");
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
        _status.set(_lifemaplayer->getStatus().code(), Stringify() << "Opening life map layer: " << _lifemaplayer->getStatus().message());
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
        _status.set(_biomelayer->getStatus().code(), Stringify() << "Opening biome layer: " << _biomelayer->getStatus().message());
        return;
    }

    // open the groundcover layer
    if (_veglayer->open().isError())
    {
        _status.set(_veglayer->getStatus().code(), Stringify() << "Opening vegetation layer: " << _veglayer->getStatus().message());
        return;
    }

#if 0
    // make sure the lifemap intersects the LOD of the GC layer
    if (_veglayer->getLOD() < _lifemaplayer->getMinLevel() ||
        _veglayer->getLOD() > _lifemaplayer->getMaxLevel())
    {
        _status.set(
            Status::ResourceUnavailable,
            "GC Layer LOD is outside the min/max LOD of your LifeMap layer");
        return;
    }
#endif

    // open the active elevation layers
    ElevationLayerVector elevLayers;   
    _map->getLayers(elevLayers, [](const Layer* layer) { 
        return layer->getOpenAutomatically();
        });

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
    if (_veglayer.valid())
        _manifest.insert(_veglayer.get());
    for (auto& i : elevLayers)
        _manifest.insert(i.get());

    _status.set(Status::NoError);
}

Status
VegetationFeatureGenerator::getFeatures(const GeoExtent& extent, FeatureList& output) const
{
    if (!_map.valid() || _map->getProfile()==NULL)
        return Status(Status::ConfigurationError, "No map, or profile not set");

    if (!_veglayer.valid())
        return Status(Status::ConfigurationError, "No VegetationLayer");

    if (extent.isInvalid())
        return Status(Status::ConfigurationError, "Invalid extent");

    unsigned lod = _veglayer->options().groups()[AssetGroup::TREES].lod().get();

    std::vector<TileKey> keys;
    _map->getProfile()->getIntersectingTiles(extent, lod, keys);
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
VegetationFeatureGenerator::getFeatures(
    const TileKey& key, 
    FeatureList& output) const
{
    VegetationLayer::Options::Group& trees = _veglayer->options().groups()[AssetGroup::TREES];
    unsigned lod = trees.lod().get();
    if (key.getLOD() != lod)
        return Status(Status::ConfigurationError, "TileKey LOD does not match GroundCoverLayer LOD");

    std::vector<VegetationLayer::Placement> placements;

    bool ok = _veglayer->getAssetPlacements(
        key,
        AssetGroup::TREES,
        true,
        placements,
        nullptr);

    for (auto& p : placements)
    {
        // record the asset's position and properties as a point feature:
        Point* point = new Point();
        point->push_back(p.mapPoint());

        osg::ref_ptr<Feature> feature = new Feature(point, key.getExtent().getSRS());
        feature->set("elevation", p.mapPoint().z());

        float width = std::max(
            p.asset()->boundingBox().xMax() - p.asset()->boundingBox().xMin(),
            p.asset()->boundingBox().yMax() - p.asset()->boundingBox().yMin());

        float height =
            p.asset()->boundingBox().zMax() - p.asset()->boundingBox().zMin();

        feature->set("width", width * std::max(p.scale().x(), p.scale().y()));
        feature->set("height", height * p.scale().z());
        feature->set("rotation", p.rotation());

        feature->set("name", p.asset()->assetDef()->name());

        // Store any pass-thru properties
        if (!_propNames.empty())
        {
            const Config& assetConfig = p.asset()->assetDef()->getSourceConfig();

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

    return Status::NoError;
}
