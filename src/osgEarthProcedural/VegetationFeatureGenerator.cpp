/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include "VegetationFeatureGenerator"
#include <osgEarth/NoiseTextureFactory>
#include <osgEarth/ImageUtils>
#include <osgEarth/Math>
#include <osg/ComputeBoundsVisitor>

using namespace osgEarth;
using namespace osgEarth::Procedural;

#define LC "[VegetationFeatureGenerator] "

VegetationFeatureGenerator::VegetationFeatureGenerator() :
    _status(Status::ConfigurationError)
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

    if (!_veglayer.valid())
    {
        _veglayer = _map->getLayer<VegetationLayer>();
    }

    if (!_veglayer.valid())
    {
        _status.set(Status::ConfigurationError, "Missing VegetationLayer");
        return;
    }

    // open the groundcover layer
    if (_veglayer->open().isError())
    {
        _status.set(_veglayer->getStatus().code(), Stringify() << "Opening vegetation layer: " << _veglayer->getStatus().message());
        return;
    }

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

    unsigned lod = _veglayer->options().group("trees").lod().get();

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
    VegetationLayer::Options::Group& trees = _veglayer->options().group("trees");
    unsigned lod = trees.lod().get();
    if (key.getLOD() != lod)
        return Status(Status::ConfigurationError, "TileKey LOD does not match Vegetation group LOD");

    std::vector<VegetationLayer::Placement> placements;

    bool ok = _veglayer->getAssetPlacements(
        key,
        "trees",
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
        feature->set("rotation", p.rotation() * 180.0f / M_PI);
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
