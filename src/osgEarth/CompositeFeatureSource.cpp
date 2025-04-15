/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "CompositeFeatureSource"

using namespace osgEarth;

REGISTER_OSGEARTH_LAYER(compositefeatures, CompositeFeatureSource);
REGISTER_OSGEARTH_LAYER(composite_features, CompositeFeatureSource);

Config
CompositeFeatureSource::Options::getConfig() const
{
    Config config = super::getConfig();
    if (!layers().empty())
    {
        auto& layers_conf = config.add("layers");
        for (auto& layer_ref : layers())
            layer_ref.set(layers_conf, "features");            
    }
    return config;
}

void
CompositeFeatureSource::Options::fromConfig(const Config& conf)
{
    auto& layers_conf = conf.child("layers").children();
    for (auto& layer_conf : layers_conf)
    {
        layers().emplace_back();
        layers().back().get(layer_conf, "features");
    }
}

void
CompositeFeatureSource::init()
{
    super::init();
}

Status
CompositeFeatureSource::openImplementation()
{
    auto parent = super::openImplementation();
    if (parent.isError()) return parent;

    return Status::NoError;
}

Status
CompositeFeatureSource::closeImplementation()
{
    return Status::NoError;
}

const GeoExtent&
CompositeFeatureSource::getExtent() const
{
    return _extent;
}

void
CompositeFeatureSource::addedToMap(const Map* map)
{
    super::addedToMap(map);

    for(auto& layer_ref : options().layers())
    {
        layer_ref.addedToMap(map);

        auto layer = layer_ref.getLayer();
        if (!layer)
        {
            setStatus(Status::ResourceUnavailable,
                "Component feature source layer is not available; \"" + layer_ref.externalLayerName().value() + "\" does not exist");
            return;
        }

        if (layer->getStatus().isError())
        {
            setStatus(layer->getStatus());
            return;
        }

        // every layer MUST have a feature profile
        auto* fp = layer->getFeatureProfile();
        if (!fp)
        {
            setStatus(Status::ConfigurationError,
                "Component feature sources must have a valid profile; \"" + layer->getName() + "\" does not");
            return;
        }

        // inherit the profile of the first component layer.
        if (!getFeatureProfile())
        {
            setFeatureProfile(fp);
        }

        // every layer MUST be tiled (for now).
        if (!fp->isTiled())
        {
            setStatus(Status::ConfigurationError,
                "Component feature sources must be tiled; \"" + layer->getName() + "\" is not");
            return;
        }

        // update the combined extent
        _extent.expandToInclude(layer->getExtent());
    }
}

void
CompositeFeatureSource::removedFromMap(const Map* map)
{
    for (auto& layer : options().layers())
    {
        layer.removedFromMap(map);
    }

    super::removedFromMap(map);
}

void
CompositeFeatureSource::dirty()
{
    for (auto& layer_ref : options().layers())
    {
        if (layer_ref.getLayer())
        {
            layer_ref.getLayer()->dirty();
        }
    }

    super::dirty();
}

FeatureCursor*
CompositeFeatureSource::createFeatureCursorImplementation(const Query& query, ProgressCallback* progress) const
{
    if (!isOpen())
        return nullptr;

    // Only TileKey sources for now.
    OE_SOFT_ASSERT_AND_RETURN(query.tileKey().isSet(), nullptr, "Only tiled queries are allowed");
    
    // Go through all the layers and try to create a valid cursor.
    // Once we get one return it.
    for (auto& layer_ref : options().layers())
    {
        auto cursor = layer_ref.getLayer()->createFeatureCursor(query, {}, nullptr, progress);

        if (cursor.valid() && cursor->hasMore())
        {
            return cursor.release();
        }
    }

    return nullptr;
}
