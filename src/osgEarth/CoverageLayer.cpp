#include "CoverageLayer"

using namespace osgEarth;


REGISTER_OSGEARTH_LAYER(coverage, CoverageLayer);

Config
CoverageLayer::SourceLayerOptions::getConfig() const
{
    Config conf;
    source().set(conf, "source");
    conf.set("mappings", mappings());
    return conf;
}

void
CoverageLayer::SourceLayerOptions::fromConfig(const Config& conf)
{
    source().get(conf, "source");
    conf.get("mappings", mappings());
}

Config
CoverageLayer::Options::getConfig() const
{
    Config conf = TileLayer::Options::getConfig();
    conf.set("presets", presets());
    if (_layers.empty() == false)
    {
        Config layersConf("layers");
        for (auto& layer : _layers)
        {
            layersConf.add(layer.getConfig());
        }
        conf.set(layersConf);
    }
    return conf;
}

void
CoverageLayer::Options::fromConfig(const Config& conf)
{
    conf.get("presets", presets());
    const ConfigSet& layers = conf.child("layers").children();
    for (auto& layer : layers)
    {
        SourceLayerOptions slo;
        slo.fromConfig(layer);
        _layers.emplace_back(std::move(slo));
    }
}

Status
CoverageLayer::openImplementation()
{
    Status parent = TileLayer::openImplementation();
    if (parent.isError())
        return parent;

    if (!getCacheSettings()->cachePolicy()->isCacheOnly())
    {
        for (auto& layer : options().layers())
        {
            layer.source().open(getReadOptions());

#if 0
            // Try to open it.
            Status cs = layer.source().open(getReadOptions());
            if (cs.isError())
                return cs;

            // Pull this layer's extents from the coverage layer.
            ImageLayer* imageLayer = dynamic_cast<ImageLayer*>(layer.source().getLayer());
            if (!imageLayer)
                return Status(Status::ResourceUnavailable, "Source must be an image layer");

            // GW: do we really need this? Probably not
            imageLayer->setUpL2Cache(9u);

            // Force the image source into coverage mode.
            imageLayer->options().coverage() = true;

            // inherit a profile from the first component layer, why not.
            if (getProfile() == nullptr)
            {
                setProfile(imageLayer->getProfile());
            }
#endif
        }
    }

    //if (getProfile() == nullptr)
    //{
    //    setProfile(Profile::create(Profile::GLOBAL_GEODETIC));
    //}

    return Status::NoError;
}

void
CoverageLayer::addedToMap(const Map* map)
{
    TileLayer::addedToMap(map);

    DataExtentList dataExtents;

    // in cache-only mode, we never want to access the component layer.
    bool cacheOnly = getCacheSettings()->cachePolicy()->isCacheOnly();
    if (!cacheOnly)
    {
        for (auto& layer : options().layers())
        {
            layer.source().addedToMap(map);

            auto* sourceLayer = layer.source().getLayer();

            // was the layer not found?
            if (sourceLayer == nullptr)
            {
                setStatus(Status::ResourceUnavailable, "Coverage component \"" + layer.source().externalLayerName().value() + "\" not found");
                return;
            }

            // was there a problem opening the layer?
            if (sourceLayer->getStatus().isError())
            {
                setStatus(sourceLayer->getStatus());
                return;
            }

            // Pull this layer's extents from the coverage layer.
            ImageLayer* imageLayer = dynamic_cast<ImageLayer*>(sourceLayer);
            if (imageLayer)
            {
                if (!cacheOnly)
                {
                    // GW: do we really need this? Probably not
                    imageLayer->setUpL2Cache(9u);

                    // Force the image source into coverage mode.
                    imageLayer->options().coverage() = true;

                    // inherit a profile from the first component layer, why not.
                    if (getProfile() == nullptr)
                    {
                        setProfile(imageLayer->getProfile());
                    }
                }

                // append the component's extents to the overall extents
                DataExtentList temp;
                imageLayer->getDataExtents(temp);
                dataExtents.insert(dataExtents.end(), temp.begin(), temp.end());
            }
            else
            {
                setStatus(Status::ConfigurationError, "Coverage component \"" + sourceLayer->getName() + "\" is not an image layer");
                return;
            }
        }
    }

    if (getProfile() == nullptr)
    {
        setProfile(Profile::create(Profile::GLOBAL_GEODETIC));
    }

    // We commented this out because it is causing coverage data that changes
    // (such as a decal layer) to malfunction. For now we just want it working;
    // later we can invesitgate and try to repair it. Omitting this will not
    // affect functionality but it might degrade load performance. -g
    //setDataExtents(dataExtents);
}

void
CoverageLayer::removedFromMap(const Map* map)
{
    TileLayer::removedFromMap(map);

    for (auto& layer : options().layers())
    {
        layer.source().removedFromMap(map);
    }
}

unsigned
CoverageLayer::getNumSourceLayers() const
{
    return options().layers().size();
}

ImageLayer*
CoverageLayer::getSourceLayerByName(const std::string& name) const
{
    for (auto& layer : options().layers())
    {
        if (layer.source().getLayer() && layer.source().getLayer()->getName() == name)
        {
            return layer.source().getLayer();
        }
    }
    return nullptr;
}

ImageLayer*
CoverageLayer::getSourceLayerByIndex(unsigned index) const
{
    if (index < options().layers().size())
    {
        return options().layers()[index].source().getLayer();
    }
    return nullptr;
}

void
CoverageLayer::addSourceLayer(ImageLayer* layer, const Config& mappingsConf)
{
    options().layers().push_back(SourceLayerOptions{});
    auto& entry = options().layers().back();
    entry.source().setLayer(layer);
    entry.mappings() = mappingsConf;
}
