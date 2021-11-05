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
    Config conf = Layer::Options::getConfig();
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

void
CoverageLayer::init()
{
    Layer::init();
}

Status
CoverageLayer::openImplementation()
{
    Status parent = Layer::openImplementation();
    if (parent.isError())
        return parent;

    for (auto& layer : options().layers())
    {
        // We never want to cache data from a coverage, because the "parent" layer
        // will be caching the entire result of a multi-coverage composite.
        layer.sourceEmbeddedOptions()->cachePolicy() = CachePolicy::NO_CACHE;

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
        imageLayer->setCoverage(true);
    }

    return Status::NoError;
}

void
CoverageLayer::addedToMap(const Map* map)
{
    Layer::addedToMap(map);

    for (auto& layer : options().layers())
    {
        layer.source().addedToMap(map);
    }
}

void
CoverageLayer::removedFromMap(const Map* map)
{
    Layer::removedFromMap(map);

    for (auto& layer : options().layers())
    {
        layer.source().removedFromMap(map);
    }
}

#if 0

//...................................................................

#undef LC
#define LC "[CompositeCoverage] "

REGISTER_OSGEARTH_LAYER(compositecoverage, CompositeCoverageLayer);

Config
CompositeCoverageLayer::Options::getConfig() const
{
    Config conf = CoverageLayer::Options::getConfig();
    if (layers().empty() == false)
    {
        Config layersConf("layers");
        for(auto& layer : layers())
        {
            layersConf.add(layer.getConfig());
        }
        conf.set(layersConf);
    }
    return conf;
}

void
CompositeCoverageLayer::Options::fromConfig(const Config& conf)
{
    const ConfigSet& layers = conf.child("layers").children();
    for(auto& layer : layers)
    {
        _layers.push_back(ConfigOptions(layer));
    }
}

//void
//CompositeCoverageLayer::addLayer(CoverageLayer* layer)
//{
//    if (isOpen())
//    {
//        OE_WARN << LC << "Illegal call to addLayer when layer is already open" << std::endl;
//    }
//    else if (layer)
//    {
//        _layers.push_back(layer);
//    }
//}

const CoverageLayerVector&
CompositeCoverageLayer::getLayers() const
{
    return _layers;
}

void
CompositeCoverageLayer::init()
{
    CoverageLayer::init();
}

void
CompositeCoverageLayer::addedToMap(const Map* map)
{
    for (auto& layer : _layers)
    {
        layer->addedToMap(map);
    }
}

void
CompositeCoverageLayer::removedFromMap(const Map* map)
{
    for (auto& layer : _layers)
    {
        layer->removedFromMap(map);
    }
}

Status
CompositeCoverageLayer::openImplementation()
{
    Status parent = CoverageLayer::openImplementation();
    if (parent.isError())
        return parent;

    // If we're in cache-only mode, do not attempt to open the component layers!
    if (getCacheSettings()->cachePolicy()->isCacheOnly())
        return Status::NoError;

    osg::ref_ptr<const Profile> profile;

    bool dataExtentsValid = true;

    // You may not call addLayers() and also put layers in the options.
    if (_layers.empty() == false && options().layers().empty() == false)
    {
        return Status(Status::ConfigurationError,
            "Illegal to add layers both by options and by API");
    }

    // If the user didn't call addLayer(), try to read them from the options.
    if (_layers.empty())
    {
        for(auto& i : options().layers())
        {
            osg::ref_ptr<Layer> newLayer = Layer::create(i);
            CoverageLayer* layer = dynamic_cast<CoverageLayer*>(newLayer.get());
            if (layer)
            {
                // Add to the list
                _layers.push_back(layer);
            }
            else
            {
                OE_WARN << LC << "This composite can only contains CoverageLayers; discarding a layer" << std::endl;
            }
        }
    }

    else
    {
        // the user added layers through the API, so store each layer's options
        // for serialization
        for (auto& layer : _layers)
        {
            options().layers().push_back(layer->getConfig());
        }
    }

    for(auto& layer : _layers)
    {
        layer->setReadOptions(getReadOptions());

        Status status = layer->open();

        if (status.isOK())
        {
            OE_DEBUG << LC << "...opened " << layer->getName() << " OK" << std::endl;
        }

        else
        {
            OE_WARN << LC << "...failed to open " << layer->getName() << ": " << status.message() << std::endl;
            if (getCacheSettings()->isCacheEnabled())
            {
                OE_WARN << LC << "...cache writes will be DISABLED for this layer" << std::endl;
                getCacheSettings()->integrateCachePolicy(CachePolicy(CachePolicy::USAGE_READ_ONLY));
            }
        }
    }

    return Status::NoError;
}

Status
CompositeCoverageLayer::closeImplementation()
{
    for (auto& layer : _layers)
    {
        layer->close();
    }

    return Status::OK();
}
#endif
