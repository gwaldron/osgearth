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

Status
CoverageLayer::openImplementation()
{
    Status parent = Layer::openImplementation();
    if (parent.isError())
        return parent;

    for (auto& layer : options().layers())
    {
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
