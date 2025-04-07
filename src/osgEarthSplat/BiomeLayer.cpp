/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/
#include "BiomeLayer"

using namespace osgEarth;
using namespace osgEarth::Splat;

REGISTER_OSGEARTH_LAYER(biomes, BiomeLayer);

//...................................................................

Biome2ModelAsset::Biome2ModelAsset(const Config& conf)
{
    conf.get("name", name());
    conf.get("url", uri());
    conf.get("uri", uri());
}

Config
Biome2ModelAsset::getConfig() const
{
    Config conf("model");
    conf.set("name", name());
    conf.set("url", uri());
    return conf;
}

ReadResult
Biome2ModelAsset::getOrCreate(const osgDB::Options* dbo)
{
    if (!_node.valid())
    {
        _node = uri()->getNode(dbo);
    }

    return ReadResult(_node.get());
}

//...................................................................

Biome2::Biome2(const Config& conf)
{
    conf.get("name", name());
    
    ConfigSet models = conf.child("models").children("model");
    for (const auto& modelConf : models)
        modelAssets().push_back(new Biome2ModelAsset(modelConf));
}

Config
Biome2::getConfig() const
{
    Config conf("biome");
    conf.set("name", name());

    Config modelsConf("models");
    for (const auto& model : modelAssets())
        modelsConf.add(model->getConfig());
    if (!modelsConf.empty())
        conf.add(modelsConf);

    return conf;
}

//...................................................................

void
BiomeLayer::Options::fromConfig(const Config& conf)
{
    ConfigSet biomesConf = conf.child("biomes").children("biome");
    for (const auto& biomeConf : biomesConf)
        biomes().push_back(new Biome2(biomeConf));

    controlSet().get(conf, "control_features");
}

Config
BiomeLayer::Options::getConfig() const
{
    Config conf = Layer::Options::getConfig();

    for (const auto& biome : biomes())
        if (biome.valid())
            conf.add(biome->getConfig());

    controlSet().set(conf, "control_features");

    return conf;
}

//...................................................................

#undef LC
#define LC "[BiomeLayer] " << getName() << ": "

void
BiomeLayer::init()
{
    Layer::init();
}

Status
BiomeLayer::openImplementation()
{
    Status p = Layer::openImplementation();
    if (p.isError())
        return p;

    Status csStatus = options().controlSet().open(getReadOptions());
    if (csStatus.isError())
        return csStatus;

    osg::ref_ptr<FeatureCursor> cursor = getControlSet()->createFeatureCursor(Query(), nullptr);
    while (cursor.valid() && cursor->hasMore())
    {
        const Feature* f = cursor->nextFeature();
        if (f)
        {
            _index.emplace_back(ControlPoint());
            _index.back().x = f->getGeometry()->begin()->x();
            _index.back().y = f->getGeometry()->begin()->y();
            _index.back().biomeid = f->getInt("biomeid");
        }
    }
    OE_INFO << LC << "Loaded control set and found " << _index.size() << " features" << std::endl;

    return Status::OK();
}

Status
BiomeLayer::closeImplementation()
{
    _index.clear();

    options().controlSet().close();

    return Layer::closeImplementation();
}

FeatureSource*
BiomeLayer::getControlSet() const
{
    return options().controlSet().getLayer();
}

const Biome2*
BiomeLayer::getBiome(int id) const
{
    if (id >= 0 && id < options().biomes().size())
        return options().biomes()[id].get();
    else
        return nullptr;
}

void
BiomeLayer::getNearestBiomes(
    double x,
    double y,
    unsigned maxCount,
    std::set<SearchResult>& results) const
{
    results.clear();

    double farthest2 = 0.0;
    std::vector<SearchResult>::iterator farthest2_iter;

    for (const auto& cp : _index)
    {
        double range2 = (cp.x - x)*(cp.x - x) + (cp.y - y)*(cp.y - y);

        if (results.size() < maxCount)
        {
            SearchResult sr;
            sr.biomeid = cp.biomeid;
            sr.range2 = range2;
            results.insert(sr);
        }
        else
        {
            if (range2 < results.rbegin()->range2)
            {
                SearchResult sr;
                sr.biomeid = cp.biomeid;
                sr.range2 = range2;
                results.insert(sr);
                results.erase(std::prev(results.end()));
            }
        }
    }
}
