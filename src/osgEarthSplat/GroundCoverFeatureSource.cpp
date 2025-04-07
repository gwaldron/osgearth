/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include "GroundCoverFeatureSource"
#include <osgEarth/ImageUtils>

using namespace osgEarth;
using namespace osgEarth::Splat;

#define LC "[GroundCoverFeatureSource] "

REGISTER_OSGEARTH_LAYER(groundcoverfeatures, GroundCoverFeatureSource);

//...................................................................

Config
GroundCoverFeatureSource::Options::getConfig() const
{
    Config conf = FeatureSource::Options::getConfig();
    groundCoverLayer().set(conf, "groundcover_layer");
    return conf;
}

void
GroundCoverFeatureSource::Options::fromConfig(const Config& conf)
{
    groundCoverLayer().get(conf, "groundcover_layer");
}

//...................................................................

void
GroundCoverFeatureSource::init()
{
    FeatureSource::init();
    _gen.setFactory(new TerrainTileModelFactory(
        TerrainOptions()));
}

Status
GroundCoverFeatureSource::openImplementation()
{
    Status parent = FeatureSource::openImplementation();
    if (parent.isError())
        return parent;

    Status s = options().groundCoverLayer().open(getReadOptions());
    if (s.isError())
        return s;

    // We ONLY support an embedded GroundCoverLayer here.
    if (getGroundCoverLayer())
    {
        _gen.setLayer(getGroundCoverLayer());

        FeatureProfile* fp = new FeatureProfile(Profile::create(Profile::GLOBAL_GEODETIC));
        fp->setFirstLevel(getGroundCoverLayer()->getLOD());
        fp->setMaxLevel(getGroundCoverLayer()->getLOD());
        setFeatureProfile(fp);
    }

    return Status::NoError;
}

Status
GroundCoverFeatureSource::closeImplementation()
{
    return Status::NoError;
}

void
GroundCoverFeatureSource::addedToMap(const Map* map)
{
    _gen.setMap(map);

    if (getGroundCoverLayer())
    {
        getGroundCoverLayer()->addedToMap(map);
    }
    
    if (_gen.getStatus().isError())
    {
        setStatus(_gen.getStatus());
        OE_WARN << LC << "Error: " << _gen.getStatus().toString() << std::endl;
    }
}

void
GroundCoverFeatureSource::removedFromMap(const Map* map)
{
    options().groundCoverLayer().removedFromMap(map);
}

void
GroundCoverFeatureSource::setGroundCoverLayer(GroundCoverLayer* layer)
{
    options().groundCoverLayer().setLayer(layer);
}

GroundCoverLayer*
GroundCoverFeatureSource::getGroundCoverLayer() const
{
    return options().groundCoverLayer().getLayer();
}

FeatureCursor*
GroundCoverFeatureSource::createFeatureCursorImplementation(const Query& query, ProgressCallback* progress) const
{
    if (getStatus().isError())
        return NULL;

    if (query.tileKey().isSet())
    {
        FeatureList output;
        _gen.getFeatures(query.tileKey().get(), output);
        return new FeatureListCursor(output);
    }
    else
    {
        OE_WARN << LC << "Illegal: GroundCoverFeatureSource requires a tilekey" << std::endl;
        return NULL;
    }
}