#include "AzureMaps"
#include <cstdlib>

using namespace osgEarth;

#undef LC
#define LC "[AzureMaps] "

//........................................................................

Config
AzureMapsImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    conf.set("subscription_key", subscriptionKey());
    conf.set("tileset_id", tilesetId());
    conf.set("map_tile_endpoint", mapTileEndpoint());
    conf.set("api_version", apiVersion());
    return conf;
}

void
AzureMapsImageLayer::Options::fromConfig(const Config& conf)
{
    conf.get("subscription_key", subscriptionKey());
    conf.get("tileset_id", tilesetId());
    conf.get("map_tile_endpoint", mapTileEndpoint());
    conf.get("api_version", apiVersion());
}


//........................................................................

REGISTER_OSGEARTH_LAYER(azuremapsimage, AzureMapsImageLayer);
REGISTER_OSGEARTH_LAYER(azureimage, AzureMapsImageLayer);


void
AzureMapsImageLayer::init()
{
    super::init();

    // disable caching by default due to TOS
    layerHints().cachePolicy() = CachePolicy::NO_CACHE;
}

AzureMapsImageLayer::~AzureMapsImageLayer()
{
    // nop
}

Status
AzureMapsImageLayer::openImplementation()
{
    Status parent = super::openImplementation();
    if (parent.isError())
        return parent;

    const char* key = ::getenv("OSGEARTH_AZURE_KEY");
    if (key)
    {
        options().subscriptionKey().unset();
        options().subscriptionKey().setDefault(key);
    }

    // Azure uses web mercator
    setProfile(Profile::create(Profile::SPHERICAL_MERCATOR));

    // authentication header
    _uricontext.addHeader("subscription-key", options().subscriptionKey().value());

    return Status::NoError;
}

Status
AzureMapsImageLayer::closeImplementation()
{
    return super::closeImplementation();
}

GeoImage
AzureMapsImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    std::stringstream query;
    query << "?api-version=" << options().apiVersion().value();
    query << "&tilesetId=" << options().tilesetId().value();
    query << "&zoom=" << key.getLOD() << "&x=" << key.getTileX() << "&y=" << key.getTileY();
    query << "&tileSize=" << std::to_string(getTileSize());

    // note: _uriContext holds our authentication headers
    URI imageURI(options().mapTileEndpoint()->full() + query.str(), _uricontext);

    auto fetch = imageURI.readImage(getReadOptions(), progress);
    if (fetch.failed())
    {
        if (fetch.code() == ReadResult::RESULT_UNAUTHORIZED)
        {
            OE_WARN << LC << "Azure Maps request failed due to authorization failure" << std::endl;
            OE_WARN << LC << fetch.errorDetail() << std::endl;
            setStatus(Status::ServiceUnavailable, fetch.errorDetail());
        }
        return GeoImage(fetch.errorDetail());
    }
    else
    {
        return GeoImage(fetch.releaseImage(), key.getExtent());
    }
}
