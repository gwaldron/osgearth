/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "CesiumIon"
#include <osgEarth/Registry>
#include <osgEarth/JsonUtils>
#include <osgEarth/TDTiles>
#include <osgEarth/TMS>
#include <osgEarth/Bing>
#include <osgEarth/Notify>
#include <osgEarth/QuantizedMesh>

using namespace osgEarth;
using namespace osgEarth::Contrib::ThreeDTiles;

#undef LC
#define LC "[CesiumIon] "

//............................................................................

Status
CesiumIonResource::open(const URI& server,
                        const std::string& assetId,
                        const std::string& token,
                        const osgDB::Options* readOptions)
{
    if (assetId.empty())
    {
        return Status::Error(Status::ConfigurationError, "Fail: driver requires a valid \"asset_id\" property");
    }

    if (token.empty())
    {
        return Status::Error(Status::ConfigurationError, "Fail: driver requires a valid \"token\" property");
    }

    std::stringstream buf;
    buf << server.full();
    if (!endsWith(server.full(), "/")) buf << "/";
    buf << "v1/assets/" << assetId << "/endpoint?access_token=" << token;
    URI endpoint(buf.str());
    OE_DEBUG << "Getting endpoint " << endpoint.full() << std::endl;

    ReadResult r = URI(endpoint).readString(readOptions);
    if (r.failed())
        return Status::Error(Status::ConfigurationError, "Failed to get metadata from asset endpoint");

    Json::Value doc;
    Json::Reader reader;
    if (!reader.parse(r.getString(), doc))
    {
        return Status::Error(Status::ConfigurationError, "Failed to parse metadata from asset endpoint");
    }

    _resourceUrl = doc["url"].asString();
    _resourceToken = doc["accessToken"].asString();

    // Configure the accept header
    std::stringstream buf2;
    //buf2 << "*/*;access_token=" << _resourceToken;
    buf2 << "Bearer " << _resourceToken << std::endl;
    _acceptHeader = buf2.str();

    if (doc.isMember("externalType"))
    {
        _externalType = doc["externalType"].asString();
        if (doc.isMember("options"))
        {
            _externalOptions = doc["options"];
        }
    }

    return STATUS_OK;
}

//........................................................................

Config
CesiumIonImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
            conf.set("server", _server);
            conf.set("asset_id", _assetId);
            conf.set("token", _token);
    return conf;
}

void
CesiumIonImageLayer::Options::fromConfig(const Config& conf)
{
    _server.init("https://api.cesium.com/");
    conf.get("server", _server);
    conf.get("asset_id", _assetId);
    conf.get("token", _token);
}

Config
CesiumIonImageLayer::Options::getMetadata()
{
    return Config::readJSON(R"( {
        "name" : "CesiumIon Service",
        "properties" : [
        ]
    } )");
}

REGISTER_OSGEARTH_LAYER(cesiumionimage, CesiumIonImageLayer);

OE_LAYER_PROPERTY_IMPL(CesiumIonImageLayer, URI, Server, server);
OE_LAYER_PROPERTY_IMPL(CesiumIonImageLayer, std::string, AssetId, assetId);
OE_LAYER_PROPERTY_IMPL(CesiumIonImageLayer, std::string, Token, token);

void
CesiumIonImageLayer::init()
{
    ImageLayer::init();
}

Status
CesiumIonImageLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    const char* key = ::getenv("OSGEARTH_CESIUMION_KEY");
    if (key)
        _key = key;
    else
        _key = options().token().get();

    if (_key.empty())
    {
        return Status(Status::ConfigurationError, "CesiumIon API key is required");
    }

    CesiumIonResource ionResource;
    Status status = ionResource.open(
        options().server().get(),
        options().assetId().get(),
        _key,
        getReadOptions());

    if (status.isOK())
    {
        URIContext uriContext(ionResource._resourceUrl);
        uriContext.addHeader("authorization", ionResource._acceptHeader);
        URI tmsURI = URI("tilemapresource.xml", uriContext);


        if (ionResource._externalType.empty())
        {
            TMSImageLayer* tmsImageLayer = new TMSImageLayer();
            tmsImageLayer->setURL(tmsURI);
            _imageLayer = tmsImageLayer;
        }
        else
        {
            if (ionResource._externalType == "BING")
            {
                BingImageLayer *bingImageLayer = new BingImageLayer();
                bingImageLayer->setAPIKey(ionResource._externalOptions["key"].asString());
                bingImageLayer->setImagerySet(ionResource._externalOptions["mapStyle"].asString());
                _imageLayer = bingImageLayer;
            }
        }

        if (_imageLayer)
        {
            _imageLayer->setName(getName());
            status = _imageLayer->open();
            if (status.isError())
                return status;
            setProfile(_imageLayer->getProfile());
            DataExtentList dataExtents;
            _imageLayer->getDataExtents(dataExtents);
            setDataExtents(dataExtents);
        }
        else
        {
            return Status::Error("Unsupported Cesium Ion image layer");
        }
    }

    return status;
}

Status
CesiumIonImageLayer::closeImplementation()
{
    if (_imageLayer.valid())
    {
        _imageLayer->close();
        _imageLayer = NULL;
    }
    return ImageLayer::closeImplementation();
}

GeoImage
CesiumIonImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (_imageLayer)
    {
        return _imageLayer->createImage(key, progress);
    }
    return GeoImage(Status("Invalid image layer"));    
}

//........................................................................

Config
CesiumIon3DTilesLayer::Options::getConfig() const
{
    Config conf = ThreeDTilesLayer::Options::getConfig();
    conf.set("server", _server);
    conf.set("asset_id", _assetId);
    conf.set("token", _token);
    return conf;
}

void
CesiumIon3DTilesLayer::Options::fromConfig(const Config& conf)
{
    _server.init("https://api.cesium.com/");
    conf.get("server", _server);
    conf.get("asset_id", _assetId);
    conf.get("token", _token);
}

void
CesiumIon3DTilesLayer::init()
{
    ThreeDTilesLayer::init();
}

Status
CesiumIon3DTilesLayer::openImplementation()
{
    const char* key = ::getenv("OSGEARTH_CESIUMION_KEY");
    if (key)
        _key = key;
    else
        _key = options().token().get();

    if (_key.empty())
    {
        return Status(Status::ConfigurationError, "CesiumIon API key is required");
    }

    CesiumIonResource ionResource;
    Status status = ionResource.open(
        options().server().get(),
        options().assetId().get(),
        _key,
        getReadOptions());

    URI serverURI;
    if (status.isOK())
    {
        URIContext uriContext;
        uriContext.addHeader("authorization", ionResource._acceptHeader);
        serverURI = URI(ionResource._resourceUrl, uriContext);
    }

    Status parentStatus = VisibleLayer::openImplementation();
    if (parentStatus.isError())
        return parentStatus;

    ReadResult rr = serverURI.readString();
    if (rr.failed())
    {
        return Status(Status::ResourceUnavailable, Stringify() << "Error loading tileset: " << rr.errorDetail());
    }

    //OE_NOTICE << "Read tileset " << rr.getString() << std::endl;

    Tileset* tileset = Tileset::create(rr.getString(), serverURI.full());
    if (!tileset)
    {
        return Status(Status::GeneralError, "Bad tileset");
    }

    // Clone the read options
    osg::ref_ptr< osgDB::Options > readOptions = osgEarth::Registry::instance()->cloneOrCreateOptions(this->getReadOptions());

    _tilesetNode = new ThreeDTilesetNode(tileset, ionResource._acceptHeader, getSceneGraphCallbacks(), readOptions.get());
    _tilesetNode->setMaximumScreenSpaceError(*options().maximumScreenSpaceError());

    return STATUS_OK;
}

REGISTER_OSGEARTH_LAYER(cesiumion3dtiles, CesiumIon3DTilesLayer);
OE_LAYER_PROPERTY_IMPL(CesiumIon3DTilesLayer, URI, Server, server);
OE_LAYER_PROPERTY_IMPL(CesiumIon3DTilesLayer, std::string, AssetId, assetId);
OE_LAYER_PROPERTY_IMPL(CesiumIon3DTilesLayer, std::string, Token, token);





//........................................................................

Config
CesiumIonTerrainMeshLayer::Options::getConfig() const
{
    Config conf = TerrainMeshLayer::Options::getConfig();
    conf.set("server", _server);
    conf.set("asset_id", _assetId);
    conf.set("token", _token);
    return conf;
}

void
CesiumIonTerrainMeshLayer::Options::fromConfig(const Config& conf)
{
    _server.init("https://api.cesium.com/");
    conf.get("server", _server);
    conf.get("asset_id", _assetId);
    conf.get("token", _token);
}

Config
CesiumIonTerrainMeshLayer::Options::getMetadata()
{
    return Config::readJSON(R"(
        { "name" : "CesiumIon Service",
            "properties" : [
            ]
        })");    
}

REGISTER_OSGEARTH_LAYER(cesiumionterrainmesh, CesiumIonTerrainMeshLayer);

OE_LAYER_PROPERTY_IMPL(CesiumIonTerrainMeshLayer, URI, Server, server);
OE_LAYER_PROPERTY_IMPL(CesiumIonTerrainMeshLayer, std::string, AssetId, assetId);
OE_LAYER_PROPERTY_IMPL(CesiumIonTerrainMeshLayer, std::string, Token, token);

void
CesiumIonTerrainMeshLayer::init()
{
    TerrainMeshLayer::init();
}

Status
CesiumIonTerrainMeshLayer::openImplementation()
{
    Status parent = TerrainMeshLayer::openImplementation();
    if (parent.isError())
        return parent;

    const char* key = ::getenv("OSGEARTH_CESIUMION_KEY");
    if (key)
        _key = key;
    else
        _key = options().token().get();

    if (_key.empty())
    {
        return Status(Status::ConfigurationError, "CesiumIon API key is required");
    }

    CesiumIonResource ionResource;
    Status status = ionResource.open(
        options().server().get(),
        options().assetId().get(),
        _key,
        getReadOptions());

    if (status.isOK())
    {
        URIContext uriContext;
        uriContext.addHeader("authorization", ionResource._acceptHeader);
        _assetURI = URI(ionResource._resourceUrl, uriContext);
    }

    return status;
}

Status
CesiumIonTerrainMeshLayer::closeImplementation()
{
    return TerrainMeshLayer::closeImplementation();
}


TileMesh CesiumIonTerrainMeshLayer::createTileImplementation(
    const TileKey& key,
    ProgressCallback* progress) const
{
    unsigned x, y;
    key.getTileXY(x, y);
    unsigned int numRows, numCols;
    key.getProfile()->getNumTiles(key.getLevelOfDetail(), numCols, numRows);
    y = numRows - y - 1;

    URI tileURI(Stringify() << _assetURI.full() << key.getLevelOfDetail() << "/" << x << "/" << y << ".terrain", _assetURI.context());

    osgEarth::ReadResult result;
    for (unsigned int i = 0; i < 4; ++i)
    {
        result = tileURI.readString(getReadOptions(), progress);
        if (result.succeeded())
        {
            std::string data = result.getString();
            auto mesh = QuantizedMeshReader::readFromString(key, data);
            applyConstraints(key, mesh);
            return mesh;
        }
        else
        {
            //OE_NOTICE << "Request for " << tileURI.full() << " failed attempt=" << i << std::endl;
        }
    }

    TileMesh invalid;
    return invalid;
}
