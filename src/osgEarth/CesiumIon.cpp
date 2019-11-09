/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2018 Pelican Mapping
 * http://osgearth.org
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include "CesiumIon"
#include <osgEarth/Registry>
#include <osgEarth/FileUtils>
#include <osgEarth/XmlUtils>
#include <osgEarth/JsonUtils>
#include <osgDB/FileUtils>

using namespace osgEarth;
using namespace osgEarth::CesiumIon;

#undef LC
#define LC "[CesiumIon] "

//............................................................................

Status
CesiumIon::Driver::open(const URI& server,
                        const std::string& format,
                        const std::string& assetId,
                        const std::string& token,
                        osg::ref_ptr<const Profile>& profile,
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

    _format = format;

    profile = Profile::create("spherical-mercator");

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
    buf2 << "*/*;access_token=" << _resourceToken;
    _acceptHeader = buf2.str();

    return STATUS_OK;
}

osgEarth::ReadResult
CesiumIon::Driver::read(const URI& server,
                        const TileKey& key,
                        ProgressCallback* progress,
                        const osgDB::Options* readOptions) const
{
    unsigned x, y;
    key.getTileXY(x, y);

    // Invert the y value
    unsigned cols = 0, rows = 0;
    key.getProfile()->getNumTiles(key.getLevelOfDetail(), cols, rows);
    y = rows - y - 1;

    std::string location = _resourceUrl;
    std::stringstream buf;
    buf << location;
    if (!endsWith(location, "/")) buf << "/";
    buf << key.getLevelOfDetail() << "/" << x << "/" << y << "." << _format;

    URIContext context = server.context();
    context.addHeader("accept", _acceptHeader);
    URI uri(buf.str(), context);
    return uri.getImage(readOptions, progress);
}

//........................................................................

Config
CesiumIonImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
            conf.set("server", _server);
            conf.set("asset_id", _assetId);
            conf.set("format", _format);
            conf.set("token", _token);
    return conf;
}

void
CesiumIonImageLayer::Options::fromConfig(const Config& conf)
{
    _server.init("https://api.cesium.com/");
    _format.init("png");

    conf.get("server", _server);
    conf.get("format", _format);
    conf.get("asset_id", _assetId);
    conf.get("token", _token);
}

Config
CesiumIonImageLayer::Options::getMetadata()
{
    return Config::readJSON( OE_MULTILINE(
        { "name" : "CesiumIon Service",
            "properties": [
            ]
        }
    ) );
}

//........................................................................

REGISTER_OSGEARTH_LAYER(cesiumionimage, CesiumIonImageLayer);

OE_LAYER_PROPERTY_IMPL(CesiumIonImageLayer, URI, Server, server);
OE_LAYER_PROPERTY_IMPL(CesiumIonImageLayer, std::string, Format, format);
OE_LAYER_PROPERTY_IMPL(CesiumIonImageLayer, std::string, AssetId, assetId);
OE_LAYER_PROPERTY_IMPL(CesiumIonImageLayer, std::string, Token, token);

void
CesiumIonImageLayer::init()
{
    ImageLayer::init();
    setTileSourceExpected(false);
}

Status
CesiumIonImageLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    osg::ref_ptr<const Profile> profile = getProfile();

    Status status = _driver.open(
        options().server().get(),
        options().format().get(),
        options().assetId().get(),
        options().token().get(),
        profile,
        getReadOptions());

    if (status.isOK() && profile.get() != getProfile())
    {
        setProfile(profile.get());
    }    

    return status;
}

GeoImage
CesiumIonImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    ReadResult r = _driver.read(
        options().server().get(),
        key,
        progress,
        getReadOptions());

    if (r.succeeded())
        return GeoImage(r.releaseImage(), key.getExtent());
    else
        return GeoImage(Status(r.errorDetail()));
}
