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
#include <osgEarth/Bing>
#include <osgEarth/Registry>
#include <osgEarth/FileUtils>
#include <osgEarth/XmlUtils>
#include <osgEarth/JsonUtils>
#include <osgEarth/Progress>
#include <osgDB/FileUtils>
#include <osgDB/ReadFile>
#include <cstdlib>

using namespace osgEarth;

#undef LC
#define LC "[Bing] "

//........................................................................

Config
BingImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    conf.set("key", _apiKey);
    conf.set("imagery_set", _imagerySet);
    conf.set("imagery_metadata_api_url", _imageryMetadataURL);
    return conf;
}

void
BingImageLayer::Options::fromConfig(const Config& conf)
{
    _imagerySet.init("Aerial");
    _imageryMetadataURL.init("https://dev.virtualearth.net/REST/v1/Imagery/Metadata");

    conf.get("key", _apiKey);
    conf.get("imagery_set", _imagerySet);
    conf.get("imagery_metadata_api_url", _imageryMetadataURL);
}


//........................................................................

REGISTER_OSGEARTH_LAYER(bingimage, BingImageLayer);

OE_LAYER_PROPERTY_IMPL(BingImageLayer, std::string, APIKey, apiKey);
OE_LAYER_PROPERTY_IMPL(BingImageLayer, std::string, ImagerySet, imagerySet);
OE_LAYER_PROPERTY_IMPL(BingImageLayer, URI, ImageryMetadataURL, imageryMetadataURL);

void
BingImageLayer::init()
{
    ImageLayer::init();

    _debugDirect = false;
    _tileURICache = new TileURICache(true, 1024u);
    
    if ( ::getenv("OSGEARTH_BING_DIRECT") )
        _debugDirect = true;

    // disable caching by default due to TOS
    layerHints().cachePolicy() = CachePolicy::NO_CACHE;

    const char* key = ::getenv("OSGEARTH_BING_KEY");
    if (key)
        _key = key;
    else
        _key = options().apiKey().get();
}

BingImageLayer::~BingImageLayer()
{
    delete _tileURICache;
}

Status
BingImageLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    if (_key.empty())
    {
        return Status(Status::ConfigurationError, "Bing API key is required");
    }
    
    // Bing maps profile is spherical mercator with 2x2 tiles are the root.
    setProfile(Profile::create(
        SpatialReference::get("spherical-mercator"),
            MERC_MINX, MERC_MINY, MERC_MAXX, MERC_MAXY,
            2u, 2u));

    return Status::NoError;
}

Status
BingImageLayer::closeImplementation()
{
    _tileURICache->clear();
    return ImageLayer::closeImplementation();
}

GeoImage
BingImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    osg::ref_ptr<osg::Image> image;

    if (_debugDirect)
    {
        ++_apiCount;
        image = URI(getDirectURI(key)).getImage(_readOptions.get(), progress);
    }

    else
    {
        // center point of the tile (will be in spherical mercator)
        double x, y;
        key.getExtent().getCentroid(x, y);

        // transform it to lat/long:
        GeoPoint geo;

        GeoPoint(getProfile()->getSRS(), x, y).transform(
            getProfile()->getSRS()->getGeographicSRS(),
            geo);

        // contact the REST API. Docs are here:
        // http://msdn.microsoft.com/en-us/library/ff701716.aspx

        // construct the request URI:
        std::string request = Stringify()
            << std::setprecision(12)
            << options().imageryMetadataURL()->full()  // base REST API
            << "/" << options().imagerySet().get()     // imagery set to use
            << "/" << geo.y() << "," << geo.x()        // center point in lat/long
            << "?zl=" << key.getLOD() + 1              // zoom level
            << "&o=json"                               // response format
            << "&key=" << _key;                        // API key

        // check the URI cache.
        URI                  location;
        TileURICache::Record rec;

        if (_tileURICache->get(request, rec))
        {
            location = URI(rec.value());
        }
        else
        {
            unsigned c = ++_apiCount;
            if (c % 25 == 0)
                OE_DEBUG << LC << "API calls = " << c << std::endl;

            // fetch it:
            ReadResult metadataResult = URI(request).readString(_readOptions.get(), progress);
            if (metadataResult.failed())
            {
                // check for a REST error:
                if (metadataResult.code() == ReadResult::RESULT_SERVER_ERROR)
                {
                    return Status("Bing REST API error");
                }
                else
                {
                    OE_DEBUG << LC << "Request error: " << metadataResult.getResultCodeString() << std::endl;
                    if (progress)
                        progress->cancel();
                }
                return GeoImage::INVALID;
            }

            Json::Reader reader;

            Json::Value metadata;
            if (!reader.parse(metadataResult.getString(), metadata))
            {
                return Status("Bing: Error decoding REST API response");
            }

            // check the vintage field. If it's empty, that means we got a "no data" tile.
            const Json::Value& vintageEnd = Json::Path(".resourceSets[0].resources[0].vintageEnd").resolve(metadata);
            if (vintageEnd.empty())
            {
                return Status("Bing: NO data image encountered");
            }

            // find the tile URI:
            const Json::Value& imageUrl = Json::Path(".resourceSets[0].resources[0].imageUrl").resolve(metadata);
            if (imageUrl.empty())
            {
                return Status("Bing: REST API JSON parsing error (imageUrl not found)");
            }

            location = URI(imageUrl.asString());
            _tileURICache->insert(request, location.full());
        }

        // request the actual tile
        //OE_INFO << "key = " << key.str() << ", URL = " << location->value() << std::endl;
        image = osgDB::readRefImageFile(location.full());
    }

    return GeoImage(image.get(), key.getExtent());
}

std::string
BingImageLayer::getQuadKey(const TileKey& key) const
{
    unsigned int tile_x, tile_y;
    key.getTileXY(tile_x, tile_y);
    unsigned int lod = key.getLevelOfDetail();

    std::stringstream ss;
    for (unsigned i = (int)lod + 1; i > 0; i--)
    {
        char digit = '0';
        unsigned mask = 1 << (i - 1);
        if ((tile_x & mask) != 0)
        {
            digit++;
        }
        if ((tile_y & mask) != 0)
        {
            digit += 2;
        }
        ss << digit;
    }
    return ss.str();
}

std::string
BingImageLayer::getDirectURI(const TileKey& key) const
{
    return Stringify()
        << "https://ecn.t"
        << _apiCount%4
        << ".tiles.virtualearth.net/tiles/h"
        << getQuadKey(key)
        << ".jpeg?g=1236";
}

//........................................................................

Config
BingElevationLayer::Options::getConfig() const
{
    Config conf = ElevationLayer::Options::getConfig();
    conf.set("key", _apiKey);
    conf.set("url", _url);
    return conf;
}

void
BingElevationLayer::Options::fromConfig(const Config& conf)
{
    conf.get("key", _apiKey);
    conf.get("url", _url);
}

//........................................................................

REGISTER_OSGEARTH_LAYER(bingelevation, BingElevationLayer);

OE_LAYER_PROPERTY_IMPL(BingElevationLayer, std::string, APIKey, apiKey);
OE_LAYER_PROPERTY_IMPL(BingElevationLayer, URI, URL, url);

void
BingElevationLayer::init()
{
    ElevationLayer::init();

    // disable caching by default due to TOS
    layerHints().cachePolicy() = CachePolicy::NO_CACHE;

    _globalGeodetic = Profile::create("global-geodetic");

    const char* key = ::getenv("OSGEARTH_BING_KEY");
    if (key)
        _key = key;
    else
        _key = options().apiKey().get();
}

BingElevationLayer::~BingElevationLayer()
{
    //nop
}

Status
BingElevationLayer::openImplementation()
{
    Status parent = ElevationLayer::openImplementation();
    if (parent.isError())
        return parent;

    if (_key.empty())
    {
        return Status(Status::ConfigurationError, "Bing API key is required");
    }

    // Bing maps profile is spherical mercator with 2x2 tiles are the root.
    setProfile(Profile::create(
        SpatialReference::get("spherical-mercator"),
        MERC_MINX, MERC_MINY, MERC_MAXX, MERC_MAXY,
        2u, 2u));

    return Status::NoError;
}

GeoHeightField
BingElevationLayer::createHeightFieldImplementation(const TileKey& key, ProgressCallback* progress) const
{
    osg::ref_ptr<osg::HeightField> hf;

    // Get the extent in lat/long:
    GeoExtent latLonExtent = _globalGeodetic->clampAndTransformExtent(key.getExtent());

    // contact the REST API. Docs are here:
    // http://dev.virtualearth.net/REST/v1/Elevation/{Bounds}?bounds={boundingBox}&rows={rows}&cols={cols}&heights={heights}&key={BingMapsAPIKey}

    // max return data is 1024 samples (32x32)
    int tileSize = 32;

    // construct the request URI:
    std::string request = Stringify()
        << std::setprecision(12)
        << "https://dev.virtualearth.net/REST/v1/Elevation/Bounds"
        << "?bounds=" << latLonExtent.yMin() << "," << latLonExtent.xMin() << "," << latLonExtent.yMax() << "," << latLonExtent.xMax()
        << "&rows=" << tileSize
        << "&cols=" << tileSize
        << "&heights=ellipsoid"
        << "&key=" << _key;

    // fetch it:
    ReadResult result = URI(request).readString(_readOptions.get(), progress);
    if (result.failed())
    {
        // check for a REST error:
        if (result.code() == ReadResult::RESULT_SERVER_ERROR)
        {
            return Status(result.errorDetail());
        }
        else
        {
            OE_DEBUG << LC << "Request error: " << result.getResultCodeString() << std::endl;
            if (progress)
                progress->cancel();

            return GeoHeightField::INVALID;
        }
    }

    Json::Value response;

    Json::Reader reader;
    if (!reader.parse(result.getString(), response, false))
    {
        return Status("Bing response: Invalid JSON in response");
    }

    const Json::Value& elevations = Json::Path(".resourceSets[0].resources[0].elevations").resolve(response);
    if (elevations.isArray() == false)
    {
        return Status("Bing response: JSON path did not resolve");
    }

    if (elevations.size() != tileSize * tileSize)
    {
        return Status("Bing response: Insufficient data");
    }

    hf = new osg::HeightField();
    hf->allocate(tileSize, tileSize);
    int ptr = 0;
    for (Json::ValueConstIterator i = elevations.begin(); i != elevations.end(); ++i)
    {
        float elevation = (*i).asDouble();
        (*hf->getFloatArray())[ptr++] = elevation;
    }

    return GeoHeightField(hf.release(), key.getExtent());
}
