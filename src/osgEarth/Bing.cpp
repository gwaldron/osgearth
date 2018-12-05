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
#include <osgDB/FileUtils>
#include <cstdlib>

using namespace osgEarth;

#undef LC
#define LC "[Bing] "

//........................................................................

Config
BingImageLayerOptions::getConfig() const
{
    Config conf = ImageLayerOptions::getConfig();
    conf.set("key", _apiKey);
    conf.set("imagery_set", _imagerySet);
    conf.set("imagery_metadata_api_url", _imageryMetadataURL);
    return conf;
}

void
BingImageLayerOptions::fromConfig(const Config& conf)
{
    _imagerySet.init("Aerial");
    _imageryMetadataURL.init("http://dev.virtualearth.net/REST/v1/Imagery/Metadata");

    conf.get("key", _apiKey);
    conf.get("imagery_set", _imagerySet);
    conf.get("imagery_metadata_api_url", _imageryMetadataURL);
}


//........................................................................

REGISTER_OSGEARTH_LAYER(bingimage, BingImageLayer);

void
BingImageLayer::init()
{
    ImageLayer::init();

    setTileSourceExpected(false);
    _debugDirect = false;
    _tileURICache = new TileURICache(true, 1024u);
    
    if ( ::getenv("OSGEARTH_BING_DIRECT") )
        _debugDirect = true;
}

BingImageLayer::~BingImageLayer()
{
    delete _tileURICache;
}

const Status&
BingImageLayer::open()
{
    if (!options().apiKey().isSet())
    {
        return setStatus(Status::ConfigurationError, "Bing API key is required");
    }

    // Default cache policy to NO_CACHE for TOS compliance
    if (options().cachePolicy().isSet() == false)
    {
        options().cachePolicy() = CachePolicy::NO_CACHE;
    }
    
    // Bing maps profile is spherical mercator with 2x2 tiles are the root.
    setProfile(Profile::create(
        SpatialReference::get("spherical-mercator"),
            MERC_MINX, MERC_MINY, MERC_MAXX, MERC_MAXY,
            2u, 2u));

    return ImageLayer::open();
}

void
BingImageLayer::close()
{
    _tileURICache->clear();
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
            << "&key=" << options().apiKey().get();    // API key

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
                    OE_WARN << LC << "REST API request error!" << std::endl;

                    Config metadata;
                    std::string content = metadataResult.getString();
                    metadata.fromJSON(content);
                    ConfigSet errors = metadata.child("errorDetails").children();
                    for (ConfigSet::const_iterator i = errors.begin(); i != errors.end(); ++i)
                    {
                        OE_WARN << LC << "REST API: " << i->value() << std::endl;
                    }
                    return GeoImage::INVALID;
                }
                else
                {
                    OE_WARN << LC << "Request error: " << metadataResult.getResultCodeString() << std::endl;
                }
                return GeoImage::INVALID;
            }

            // decode it:
            Config metadata;
            if (!metadata.fromJSON(metadataResult.getString()))
            {
                OE_WARN << LC << "Error decoding REST API response" << std::endl;
                return GeoImage::INVALID;
            }

            // check the vintage field. If it's empty, that means we got a "no data" tile.
            Config* vintageEnd = metadata.find("vintageEnd");
            if (!vintageEnd || vintageEnd->value().empty())
            {
                OE_DEBUG << LC << "NO data image encountered." << std::endl;
                return GeoImage::INVALID;
            }

            // find the tile URI:
            Config* locationConf = metadata.find("imageUrl");
            if (!locationConf)
            {
                OE_WARN << LC << "REST API JSON parsing error (imageUrl not found)" << std::endl;
                return GeoImage::INVALID;
            }

            location = URI(locationConf->value());
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
        << "http://ecn.t"
        << _apiCount%4
        << ".tiles.virtualearth.net/tiles/h"
        << getQuadKey(key)
        << ".jpeg?g=1236";
}