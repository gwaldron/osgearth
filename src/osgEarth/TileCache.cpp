/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <osgEarth/TileCache>
#include <osgEarth/Registry>
#include <osgEarth/FileUtils>
#include <osgEarth/XmlUtils>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgDB/FileUtils>

using namespace osgEarth;
using namespace osgEarth::TileCache;

#undef LC
#define LC "[TileCache] "

//........................................................................

namespace osgEarth { namespace TileCache
{
    std::string toString(double value, int precision = 7)
    {
        std::stringstream out;
        out << std::fixed << std::setprecision(precision) << value;
	    std::string outStr;
	    outStr = out.str();
        return outStr;
    }
} }

//...................................................................

void
TileCache::Options::writeTo(Config& conf) const
{
    conf.set("url", url());
    conf.set("layer", layer());
    conf.set("format", format());
}

void
TileCache::Options::readFrom(const Config& conf)
{
    conf.get("url", url());
    conf.get("layer", layer());
    conf.get("format", format());
}

//........................................................................


Status
TileCache::Driver::open(const URI& uri, const osgDB::Options* readOptions)
{
    // URI is mandatory.
    if (uri.empty())
    {
        return Status::Error( Status::ConfigurationError, "TMS driver requires a valid \"url\" property" );
    }

    return STATUS_OK;
}

osgEarth::ReadResult
TileCache::Driver::read(const URI& uri,
                        const TileKey& key,
                        const std::string& layer,
                        const std::string& format,
                        ProgressCallback* progress,
                        const osgDB::Options* readOptions) const
{
    unsigned int level, tile_x, tile_y;
    level = key.getLevelOfDetail();
    key.getTileXY(tile_x, tile_y);

    unsigned int numCols, numRows;
    key.getProfile()->getNumTiles(level, numCols, numRows);

    // need to invert the y-tile index
    tile_y = numRows - tile_y - 1;

    char buf[2048];
    sprintf(buf, "%s/%s/%02d/%03d/%03d/%03d/%03d/%03d/%03d.%s",
        uri.full().c_str(),
        layer.c_str(),
        level,
        (tile_x / 1000000),
        (tile_x / 1000) % 1000,
        (tile_x % 1000),
        (tile_y / 1000000),
        (tile_y / 1000) % 1000,
        (tile_y % 1000),
        format.c_str());


    std::string path(buf);
    return URI(path).readImage(readOptions, progress).releaseImage();
}

//...................................................................

Config
TileCacheImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    writeTo(conf);
    return conf;
}

void
TileCacheImageLayer::Options::fromConfig(const Config& conf)
{
    readFrom(conf);
}

//........................................................................

REGISTER_OSGEARTH_LAYER(tilecacheimage, TileCacheImageLayer);

OE_LAYER_PROPERTY_IMPL(TileCacheImageLayer, URI, URL, url);
OE_LAYER_PROPERTY_IMPL(TileCacheImageLayer, std::string, Layer, layer);
OE_LAYER_PROPERTY_IMPL(TileCacheImageLayer, std::string, Format, format);

void
TileCacheImageLayer::init()
{
    ImageLayer::init();
}

Status
TileCacheImageLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    if (!getProfile())
        setProfile(Profile::create("global-geodetic"));

    Status status = _driver.open(
        options().url().get(),
        getReadOptions());

    if (status.isError())
        return status;

    return Status::NoError;
}

GeoImage
TileCacheImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    ReadResult r = _driver.read(
        options().url().get(),
        key,
        options().layer().get(),
        options().format().get(),
        progress,
        getReadOptions());

    if (r.succeeded())
        return GeoImage(r.releaseImage(), key.getExtent());
    else
        return GeoImage(Status(r.errorDetail()));
}

//...................................................................

Config
TileCacheElevationLayer::Options::getConfig() const
{
    Config conf = ElevationLayer::Options::getConfig();
    writeTo(conf);
    return conf;
}

void
TileCacheElevationLayer::Options::fromConfig(const Config& conf)
{
    readFrom(conf);
}


//........................................................................

REGISTER_OSGEARTH_LAYER(tilecacheelevation, TileCacheElevationLayer);

OE_LAYER_PROPERTY_IMPL(TileCacheElevationLayer, URI, URL, url);
OE_LAYER_PROPERTY_IMPL(TileCacheElevationLayer, std::string, Layer, layer);
OE_LAYER_PROPERTY_IMPL(TileCacheElevationLayer, std::string, Format, format);

void
TileCacheElevationLayer::init()
{
    ElevationLayer::init();
}

Status
TileCacheElevationLayer::openImplementation()
{
    Status parent = ElevationLayer::openImplementation();
    if (parent.isError())
        return parent;

    // Create an image layer under the hood. TMS fetch is the same for image and
    // elevation; we just convert the resulting image to a heightfield
    _imageLayer = new TileCacheImageLayer(options());

    // Initialize and open the image layer
    _imageLayer->setReadOptions(getReadOptions());
    Status status = _imageLayer->open();
    if (status.isError())
        return status;

    setProfile(_imageLayer->getProfile());            

    return Status::NoError;
}

GeoHeightField
TileCacheElevationLayer::createHeightFieldImplementation(const TileKey& key, ProgressCallback* progress) const
{
    // Make an image, then convert it to a heightfield
    GeoImage image = _imageLayer->createImageImplementation(key, progress);
    if (image.valid())
    {
        ImageToHeightFieldConverter conv;
        osg::HeightField* hf = conv.convert( image.getImage() );
        return GeoHeightField(hf, key.getExtent());
    }
    else return GeoHeightField::INVALID;
}
