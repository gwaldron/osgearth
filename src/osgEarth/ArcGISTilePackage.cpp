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
#include <osgEarth/ArcGISTilePackage>
#include <osgEarth/XmlUtils>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

using namespace osgEarth;
using namespace osgEarth::ArcGIS;

#undef LC
#define LC "[ArcGISTilePackage] "

//........................................................................

#define INDEX_HEADER_SIZE 16
#define INDEX_SIZE 5

namespace osgEarth { namespace ArcGIS
{
    unsigned int computeOffset(const std::vector<char>& buffer)
    {
        unsigned int sum = 0;
        for (unsigned int i = 0; i < buffer.size(); i++) {
            char v = buffer[i];
            sum += ((unsigned int)v & 0xff) * pow(2.0, 8.0 * i);
        }
        return sum;
    }

    unsigned int hexFromString(const std::string& input)
    {
        unsigned int result;
        std::stringstream ss;
        ss << std::hex << input;
        ss >> result;
        return result;
    }

    std::string toHex(unsigned int value)
    {
        std::stringstream ss;
        ss << std::hex << value;
        return ss.str();
    }

    std::string padLeft(std::string value, unsigned int length)
    {
        std::stringstream ss;
        if (value.size() < length)
        {
            for (unsigned int i = 0; i < (length - value.size()); i++)
            {
                ss << "0";
            }
            ss << value;
            return ss.str();
        }
        else
        {
            return value;
        }
    }

} }

BundleReader::BundleReader(const std::string& bundleFile, unsigned int bundleSize) :
    _bundleFile(bundleFile),
    _lod(0),
    _colOffset(0),
    _rowOffset(0),
    _bundleSize(bundleSize)
{
    init();
}

void BundleReader::init()
{
    std::string base = osgDB::getNameLessExtension(_bundleFile);
    _indexFile = base + ".bundlx";

    // Open the bundle
    _in.open(_bundleFile.c_str(), std::ofstream::binary);

    // Read the index
    readIndex(_indexFile, _index);

    std::string baseName = osgDB::getSimpleFileName(base);

    _rowOffset = hexFromString(baseName.substr(1, 4));
    _colOffset = hexFromString(baseName.substr(6, 4));

    std::string path = osgDB::getFilePath(_bundleFile);

    std::string levelDir = osgDB::getSimpleFileName(path);
    _lod = as<unsigned int>(levelDir.substr(1, 2), 0);
}

/**
* Reads the index of a bundle file.
*/
void BundleReader::readIndex(const std::string& filename, std::vector<int>& index)
{
    std::ifstream input(filename.c_str(), std::ifstream::binary);
    char header[INDEX_HEADER_SIZE];
    input.read(header, INDEX_HEADER_SIZE);
    while (input.good()) {
        std::vector<char> buffer;
        buffer.resize(5);
        if (input.read(&buffer[0], INDEX_SIZE))
        {
            int offset = computeOffset(buffer);
            index.push_back(offset);
        }
    }
}

osg::Image* BundleReader::readImage(const TileKey& key)
{
    // Figure out the index for the tilekey
    unsigned int row = key.getTileX() - _colOffset;
    unsigned int i = key.getTileY() - _rowOffset + (row * _bundleSize);
    return readImage(i);
}

osg::Image* BundleReader::readImage(unsigned int index)
{
    if (index < 0 || index >= _index.size()) return 0;

    _in.seekg(_index[index], std::ios::beg);
    std::vector<char> sizeBuffer;
    sizeBuffer.resize(4);
    _in.read(&sizeBuffer[0], 4);
    int size = computeOffset(sizeBuffer);
    if (size > 0)
    {
        std::string image;
        image.resize(size);
        _in.read(&image[0], size);
        std::stringstream ss(image);
        return ImageUtils::readStream(ss, 0);
    }

    return 0;
}

//........................................................................

Config
ArcGISTilePackageImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    conf.set("url", _url);
    return conf;
}

void
ArcGISTilePackageImageLayer::Options::fromConfig(const Config& conf)
{
    conf.get("url", _url);
}

//........................................................................

REGISTER_OSGEARTH_LAYER(arcgistilepackageimage, ArcGISTilePackageImageLayer);

OE_LAYER_PROPERTY_IMPL(ArcGISTilePackageImageLayer, URI, URL, url);

void
ArcGISTilePackageImageLayer::init()
{
    ImageLayer::init();
    _bundleSize = 128u;
    _extension = "png";
}

ArcGISTilePackageImageLayer::~ArcGISTilePackageImageLayer()
{
    //nop
}

Status
ArcGISTilePackageImageLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    readConf();

    // establish a profile if we don't already have one:
    if (!getProfile())
    {
        const Profile* profile = NULL;

        if (_profileConf.isSet())
        {
            profile = Profile::create(_profileConf.get());
        }
        else
        {
            // finally, fall back on mercator
            profile = Profile::create("spherical-mercator");
        }
        setProfile(profile);
    }

    return Status::NoError;
}

GeoImage
ArcGISTilePackageImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    // Try to figure out which bundle file the incoming tilekey is in.
    unsigned int numWide, numHigh;
    getProfile()->getNumTiles(key.getLevelOfDetail(), numWide, numHigh);

    std::stringstream buf;
    buf << options().url()->full() << "/_alllayers/";
    buf << "L" << padLeft(toString<unsigned int>(key.getLevelOfDetail()), 2) << "/";

    unsigned int colOffset = static_cast<unsigned int>(floor(static_cast<double>(key.getTileX() / _bundleSize) * _bundleSize));
    unsigned int rowOffset = static_cast<unsigned int>(floor(static_cast<double>(key.getTileY() / _bundleSize) * _bundleSize));

    buf << "R" << padLeft(toHex(rowOffset), 4) << "C" << padLeft(toHex(colOffset), 4);
    buf << ".bundle";

    std::string bundleFile = buf.str();
    if (osgDB::fileExists(bundleFile))
    {
        BundleReader reader(bundleFile, _bundleSize);
        osg::Image* result = reader.readImage(key);
        return GeoImage(result, key.getExtent());
    }

    return GeoImage::INVALID;
}

void
ArcGISTilePackageImageLayer::readConf()
{
    std::string confPath = options().url()->full() + "/conf.xml";

    osg::ref_ptr<XmlDocument> doc = XmlDocument::load(confPath);
    if (doc.valid())
    {
        Config conf = doc->getConfig();
        Config tileCacheInfo = conf.child("cacheinfo").child("tilecacheinfo");
        std::string wkt = tileCacheInfo.child("spatialreference").value("wkt");
        if (!wkt.empty())
        {
            SpatialReference* srs = SpatialReference::create(wkt);
            if (srs)
            {
                if (srs->isMercator())
                {
                    setProfile(Profile::create("spherical-mercator"));
                }
                else
                {
                    setProfile(Profile::create("global-geodetic"));
                }
            }
        }

        setTileSize(as<unsigned int>(tileCacheInfo.value("tilecols"), 256));

        std::string format = conf.child("cacheinfo").child("tileimageinfo").value("cachetileformat");
        if (format == "JPEG")
        {
            _extension = "jpg";
        }
        // All other cases will use png, this includes mixed mode.
        else
        {
            _extension = "png";
        }

        _bundleSize = as<unsigned int>(conf.child("cacheinfo").child("cachestorageinfo").value("packetsize"), 128);
    }
}