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
#include <osgEarth/ArcGISTilePackage>
#include <osgEarth/XmlUtils>
#include <osgEarth/MVT>
#include <osgEarth/Registry>
#include <osgEarth/JsonUtils>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgEarth/ImageToHeightFieldConverter>

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

//........................................................................

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

osg::Image* BundleReader::readImage(const TileKey& key, const osgDB::ReaderWriter* rw)
{
    // Figure out the index for the tilekey
    unsigned int row = key.getTileX() - _colOffset;
    unsigned int i = key.getTileY() - _rowOffset + (row * _bundleSize);
    return readImage(i, rw);
}

osg::Image* BundleReader::readImage(unsigned int index, const osgDB::ReaderWriter* rw)
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

        osg::Image* result = ImageUtils::readStream(ss, 0);
        if (!result && rw)
        {
            result = rw->readImage(ss, 0).takeImage();
        }
        return result;
    }

    return 0;
}

void BundleReader::readFeatures(const TileKey& key, FeatureList& features)
{
#ifdef OSGEARTH_HAVE_MVT
    // Figure out the index for the tilekey
    unsigned int row = key.getTileX() - _colOffset;
    unsigned int index = key.getTileY() - _rowOffset + (row * _bundleSize);

    if (index < 0 || index >= _index.size()) return;

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

        osgEarth::MVT::readTile(ss, key, features);
    }
#else
    OE_WARN << LC << "osgEarth is not built with MVT/PBF support" << std::endl;
#endif
}

//........................................................................
const unsigned long long M = pow(2, 40);

BundleReader2::BundleReader2(const std::string& bundleFile, unsigned int bundleSize) :
    _bundleFile(bundleFile),
    _lod(0),
    _colOffset(0),
    _rowOffset(0),
    _bundleSize(bundleSize)
{
    init();
}

void BundleReader2::init()
{
    std::string base = osgDB::getNameLessExtension(_bundleFile);

    // Open the bundle
    _in.open(_bundleFile.c_str(), std::ofstream::binary);

    // Read the index
    readIndex(_index);

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
void BundleReader2::readIndex(std::vector<unsigned long long>& index)
{
    // Go past the bundle header
    _in.seekg(64);

    const unsigned int numEntries = 128 * 128;
    index = std::vector<unsigned long long>(numEntries, 0);

    //unsigned long long indexBuffer[numEntries];
    _in.read((char*)&index[0], numEntries * sizeof(unsigned long long));
    unsigned int numGoodEntries = 0;
    for (unsigned int i = 0; i < numEntries; i++)
    {
        unsigned long long tile_index = index[i];
        unsigned long long tileOffset = tile_index % (unsigned long long)M;
        unsigned long long tileSize = floor((unsigned long long)tile_index / M);
        if (tileSize > 0)
        {
            numGoodEntries++;
        }
    }
}

osg::Image* BundleReader2::readImage(const TileKey& key, const osgDB::ReaderWriter* rw)
{
    unsigned int col = key.getTileX() - _colOffset;
    unsigned int row = key.getTileY() - _rowOffset;
    unsigned int i = _bundleSize * row + col;
    return readImage(i, rw);
}

osg::Image* BundleReader2::readImage(unsigned int index, const osgDB::ReaderWriter* rw)
{
    if (index < 0 || index >= _index.size()) return 0;

    unsigned long long tile_index = _index[index];
    unsigned long long tileOffset = tile_index % (unsigned long long)M;
    unsigned long long tileSize = floor((unsigned long long)tile_index / M);

    _in.seekg(tileOffset, std::ios::beg);
    if (tileSize > 0)
    {
        std::string imageData;
        imageData.resize(tileSize);
        _in.read(&imageData[0], tileSize);
        std::stringstream ss(imageData);

        osg::Image* result = ImageUtils::readStream(ss, 0);
        if (!result && rw)
        {
            result = rw->readImage(ss, 0).takeImage();
        }
        return result;
    }

    return 0;
}

void BundleReader2::readFeatures(const TileKey& key, FeatureList& features)
{
#ifdef OSGEARTH_HAVE_MVT
    unsigned int col = key.getTileX() - _colOffset;
    unsigned int row = key.getTileY() - _rowOffset;
    unsigned int index = _bundleSize * row + col;

    if (index < 0 || index >= _index.size()) return;

    unsigned long long tile_index = _index[index];
    unsigned long long tileOffset = tile_index % (unsigned long long)M;
    unsigned long long tileSize = floor((unsigned long long)tile_index / M);

    _in.seekg(tileOffset, std::ios::beg);
    if (tileSize > 0)
    {
        std::string imageData;
        imageData.resize(tileSize);
        _in.read(&imageData[0], tileSize);
        std::stringstream ss(imageData);
        osgEarth::MVT::readTile(ss, key, features);
    }
#else
    OE_WARN << LC << "osgEarth is not built with MVT/PBF support" << std::endl;
#endif
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
    _storageFormat = STORAGE_FORMAT_COMPACT;
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
            profile = Profile::create(Profile::SPHERICAL_MERCATOR);
        }
        setProfile(profile);
    }

    return Status::NoError;
}

GeoImage
ArcGISTilePackageImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
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
        osg::Image* result = NULL;
        if (_storageFormat == STORAGE_FORMAT_COMPACT)
        {
            BundleReader reader(bundleFile, _bundleSize);
            result = reader.readImage(key, _rw.get());
        }
        if (_storageFormat == STORAGE_FORMAT_COMPACTV2)
        {
            BundleReader2 reader(bundleFile, _bundleSize);
            result = reader.readImage(key, _rw.get());
        }
        if (result)
        {
            return GeoImage(result, key.getExtent());
        }
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
                    setProfile(Profile::create(Profile::SPHERICAL_MERCATOR));
                }
                else
                {
                    setProfile(Profile::create(Profile::GLOBAL_GEODETIC));
                }
            }
        }

        setTileSize(as<unsigned int>(tileCacheInfo.value("tilecols"), 256));

        std::string format = conf.child("cacheinfo").child("tileimageinfo").value("cachetileformat");
        _rw = osgDB::Registry::instance()->getReaderWriterForExtension(osgEarth::toLower(format));

        _bundleSize = as<unsigned int>(conf.child("cacheinfo").child("cachestorageinfo").value("packetsize"), 128);
        std::string storageFormat = conf.child("cacheinfo").child("cachestorageinfo").value("storageformat");
        if (ciEquals(storageFormat, "esriMapCacheStorageModeCompact"))
        {
            _storageFormat = STORAGE_FORMAT_COMPACT;
        }
        else if (ciEquals(storageFormat, "esriMapCacheStorageModeCompactV2"))
        {
            _storageFormat = STORAGE_FORMAT_COMPACTV2;
        }


    }
}


Config
ArcGISTilePackageElevationLayer::Options::getConfig() const
{
    Config conf = ElevationLayer::Options::getConfig();
    conf.set("url", _url);
    return conf;
}

void
ArcGISTilePackageElevationLayer::Options::fromConfig(const Config& conf)
{
    conf.get("url", _url);
}

REGISTER_OSGEARTH_LAYER(arcgistilepackageelevation, ArcGISTilePackageElevationLayer);
OE_LAYER_PROPERTY_IMPL(ArcGISTilePackageElevationLayer, URI, URL, url);

void
ArcGISTilePackageElevationLayer::init()
{
    ElevationLayer::init();
    _bundleSize = 128u;
    _storageFormat = STORAGE_FORMAT_COMPACT;
}

ArcGISTilePackageElevationLayer::~ArcGISTilePackageElevationLayer()
{
    //nop
}

Status
ArcGISTilePackageElevationLayer::openImplementation()
{
    Status parent = ElevationLayer::openImplementation();
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
            profile = Profile::create(Profile::SPHERICAL_MERCATOR);
        }
        setProfile(profile);
    }

    return Status::NoError;
}

GeoHeightField
ArcGISTilePackageElevationLayer::createHeightFieldImplementation(const TileKey& key, ProgressCallback* progress) const
{
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
        osg::Image* result = NULL;
        if (_storageFormat == STORAGE_FORMAT_COMPACT)
        {
            BundleReader reader(bundleFile, _bundleSize);
            result = reader.readImage(key, _rw.get());
        }
        if (_storageFormat == STORAGE_FORMAT_COMPACTV2)
        {
            BundleReader2 reader(bundleFile, _bundleSize);
            result = reader.readImage(key, _rw.get());
        }
        if (result)
        {
            ImageToHeightFieldConverter conv;
            osg::HeightField* hf = conv.convert(result);
            return GeoHeightField(hf, key.getExtent());
        }
    }
    return GeoHeightField::INVALID;
}

void
ArcGISTilePackageElevationLayer::readConf()
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
                    setProfile(Profile::create(Profile::SPHERICAL_MERCATOR));
                }
                else
                {
                    setProfile(Profile::create(Profile::GLOBAL_GEODETIC));
                }
            }
        }

        setTileSize(as<unsigned int>(tileCacheInfo.value("tilecols"), 256));

        std::string format = conf.child("cacheinfo").child("tileimageinfo").value("cachetileformat");
        _rw = osgDB::Registry::instance()->getReaderWriterForExtension(osgEarth::toLower(format));

        _bundleSize = as<unsigned int>(conf.child("cacheinfo").child("cachestorageinfo").value("packetsize"), 128);
        std::string storageFormat = conf.child("cacheinfo").child("cachestorageinfo").value("storageformat");
        if (ciEquals(storageFormat, "esriMapCacheStorageModeCompact"))
        {
            _storageFormat = STORAGE_FORMAT_COMPACT;
        }
        else if (ciEquals(storageFormat, "esriMapCacheStorageModeCompactV2"))
        {
            _storageFormat = STORAGE_FORMAT_COMPACTV2;
        }
    }
}


//........................................................................

REGISTER_OSGEARTH_LAYER(vtpkfeatures, VTPKFeatureSource);

OE_LAYER_PROPERTY_IMPL(VTPKFeatureSource, URI, URL, url);

Status
VTPKFeatureSource::openImplementation()
{
    Status parent = FeatureSource::openImplementation();
    if (parent.isError())
        return parent;

    Json::Reader reader;
    Json::Value root(Json::objectValue);

    std::stringstream buf;
    buf << options().url()->full() << "/p12/root.json";

    std::string rootPath = buf.str();

    std::ifstream in(rootPath);
    if (!reader.parse(in, root, false))
        return Status::Error(Stringify() << "Failed to parse " << rootPath);

    _bundleSize = root["resourceInfo"]["cacheInfo"]["storageInfo"]["packetSize"].asUInt();
    std::string storageFormat = root["resourceInfo"]["cacheInfo"]["storageInfo"]["storageFormat"].asString();

    if (ciEquals(storageFormat, "compact"))
    {
        _storageFormat = STORAGE_FORMAT_COMPACT;
    }
    else if (ciEquals(storageFormat, "compactV2"))
    {
        _storageFormat = STORAGE_FORMAT_COMPACTV2;
    }

    const osgEarth::Profile* profile = nullptr;
    std::string wkid = root["tileInfo"]["spatialReference"]["latestWkid"].asString();
    if (wkid == "3857")
    {
        profile = Profile::create(Profile::SPHERICAL_MERCATOR);
    }
    else if (wkid == "4326")
    {
        profile = Profile::create(Profile::GLOBAL_GEODETIC);
    }
    if (!profile)
    {
        // Assume it's mercator
        profile = Profile::create(Profile::SPHERICAL_MERCATOR);
    }

    FeatureProfile* featureProfile = new FeatureProfile(profile);
    unsigned int minLevel, maxLevel;
    computeMinMaxLevel(minLevel, maxLevel);
    featureProfile->setFirstLevel(minLevel);
    featureProfile->setMaxLevel(maxLevel);
    featureProfile->setTilingProfile(profile);
    featureProfile->geoInterp() = osgEarth::GEOINTERP_GREAT_CIRCLE;
    setFeatureProfile(featureProfile);


    return Status::NoError;
}

void
VTPKFeatureSource::init()
{
    FeatureSource::init();

    _storageFormat = STORAGE_FORMAT_COMPACTV2;
    _bundleSize = 128u;
}

void
VTPKFeatureSource::computeMinMaxLevel(unsigned int &min, unsigned int &max)
{
    // Scan the p12/tile directory to see what directories exist to determine the min and max levels
    min = UINT_MAX;
    max = 0;
    std::stringstream buf;
    buf << options().url()->full() << "/p12/tile/";
    auto contents = osgDB::getDirectoryContents(buf.str());
    for (auto &c : contents)
    {
        if (c.compare(".") == 0 || c.compare("..") == 0)
            continue;

        unsigned int level = as<int>(c.substr(1), 0);
        if (level < min) min = level;
        if (level > max) max = level;
    }
}

FeatureCursor*
VTPKFeatureSource::createFeatureCursorImplementation(const Query& query, ProgressCallback* progress)
{
    if (!query.tileKey().isSet())
    {
        OE_WARN << LC << "No tile key in query; no features will be returned\n";
        return 0L;
    }

    TileKey key = *query.tileKey();
    unsigned int z = key.getLevelOfDetail();
    unsigned int x = key.getTileX();
    unsigned int y = key.getTileY();
    // Arcgis uses a 360x360 profile for geodetic, which is kind of like global geodetic but only the top half.
    // We offset the zoom level by one to get the correct tile key level.
    // However, it is important not to modify the existing key as it contains the correct extents for the requested tile, which is
    // used by readFeatures/osgEarth::MVT::readTile to properly scale the encoded features to the proper geospatial coordinates.
    if (key.getProfile()->getSRS()->isGeodetic())
    {
        z += 1;
    }

    std::stringstream buf;
    buf << options().url()->full() << "/p12/tile/";
    buf << "L" << padLeft(toString<unsigned int>(z), 2) << "/";

    unsigned int colOffset = static_cast<unsigned int>(floor(static_cast<double>(x / _bundleSize) * _bundleSize));
    unsigned int rowOffset = static_cast<unsigned int>(floor(static_cast<double>(y / _bundleSize) * _bundleSize));

    buf << "R" << padLeft(toHex(rowOffset), 4) << "C" << padLeft(toHex(colOffset), 4);
    buf << ".bundle";

    FeatureList features;
    std::string bundleFile = buf.str();
    if (osgDB::fileExists(bundleFile))
    {
        if (_storageFormat == STORAGE_FORMAT_COMPACT)
        {
            BundleReader reader(bundleFile, _bundleSize);
            reader.readFeatures(key, features);
        }
        if (_storageFormat == STORAGE_FORMAT_COMPACTV2)
        {
            BundleReader2 reader(bundleFile, _bundleSize);
            reader.readFeatures(key, features);
        }
    }

    if (!features.empty())
    {
        return new FeatureListCursor(features);
    }
    return nullptr;
}

Config
VTPKFeatureSource::Options::getConfig() const
{
    Config conf = FeatureSource::Options::getConfig();
    conf.set("url", url());
    return conf;
}

void
VTPKFeatureSource::Options::fromConfig(const Config& conf)
{
    conf.get("url", url());
}