/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/PMTiles>
#include <osgEarth/Registry>
#include <osgEarth/StringUtils>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgDB/FileUtils>
#include <sstream>

using namespace osgEarth;
using namespace osgEarth::PMTiles;

#undef LC
#define LC "[PMTiles] " << getName() << " : "

//...................................................................

void
PMTiles::Options::writeTo(Config& conf) const
{
    conf.set("url", _url);
}

void
PMTiles::Options::readFrom(const Config& conf)
{
    conf.get("url", _url);
}

//...................................................................

Config
PMTilesImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    writeTo(conf);
    return conf;
}

void
PMTilesImageLayer::Options::fromConfig(const Config& conf)
{
    readFrom(conf);
}

//...................................................................

REGISTER_OSGEARTH_LAYER(pmtilesimage, PMTilesImageLayer);

OE_LAYER_PROPERTY_IMPL(PMTilesImageLayer, URI, URL, url);

void
PMTilesImageLayer::init()
{
    ImageLayer::init();

    // by default don't cache from local files
    layerHints().cachePolicy() = CachePolicy::NO_CACHE;
}

Status
PMTilesImageLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    Status status = _driver.open(
        getName(),
        options(),
        getReadOptions());

    if (status.isError())
    {
        return status;
    }

    setProfile(Profile::create(Profile::SPHERICAL_MERCATOR));

    // Set the data extents to the entire profile with the specified min/max level
    DataExtentList dataExtents;
    DataExtent e(getProfile()->getExtent());
    e.minLevel() = options().minLevel();
    e.maxLevel() = options().maxLevel();
    dataExtents.emplace_back(e);
    setDataExtents(dataExtents);

    return Status::NoError;
}

GeoImage
PMTilesImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (getStatus().isError())
        return GeoImage(getStatus());

    ReadResult r = _driver.read(key, progress, getReadOptions());

    if (r.succeeded())
        return GeoImage(r.releaseImage(), key.getExtent());
    else
        return GeoImage(Status(r.errorDetail()));
}


//...................................................................

Config
PMTilesElevationLayer::Options::getConfig() const
{
    Config conf = ElevationLayer::Options::getConfig();
    writeTo(conf);
    return conf;
}

void
PMTilesElevationLayer::Options::fromConfig(const Config& conf)
{
    readFrom(conf);
}


//........................................................................

REGISTER_OSGEARTH_LAYER(pmtileselevation, PMTilesElevationLayer);

OE_LAYER_PROPERTY_IMPL(PMTilesElevationLayer, URI, URL, url);

void
PMTilesElevationLayer::init()
{
    ElevationLayer::init();

    // by default don't cache from local files
    layerHints().cachePolicy() = CachePolicy::NO_CACHE;
}

Status
PMTilesElevationLayer::openImplementation()
{
    Status parent = ElevationLayer::openImplementation();
    if (parent.isError())
        return parent;


    Status status = _driver.open(
        getName(),
        options(),
        getReadOptions());

    if (status.isError())
    {
        return status;
    }

    setProfile(Profile::create(Profile::SPHERICAL_MERCATOR));

    // Set the data extents to the entire profile with the specified min/max level
    DataExtentList dataExtents;
    DataExtent e(getProfile()->getExtent());
    e.minLevel() = options().minLevel();
    e.maxLevel() = options().maxLevel();
    dataExtents.emplace_back(e);
    setDataExtents(dataExtents);

    return Status::NoError;
}

GeoHeightField
PMTilesElevationLayer::createHeightFieldImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (getStatus().isError())
        return GeoHeightField(getStatus());

    ReadResult r = _driver.read(key, progress, getReadOptions());

    if (r.succeeded() && r.getImage())
    {
        ImageToHeightFieldConverter conv;
        osg::HeightField* hf = conv.convert(r.getImage());
        return GeoHeightField(hf, key.getExtent());
    }
    else
    {
        return GeoHeightField(Status(r.errorDetail()));
    }
}

//...................................................................

#undef LC
#define LC "[PMTiles] \"" << _name << "\" "

PMTiles::Driver::Driver() :
    _internalCompression(pmtiles::COMPRESSION_UNKNOWN),
    _tileCompression(pmtiles::COMPRESSION_UNKNOWN),
    _minLevel(0),
    _maxLevel(19)
{
}

Driver::~Driver()
{
}

Status
PMTiles::Driver::open(
    const std::string& name,
    const PMTiles::Options& options,
    const osgDB::Options* readOptions)
{
    _name = name;

    std::string fileName = options.url()->full();
    if (!osgDB::fileExists(fileName))
    {
        OE_WARN << fileName << " doesn't exist" << std::endl;
        return Status::ResourceUnavailable;
    }

    // Read the first 127 bytes of the pmtiles file to parse the header
    _inputFile.open(fileName, std::ios::binary);
    _headerStr.resize(127);
    read(0, 127, _headerStr);

    auto header = pmtiles::deserialize_header(_headerStr);

    _minLevel = header.min_zoom;
    _maxLevel = header.max_zoom;
    _internalCompression = header.internal_compression;
    _tileCompression = header.tile_compression;

    
    std::string metadata_s;
    metadata_s.resize(header.json_metadata_bytes);
    read(header.json_metadata_offset, header.json_metadata_bytes, metadata_s);
    metadata_s = decompress(metadata_s, header.internal_compression);


    if (header.tile_type == pmtiles::TILETYPE_JPEG)
    {
        _rw = osgDB::Registry::instance()->getReaderWriterForMimeType("image/jpeg");
    }
    else if (header.tile_type == pmtiles::TILETYPE_PNG)
    {
        _rw = osgDB::Registry::instance()->getReaderWriterForMimeType("image/png");
    }
    else if (header.tile_type == pmtiles::TILETYPE_WEBP)
    {
        _rw = osgDB::Registry::instance()->getReaderWriterForMimeType("image/webp");
    }
    else
    {
        OE_WARN << LC << "Unsupported tile type: " << (int)header.tile_type << std::endl;
        return Status::ConfigurationError;
    }

    return Status::OK();
}

// Modified version of get_tile from pmtiles.hpp to not require mmap
// https://github.com/protomaps/PMTiles/blob/main/cpp/pmtiles.hpp#L607
std::pair<uint64_t, uint32_t> PMTiles::Driver::get_tile_offset_and_length(uint8_t z, uint32_t x, uint32_t y) const
{
    uint64_t tile_id = pmtiles::zxy_to_tileid(z, x, y);

    auto h = pmtiles::deserialize_header(_headerStr);

    uint64_t dir_offset = h.root_dir_offset;
    if (h.root_dir_bytes > std::numeric_limits<uint32_t>::max()) {
        throw pmtiles::malformed_directory_exception();
    }
    uint32_t dir_length = static_cast<uint32_t>(h.root_dir_bytes);
    for (int depth = 0; depth <= 3; depth++) {
        std::string dir_s;
        dir_s.resize(dir_length);
        read(dir_offset, dir_length, dir_s);
        std::string decompressed_dir = decompress(dir_s, h.internal_compression);
        auto dir_entries = pmtiles::deserialize_directory(decompressed_dir);
        auto entry = pmtiles::find_tile(dir_entries, tile_id);

        if (entry.length > 0) {
            if (entry.run_length > 0) {
                return std::make_pair(h.tile_data_offset + entry.offset, entry.length);
            }
            else {
                dir_offset = h.leaf_dirs_offset + entry.offset;
                dir_length = entry.length;
            }
        }
        else {
            return std::make_pair(0, 0);
        }
    }

    return std::make_pair(0, 0);
}


ReadResult
PMTiles::Driver::read(
    const TileKey& key,
    ProgressCallback* progress,
    const osgDB::Options* readOptions) const
{
    int z = key.getLevelOfDetail();
    int x = key.getTileX();
    int y = key.getTileY();

    if (z < (int)_minLevel)
    {
        return ReadResult::RESULT_NOT_FOUND;
    }

    if (z > (int)_maxLevel)
    {
        return ReadResult::RESULT_NOT_FOUND;
    }

    auto tile_offset_and_length = get_tile_offset_and_length(z, x, y);
    if (tile_offset_and_length.first && tile_offset_and_length.second == 0)
    {
        return ReadResult::RESULT_NOT_FOUND;
    }

    std::string compressed_tile;
    read(tile_offset_and_length.first, tile_offset_and_length.second, compressed_tile);
    std::string tile_data = decompress(compressed_tile, _tileCompression);
    std::stringstream ss(tile_data);
    return ReadResult(_rw->readImage(ss, _dbOptions.get()).takeImage());
}

bool PMTiles::Driver::read(uint64_t offset, uint32_t length, std::string& result) const
{
    std::lock_guard<std::mutex> lk(_mutex);
    result.resize(length);
    _inputFile.seekg(offset);
    _inputFile.read(&result[0], length);
    return false;
}

std::string PMTiles::Driver::decompress(const std::string& compressed, uint8_t compression) const
{
    if (compression == pmtiles::COMPRESSION_NONE || compression == pmtiles::COMPRESSION_UNKNOWN)
    {
        return compressed;
    }

    if (compression == pmtiles::COMPRESSION_GZIP)
    {
        osg::ref_ptr< osgDB::BaseCompressor> compressor = osgDB::Registry::instance()->getObjectWrapperManager()->findCompressor("zlib");
        std::istringstream in(compressed);
        std::string value;
        compressor->decompress(in, value);
        return value;
    }
    else
    {
        OE_WARN << LC << "Unsupported compression type: " << (int)compression << std::endl;
        return std::string();
    }
}