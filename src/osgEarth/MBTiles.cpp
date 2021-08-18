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
#include <osgEarth/MBTiles>
#include <osgEarth/Registry>
#include <osgEarth/FileUtils>
#include <osgEarth/XmlUtils>
#include <osgEarth/StringUtils>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgDB/FileUtils>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <sqlite3.h>

using namespace osgEarth;
using namespace osgEarth::MBTiles;

#undef LC
#define LC "[MBTiles] " << getName() << " : "

//......................................................................

namespace
{
    osgDB::ReaderWriter* getReaderWriter(const std::string& format)
    {
        osgDB::ReaderWriter* rw = 0L;

        // Get a ReaderWriter for the tile format. Try both mime-type and extension.
        rw = osgDB::Registry::instance()->getReaderWriterForMimeType( format );
        if ( rw == 0L )
        {
            rw = osgDB::Registry::instance()->getReaderWriterForExtension( format );
        }
        return rw;
    }
}

//...................................................................

void
MBTiles::Options::writeTo(Config& conf) const
{
    conf.set("filename", _url);
    conf.set("format", _format);
    conf.set("compress", _compress);
}

void
MBTiles::Options::readFrom(const Config& conf)
{
    format().init("png");
    compress().init(false);

    conf.get("filename", _url);
    conf.get("url", _url); // compat for consistency with other drivers
    conf.get("format", _format);
    conf.get("compress", _compress);
}

//...................................................................

Config
MBTilesImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    writeTo(conf);
    return conf;
}

void
MBTilesImageLayer::Options::fromConfig(const Config& conf)
{
    readFrom(conf);
}

//...................................................................

REGISTER_OSGEARTH_LAYER(mbtilesimage, MBTilesImageLayer);

OE_LAYER_PROPERTY_IMPL(MBTilesImageLayer, URI, URL, url);
OE_LAYER_PROPERTY_IMPL(MBTilesImageLayer, std::string, Format, format);
OE_LAYER_PROPERTY_IMPL(MBTilesImageLayer, bool, Compress, compress);

void
MBTilesImageLayer::init()
{
    ImageLayer::init();

    // by default don't cache from local files
    layerHints().cachePolicy() = CachePolicy::NO_CACHE;
}

Status
MBTilesImageLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    osg::ref_ptr<const Profile> profile = getProfile();

    Status status = _driver.open(
        getName(),
        options(),
        isWritingRequested(),
        options().format(),
        profile,
        dataExtents(),
        getReadOptions());

    if (status.isError())
    {
        return status;
    }

    // install the profile if there is one
    if (getProfile() == NULL && profile.valid())
    {
        setProfile(profile.get());
    }

    return Status::NoError;
}

void
MBTilesImageLayer::setDataExtents(const DataExtentList& values)
{
    dataExtents() = values;

    if (isWritingRequested())
    {
        _driver.setDataExtents(values);
    }
}

GeoImage
MBTilesImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (getStatus().isError())
        return GeoImage(getStatus());

    ReadResult r = _driver.read(key, progress, getReadOptions());

    if (r.succeeded())
        return GeoImage(r.releaseImage(), key.getExtent());
    else
        return GeoImage(Status(r.errorDetail()));
}

Status
MBTilesImageLayer::writeImageImplementation(const TileKey& key, const osg::Image* image, ProgressCallback* progress) const
{
    if (getStatus().isError())
        return getStatus();

    if (!isWritingRequested())
        return Status::ServiceUnavailable;

    return _driver.write( key, image, progress );
}

bool MBTilesImageLayer::getMetaData(const std::string& name, std::string& value)
{
    return _driver.getMetaData(name, value);
}

bool MBTilesImageLayer::putMetaData(const std::string& name, const std::string& value)
{
    return _driver.putMetaData(name, value);
}

//...................................................................

Config
MBTilesElevationLayer::Options::getConfig() const
{
    Config conf = ElevationLayer::Options::getConfig();
    writeTo(conf);
    return conf;
}

void
MBTilesElevationLayer::Options::fromConfig(const Config& conf)
{
    readFrom(conf);
}


//........................................................................

REGISTER_OSGEARTH_LAYER(mbtileselevation, MBTilesElevationLayer);

OE_LAYER_PROPERTY_IMPL(MBTilesElevationLayer, URI, URL, url);
OE_LAYER_PROPERTY_IMPL(MBTilesElevationLayer, std::string, Format, format);
OE_LAYER_PROPERTY_IMPL(MBTilesElevationLayer, bool, Compress, compress);

void
MBTilesElevationLayer::init()
{
    ElevationLayer::init();

    // by default don't cache from local files
    layerHints().cachePolicy() = CachePolicy::NO_CACHE;
}

Status
MBTilesElevationLayer::openImplementation()
{
    Status parent = ElevationLayer::openImplementation();
    if (parent.isError())
        return parent;

    osg::ref_ptr<const Profile> profile = getProfile();

    Status status = _driver.open(
        getName(),
        options(),
        isWritingRequested(),
        options().format(),
        profile,
        dataExtents(),
        getReadOptions());

    if (status.isError())
    {
        return status;
    }

    // install the profile if there is one
    if (getProfile() == NULL && profile.valid())
    {
        setProfile(profile.get());
    }

    return Status::NoError;
}

void
MBTilesElevationLayer::setDataExtents(const DataExtentList& values)
{
    dataExtents() = values;

    if (isWritingRequested())
    {
        _driver.setDataExtents(values);
    }
}

GeoHeightField
MBTilesElevationLayer::createHeightFieldImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (getStatus().isError())
        return GeoHeightField(getStatus());

    ReadResult r = _driver.read(key, progress, getReadOptions());

    if (r.succeeded() && r.getImage())
    {
        ImageToHeightFieldConverter conv;
        osg::HeightField* hf = conv.convert( r.getImage() );
        return GeoHeightField(hf, key.getExtent());
    }
    else
    {
        return GeoHeightField(Status(r.errorDetail()));
    }
}

Status
MBTilesElevationLayer::writeHeightFieldImplementation(const TileKey& key, const osg::HeightField* hf, ProgressCallback* progress) const
{
    if (getStatus().isError())
        return getStatus();

    if (!hf)
        return Status::AssertionFailure;

    if (!isWritingRequested())
        return Status::ServiceUnavailable;

    ImageToHeightFieldConverter conv;
    osg::Image* image = conv.convert(hf);
    if (image)
    {
        return _driver.write(key, image, progress);
    }
    else
    {
        return Status(Status::GeneralError, "Hf to Image conversion failed");
    }
}

bool MBTilesElevationLayer::getMetaData(const std::string& name, std::string& value)
{
    return _driver.getMetaData(name, value);
}

bool MBTilesElevationLayer::putMetaData(const std::string& name, const std::string& value)
{
    return _driver.putMetaData(name, value);
}

//...................................................................

#undef LC
#define LC "[MBTiles] Layer \"" << _name << "\" "

MBTiles::Driver::Driver() :
    _minLevel(0),
    _maxLevel(19),
    _forceRGB(false),
    _database(NULL),
    _mutex("MBTiles Driver(OE)")
{
    //nop
}

Status
MBTiles::Driver::open(
    const std::string& name,
    const MBTiles::Options& options,
    bool isWritingRequested,
    const optional<std::string>& format,
    osg::ref_ptr<const Profile>& inout_profile,
    DataExtentList& out_dataExtents,
    const osgDB::Options* readOptions)
{
    _name = name;

    _dbOptions = readOptions;

    std::string fullFilename = options.url()->full();
    if (!osgDB::fileExists(fullFilename))
    {
        fullFilename = osgDB::findDataFile(fullFilename, readOptions);
        if (fullFilename.empty())
            fullFilename = options.url()->full();
    }

    bool readWrite = isWritingRequested;

    bool isNewDatabase = readWrite && !osgDB::fileExists(fullFilename);

    if (isNewDatabase)
    {
        // For a NEW database, the profile MUST be set prior to initialization.
        if (inout_profile.valid() == false)
        {
            return Status(Status::ConfigurationError,
                "Cannot create database; required Profile is missing");
        }

        if (!format.isSet())
        {
            return Status(Status::ConfigurationError,
                "Cannot create database; required format property is missing");
        }

        OE_INFO << LC << "Database does not exist; attempting to create it." << std::endl;
    }

    // Try to open (or create) the database. We use SQLITE_OPEN_NOMUTEX to do
    // our own mutexing.
    int flags = readWrite
        ? (SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_NOMUTEX)
        : (SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX);

    sqlite3* database = (sqlite3*)_database;
    sqlite3** dbptr = (sqlite3**)&_database;
    int rc = sqlite3_open_v2(fullFilename.c_str(), dbptr, flags, 0L);
    if (rc != 0)
    {
        return Status(Status::ResourceUnavailable, Stringify()
            << "Database \"" << fullFilename << "\": " << sqlite3_errmsg(database));
    }

    // New database setup:
    if (isNewDatabase)
    {
        // Make sure we have a readerwriter for the underlying tile format:
        _tileFormat = format.get();
        _rw = getReaderWriter(_tileFormat);
        if (!_rw.valid())
        {
            return Status(Status::ServiceUnavailable,
                "No plugin found to load format \"" + _tileFormat + "\"");
        }

        // create necessary db tables:
        createTables();

        // write profile to metadata:
        std::string profileJSON = inout_profile->toProfileOptions().getConfig().toJSON(false);
        putMetaData("profile", profileJSON);

        // write format to metadata:
        putMetaData("format", _tileFormat);

        // compression?
        if (options.compress().isSetTo(true))
        {
            _compressor = osgDB::Registry::instance()->getObjectWrapperManager()->findCompressor("zlib");
            if (_compressor.valid())
            {
                putMetaData("compression", "zlib");
                OE_INFO << LC << "Data will be compressed (zlib)" << std::endl;
            }
        }

        // initialize and update as we write tiles.
        _minLevel = 0u;
        _maxLevel = 0u;
    }

    // If the database pre-existed, read in the information from the metadata.
    else // !isNewDatabase
    {
        computeLevels();
        OE_INFO << LC << "Got levels from database " << _minLevel << ", " << _maxLevel << std::endl;

        std::string profileStr;
        getMetaData("profile", profileStr);

        // The data format (e.g., png, jpg, etc.). Any format passed in
        // in the options is superseded by the one in the database metadata.
        std::string metaDataFormat;
        getMetaData("format", metaDataFormat);
        if (!metaDataFormat.empty())
        {
            _tileFormat = metaDataFormat;
        }

        // Try to get it from the options.
        if (_tileFormat.empty())
        {
            if (format.isSet())
            {
                _tileFormat = format.get();
            }
        }
        else
        {
            // warn the user if the options format differs from the database format
            if (format.isSet() && format.get() != _tileFormat)
            {
                OE_WARN << LC
                    << "Database tile format (" << _tileFormat << ") will override the layer options format ("
                    << format.get() << ")" << std::endl;
            }
        }

        // By this point, we require a valid tile format.
        if (_tileFormat.empty())
        {
            return Status(Status::ConfigurationError, "Required format not in metadata, nor specified in the options.");
        }

        // Make sure we have a readerwriter for the tile format:
        _rw = getReaderWriter(_tileFormat);
        if (!_rw.valid())
        {
            return Status(Status::ServiceUnavailable,
                "No plugin found to load format \"" + _tileFormat + "\"");
        }

        // check for compression.
        std::string compression;
        getMetaData("compression", compression);
        if (!compression.empty())
        {
            _compressor = osgDB::Registry::instance()->getObjectWrapperManager()->findCompressor(compression);
            if (!_compressor.valid())
                return Status(Status::ServiceUnavailable, "Cannot find compressor \"" + compression + "\"");
            else
                OE_INFO << LC << "Data is compressed (" << compression << ")" << std::endl;
        }

        // Set the profile
        const Profile* profile = inout_profile.get();
        if (!profile)
        {
            if (!profileStr.empty())
            {
                // try to parse it as a JSON config
                Config pconf;
                pconf.fromJSON(profileStr);
                profile = Profile::create(ProfileOptions(pconf));

                // if that didn't work, try parsing it directly
                if (!profile)
                {
                    profile = Profile::create(profileStr);
                }
            }

            if (!profile)
            {
                if (profileStr.empty() == false)
                    OE_WARN << LC << "Profile \"" << profileStr << "\" not recognized; defaulting to spherical-mercator\n";

                profile = Profile::create(Profile::SPHERICAL_MERCATOR);
            }

            inout_profile = profile;
            OE_INFO << LC << "Profile = " << profile->toString() << std::endl;
            OE_INFO << LC << "Min=" << _minLevel << ", Max=" << _maxLevel
                << ", format=" << _tileFormat  << std::endl;
        }

        // Check for bounds and populate DataExtents.
        std::string boundsStr;
        if (getMetaData("bounds", boundsStr))
        {
            std::vector<std::string> tokens;
            StringTokenizer(",").tokenize(boundsStr, tokens);
            if (tokens.size() == 4)
            {
                double minLon = osgEarth::Util::as<double>(tokens[0], 0.0);
                double minLat = osgEarth::Util::as<double>(tokens[1], 0.0);
                double maxLon = osgEarth::Util::as<double>(tokens[2], 0.0);
                double maxLat = osgEarth::Util::as<double>(tokens[3], 0.0);

                GeoExtent extent;
                if (profile)
                    extent = GeoExtent(profile->getSRS()->getGeographicSRS(), minLon, minLat, maxLon, maxLat);
                else
                    extent = GeoExtent(osgEarth::SpatialReference::get("wgs84"), minLon, minLat, maxLon, maxLat);

                if (extent.isValid())
                {
                    // Using 0 for the minLevel is not technically correct, but we use it instead of the proper minLevel to force osgEarth to subdivide
                    // since we don't really handle DataExtents with minLevels > 0 just yet.
                    out_dataExtents.push_back(DataExtent(extent, 0, _maxLevel));
                    OE_INFO << LC << "Bounds = " << extent.toString() << std::endl;
                }
                else
                {
                    OE_WARN << LC << "MBTiles has invalid bounds " << extent.toString() << std::endl;
                }
            }
        }
        else
        {
            // Using 0 for the minLevel is not technically correct, but we use it instead of the proper minLevel to force osgEarth to subdivide
            // since we don't really handle DataExtents with minLevels > 0 just yet.
            out_dataExtents.push_back(DataExtent(inout_profile->getExtent(), 0, _maxLevel));
        }
    }

    // do we require RGB? for jpeg?
    _forceRGB =
        osgEarth::endsWith(_tileFormat, "jpg", false) ||
        osgEarth::endsWith(_tileFormat, "jpeg", false);

    // make an empty image.
    int size = 256;
    _emptyImage = new osg::Image();
    _emptyImage->allocateImage(size, size, 1, GL_RGBA, GL_UNSIGNED_BYTE);
    unsigned char *data = _emptyImage->data(0, 0);
    memset(data, 0, 4 * size * size);

    return Status::OK();
}

int
MBTiles::Driver::readMaxLevel()
{
    int result = -1;

    sqlite3* database = (sqlite3*)_database;

    //Get the image
    sqlite3_stmt* select = NULL;
    std::string query = "SELECT zoom_level FROM tiles ORDER BY zoom_level DESC LIMIT 1";
    int rc = sqlite3_prepare_v2( database, query.c_str(), -1, &select, 0L );
    if ( rc != SQLITE_OK )
    {
        OE_WARN << LC << "Failed to prepare SQL: " << query << "; " << sqlite3_errmsg(database) << std::endl;
        return ReadResult::RESULT_READER_ERROR;
    }

    rc = sqlite3_step( select );
    if (rc == SQLITE_ROW)
    {
        result = sqlite3_column_int(select, 0);
    }
    else
    {
        OE_DEBUG << LC << "SQL QUERY failed for " << query << ": " << std::endl;
        return -1;
    }

    sqlite3_finalize( select );
    return result;
}

ReadResult
MBTiles::Driver::read(
    const TileKey& key,
    ProgressCallback* progress,
    const osgDB::Options* readOptions) const
{
    Threading::ScopedMutexLock exclusiveLock(_mutex);

    int z = key.getLevelOfDetail();
    int x = key.getTileX();
    int y = key.getTileY();

    if (z < (int)_minLevel)
    {
        return ReadResult::RESULT_NOT_FOUND;
    }

    if (z > (int)_maxLevel)
    {
        //If we're at the max level, just return NULL
        return ReadResult::RESULT_NOT_FOUND;
    }

    unsigned int numRows, numCols;
    key.getProfile()->getNumTiles(key.getLevelOfDetail(), numCols, numRows);
    y  = numRows - y - 1;

    sqlite3* database = (sqlite3*)_database;

    //Get the image
    sqlite3_stmt* select = NULL;
    std::string query = "SELECT tile_data from tiles where zoom_level = ? AND tile_column = ? AND tile_row = ?";
    int rc = sqlite3_prepare_v2( database, query.c_str(), -1, &select, 0L );
    if ( rc != SQLITE_OK )
    {
        OE_WARN << LC << "Failed to prepare SQL: " << query << "; " << sqlite3_errmsg(database) << std::endl;
        return ReadResult::RESULT_READER_ERROR;
    }

    bool valid = true;

    sqlite3_bind_int( select, 1, z );
    sqlite3_bind_int( select, 2, x );
    sqlite3_bind_int( select, 3, y );

    osg::Image* result = NULL;
    rc = sqlite3_step( select );
    if ( rc == SQLITE_ROW)
    {
        // the pointer returned from _blob gets freed internally by sqlite, supposedly
        const char* data = (const char*)sqlite3_column_blob( select, 0 );
        int dataLen = sqlite3_column_bytes( select, 0 );

        std::string dataBuffer( data, dataLen );

        // decompress if necessary:
        if ( _compressor.valid() )
        {
            std::istringstream inputStream(dataBuffer);
            std::string value;
            if ( !_compressor->decompress(inputStream, value) )
            {
                OE_WARN << LC << "Decompression failed" << std::endl;
                valid = false;
            }
            else
            {
                dataBuffer = value;
            }
        }

        // decode the raw image data:
        if ( valid )
        {
            std::istringstream inputStream(dataBuffer);
            result = ImageUtils::readStream(inputStream, _dbOptions.get());
            // If we couldn't load the image automatically try the reader instead.
            if (!result && _rw.valid())
            {
                result = _rw->readImage(inputStream, _dbOptions.get()).takeImage();
            }
        }
    }
    else
    {
        OE_DEBUG << LC << "SQL QUERY failed for " << query << ": " << std::endl;
        valid = false;
    }

    sqlite3_finalize( select );

    return ReadResult(result);
}


Status
MBTiles::Driver::write(
    const TileKey& key,
    const osg::Image* image,
    ProgressCallback* progress)
{
    if (!key.valid() || !image)
        return Status::AssertionFailure;

    Threading::ScopedMutexLock exclusiveLock(_mutex);

    // encode the data stream:
    std::stringstream buf;
    osgDB::ReaderWriter::WriteResult wr;
    if (_forceRGB && ImageUtils::hasAlphaChannel(image))
    {
        //TODO: skip if unnecessary
        osg::ref_ptr<osg::Image> rgb = ImageUtils::convertToRGB8(image);
        wr = _rw->writeImage(*(rgb.get()), buf, _dbOptions.get());
    }
    else
    {
        wr = _rw->writeImage(*image, buf, _dbOptions.get());
    }

    if (wr.error())
    {
        return Status(Status::GeneralError, "Image encoding failed");
    }

    std::string value = buf.str();

    // compress if necessary:
    if (_compressor.valid())
    {
        std::ostringstream output;
        if (!_compressor->compress(output, value))
        {
            return Status(Status::GeneralError, "Compressor failed");
        }
        value = output.str();
    }

    int z = key.getLOD();
    int x = key.getTileX();
    int y = key.getTileY();

    // flip Y axis
    unsigned int numRows, numCols;
    key.getProfile()->getNumTiles(key.getLevelOfDetail(), numCols, numRows);
    y = numRows - y - 1;

    sqlite3* database = (sqlite3*)_database;

    // Prep the insert statement:
    sqlite3_stmt* insert = NULL;
    std::string query = "INSERT OR REPLACE INTO tiles (zoom_level, tile_column, tile_row, tile_data) VALUES (?, ?, ?, ?)";
    int rc = sqlite3_prepare_v2(database, query.c_str(), -1, &insert, 0L);
    if (rc != SQLITE_OK)
    {
        return Status(Status::GeneralError, Stringify()
            << "Failed to prepare SQL: " << query << "; " << sqlite3_errmsg(database));
    }

    // bind parameters:
    sqlite3_bind_int(insert, 1, z);
    sqlite3_bind_int(insert, 2, x);
    sqlite3_bind_int(insert, 3, y);

    // bind the data blob:
    sqlite3_bind_blob(insert, 4, value.c_str(), value.length(), SQLITE_STATIC);

    // run the sql.
    bool ok = true;
    int tries = 0;
    do {
        rc = sqlite3_step(insert);
    } while (++tries < 100 && (rc == SQLITE_BUSY || rc == SQLITE_LOCKED));

    if (SQLITE_OK != rc && SQLITE_DONE != rc)
    {
#if SQLITE_VERSION_NUMBER >= 3007015
        return Status(Status::GeneralError, Stringify()<<"Failed query: " << query << "(" << rc << ")" << sqlite3_errstr(rc) << "; " << sqlite3_errmsg(database));
#else
        return Status(Status::GeneralError, Stringify()<< "Failed query: " << query << "(" << rc << ")" << rc << "; " << sqlite3_errmsg(database));
#endif
        ok = false;
    }

    sqlite3_finalize(insert);

    // adjust the max level if necessary
    if (key.getLOD() > _maxLevel)
    {
        _maxLevel = key.getLOD();
        //putMetaData("maxlevel", Stringify()<<_maxLevel);
    }
    if (key.getLOD() < _minLevel)
    {
        _minLevel = key.getLOD();
        //putMetaData("minlevel", Stringify()<<_minLevel);
    }

    return Status::NoError;
}

bool
MBTiles::Driver::getMetaData(const std::string& key, std::string& value)
{
    Threading::ScopedMutexLock exclusiveLock(_mutex);

    sqlite3* database = (sqlite3*)_database;

    //get the metadata
    sqlite3_stmt* select = NULL;
    std::string query = "SELECT value from metadata where name = ?";
    int rc = sqlite3_prepare_v2( database, query.c_str(), -1, &select, 0L );
    if ( rc != SQLITE_OK )
    {
        OE_WARN << LC << "Failed to prepare SQL: " << query << "; " << sqlite3_errmsg(database) << std::endl;
        return false;
    }


    bool valid = true;
    std::string keyStr = std::string( key );
    rc = sqlite3_bind_text( select, 1, keyStr.c_str(), keyStr.length(), SQLITE_STATIC );
    if (rc != SQLITE_OK )
    {
        OE_WARN << LC << "Failed to bind text: " << query << "; " << sqlite3_errmsg(database) << std::endl;
        return false;
    }

    rc = sqlite3_step( select );
    if ( rc == SQLITE_ROW)
    {
        value = (char*)sqlite3_column_text( select, 0 );
    }
    else
    {
        OE_DEBUG << LC << "SQL QUERY failed for " << query << ": " << std::endl;
        valid = false;
    }

    sqlite3_finalize( select );
    return valid;
}

bool
MBTiles::Driver::putMetaData(const std::string& key, const std::string& value)
{
    Threading::ScopedMutexLock exclusiveLock(_mutex);

    sqlite3* database = (sqlite3*)_database;

    // prep the insert statement.
    sqlite3_stmt* insert = 0L;
    std::string query = Stringify() << "INSERT OR REPLACE INTO metadata (name,value) VALUES (?,?)";
    if ( SQLITE_OK != sqlite3_prepare_v2(database, query.c_str(), -1, &insert, 0L) )
    {
        OE_WARN << LC << "Failed to prepare SQL: " << query << "; " << sqlite3_errmsg(database) << std::endl;
        return false;
    }

    // bind the values:
    if( SQLITE_OK != sqlite3_bind_text(insert, 1, key.c_str(), key.length(), SQLITE_STATIC) )
    {
        OE_WARN << LC << "Failed to bind text: " << query << "; " << sqlite3_errmsg(database) << std::endl;
        return false;
    }
    if ( SQLITE_OK != sqlite3_bind_text(insert, 2, value.c_str(), value.length(), SQLITE_STATIC) )
    {
        OE_WARN << LC << "Failed to bind text: " << query << "; " << sqlite3_errmsg(database) << std::endl;
        return false;
    }

    // execute the sql. no idea what a good return value should be :/
    sqlite3_step( insert );
    sqlite3_finalize( insert );
    return true;
}

void
MBTiles::Driver::computeLevels()
{
    sqlite3* database = (sqlite3*)_database;
    osg::Timer_t startTime = osg::Timer::instance()->tick();
    sqlite3_stmt* select = NULL;
    // Get min and max as separate queries to allow the SQLite query planner to convert it to a fast equivalent.
    std::string query = "SELECT (SELECT min(zoom_level) FROM tiles), (SELECT max(zoom_level) FROM tiles); ";
    int rc = sqlite3_prepare_v2( database, query.c_str(), -1, &select, 0L );
    if ( rc != SQLITE_OK )
    {
        OE_WARN << LC << "Failed to prepare SQL: " << query << "; " << sqlite3_errmsg(database) << std::endl;
    }

    rc = sqlite3_step( select );
    if ( rc == SQLITE_ROW)
    {
        _minLevel = sqlite3_column_int( select, 0 );
        _maxLevel = sqlite3_column_int( select, 1 );
        OE_DEBUG << LC << "Min=" << _minLevel << " Max=" << _maxLevel << std::endl;
    }
    else
    {
        OE_DEBUG << LC << "SQL QUERY failed for " << query << ": " << std::endl;
    }
    sqlite3_finalize( select );
    osg::Timer_t endTime = osg::Timer::instance()->tick();
    OE_DEBUG << LC << "Computing levels took " << osg::Timer::instance()->delta_s(startTime, endTime ) << " s" << std::endl;
}

bool
MBTiles::Driver::createTables()
{
    // https://github.com/mapbox/mbtiles-spec/blob/master/1.2/spec.md

    sqlite3* database = (sqlite3*)_database;

    std::string query =
        "CREATE TABLE IF NOT EXISTS metadata ("
        " name text PRIMARY KEY,"
        " value text)";

    if (SQLITE_OK != sqlite3_exec(database, query.c_str(), 0L, 0L, 0L))
    {
        OE_WARN << LC << "Failed to create table [metadata]" << std::endl;
        return false;
    }

    query =
        "CREATE TABLE IF NOT EXISTS tiles ("
        " zoom_level integer,"
        " tile_column integer,"
        " tile_row integer,"
        " tile_data blob)";

    char* errorMsg = 0L;

    if (SQLITE_OK != sqlite3_exec(database, query.c_str(), 0L, 0L, &errorMsg))
    {
        OE_WARN << LC << "Failed to create table [tiles]: " << errorMsg << std::endl;
        sqlite3_free( errorMsg );
        return false;
    }

    // create an index
    query =
        "CREATE UNIQUE INDEX tile_index ON tiles ("
        " zoom_level, tile_column, tile_row)";

    if (SQLITE_OK != sqlite3_exec(database, query.c_str(), 0L, 0L, &errorMsg))
    {
        OE_WARN << LC << "Failed to create index on table [tiles]: " << errorMsg << std::endl;
        sqlite3_free( errorMsg );
        // keep going...
        // return false;
    }

    // TODO: support "grids" and "grid_data" tables if necessary.

    return true;
}

void
MBTiles::Driver::setDataExtents(const DataExtentList& values)
{
    if (_database != NULL && values.size() > 0)
    {
        // Get the union of all the extents
        GeoExtent e(values[0]);
        for (unsigned int i = 1; i < values.size(); i++)
        {
            e.expandToInclude(values[i]);
        }

        // Convert the bounds to geographic
        GeoExtent bounds;
        if (e.getSRS()->isGeographic())
        {
            bounds = e;
        }
        else
        {
            osg::ref_ptr<const Profile> gg = Profile::create(Profile::GLOBAL_GEODETIC);
            bounds = gg->clampAndTransformExtent(e);
        }
        std::stringstream boundsStr;
        boundsStr << bounds.xMin() << "," << bounds.yMin() << "," << bounds.xMax() << "," << bounds.yMax();
        putMetaData("bounds", boundsStr.str());
    }
}
