/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include "MBTilesTileSource"

#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgDB/FileUtils>

#include <sstream>
#include <iomanip>
#include <algorithm>

#include <sqlite3.h>

#define LC "[MBTilesTileSource] "

using namespace osgEarth;
using namespace osgEarth::Drivers::MBTiles;

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

//......................................................................

MBTilesTileSource::MBTilesTileSource(const TileSourceOptions& options) :
TileSource( options ),
_options  ( options ),      
_database ( NULL ),
_minLevel ( 0 ),
_maxLevel ( 20 ),
_forceRGB ( false )
{
    //nop
}

Status
MBTilesTileSource::initialize(const osgDB::Options* dbOptions)
{    
    _dbOptions = Registry::instance()->cloneOrCreateOptions( dbOptions );    
    
    bool readWrite = (MODE_WRITE & (int)getMode()) != 0;

    std::string fullFilename = _options.filename()->full();   
    bool isNewDatabase = readWrite && !osgDB::fileExists(fullFilename);

    if ( isNewDatabase )
    {
        // For a NEW database, the profile MUST be set prior to initialization.
        if ( getProfile() == 0L )
            return Status::Error(Status::ConfigurationError, "Cannot create database; required Profile is missing");

        // For a NEW database the format is required.
        if ( _options.format().isSet() )
        {
            _tileFormat = _options.format().value();
            _rw = getReaderWriter( _tileFormat );
            if ( !_rw.valid() )
                return Status::Error(Status::ServiceUnavailable, "No plugin to load format \"" + _tileFormat + "\"");
        }
        else
        {
            return Status::Error(Status::ConfigurationError, "Cannot create database; required format is missing");
        }

        OE_INFO << LC << "Database does not exist; attempting to create it." << std::endl;
    }

    // Try to open (or create) the database. We use SQLITE_OPEN_NOMUTEX to do
    // our own mutexing.
    int flags = readWrite 
        ? (SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_NOMUTEX)
        : (SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX);
     
    int rc = sqlite3_open_v2( fullFilename.c_str(), &_database, flags, 0L );
    if ( rc != 0 )
    {                        
        return Status::Error( Status::ResourceUnavailable, Stringify()
            << "Database \"" << fullFilename << "\": " << sqlite3_errmsg(_database) );
    }
    
    // New database setup:
    if ( isNewDatabase )
    {
        // create necessary db tables:
        createTables();

        // write profile to metadata:
        std::string profileJSON = getProfile()->toProfileOptions().getConfig().toJSON(false);
        putMetaData("profile", profileJSON);

        // write format to metadata:
        putMetaData("format", _tileFormat);

        // compression?
        if ( _options.compress().isSetTo(true) )
        {
            _compressor = osgDB::Registry::instance()->getObjectWrapperManager()->findCompressor("zlib");
            if ( _compressor.valid() )
            {
                putMetaData("compression", "zlib");
                OE_INFO << LC << "Data will be compressed (zlib)" << std::endl;
            }
        }

        // If we have some data extents at this point, write the bounds to the metadata.
        if (getDataExtents().size() > 0)
        {
            // Get the union of all the extents
            GeoExtent e(getDataExtents()[0]);
            for (unsigned int i = 1; i < getDataExtents().size(); i++)
            {
                e.expandToInclude(getDataExtents()[i]);
            }

            // Convert the bounds to wgs84
            GeoExtent bounds = e.transform(osgEarth::SpatialReference::get("wgs84"));
            std::stringstream boundsStr;
            boundsStr << bounds.xMin() << "," << bounds.yMin() << "," << bounds.xMax() << "," << bounds.yMax();
            putMetaData("bounds", boundsStr.str());
        }
    }

    // If the database pre-existed, read in the information from the metadata.
    else // !isNewDatabase
    {
        if ( _options.computeLevels() == true )
        {
            computeLevels();
        }

        std::string profileStr;
        getMetaData( "profile", profileStr );

        // The data format (e.g., png, jpg, etc.). Any format passed in 
        // in the options is superceded by the one in the database metadata.
        std::string metaDataFormat;
        getMetaData( "format", metaDataFormat );
        if ( !metaDataFormat.empty() )
            _tileFormat = metaDataFormat;        

        // Try to get it from the options.
        if (_tileFormat.empty())
        {
            if ( _options.format().isSet() )
            {
                _tileFormat = _options.format().value();
            }
        }

        // By this point, we require a valid tile format.
        if ( _tileFormat.empty() )
            return Status::Error(Status::ConfigurationError, "Required format not in metadata, nor specified in the options.");

        _rw = getReaderWriter( _tileFormat );
        if ( !_rw.valid() )
            return Status::Error(Status::ServiceUnavailable, "No plugin to load format \"" + _tileFormat + "\"");

        // check for compression.
        std::string compression;
        getMetaData("compression", compression);
        if ( !compression.empty() )
        {
            _compressor = osgDB::Registry::instance()->getObjectWrapperManager()->findCompressor(compression);
            if ( !_compressor.valid() )
                return Status::Error(Status::ServiceUnavailable, "Cannot find compressor \"" + compression + "\"");
            else
                OE_INFO << LC << "Data is compressed (" << compression << ")" << std::endl;
        }        

        // Set the profile
        const Profile* profile = getProfile();
        if (!profile)
        {
            if (!profileStr.empty())
            {
                // try to parse it as a JSON config
                Config pconf;
                pconf.fromJSON(profileStr);
                profile = Profile::create(ProfileOptions(pconf));

                // if that didn't work, try parsing it directly
                if ( !profile )
                {
                    profile = Profile::create(profileStr);
                }

                if ( !profile )
                {
                    return Status::Error( Stringify() << "Profile not recognized: " << profileStr );
                }
            }
            
            if (!profile)
            {
                // Spherical mercator is the MBTiles default.
                profile = osgEarth::Registry::instance()->getSphericalMercatorProfile();
            }

            setProfile( profile );     
            OE_INFO << LC << "Profile = " << profileStr << std::endl;
        }

        // Check for bounds and populate DataExtents.
        std::string boundsStr;
        if ( getMetaData("bounds", boundsStr) )
        {
            std::vector<std::string> tokens;
            StringTokenizer(",").tokenize(boundsStr, tokens);
            if (tokens.size() == 4)
            {
                GeoExtent extent(
                    osgEarth::SpatialReference::get("wgs84"),
                    osgEarth::as<double>(tokens[0], 0.0),
                    osgEarth::as<double>(tokens[1], 0.0), // south
                    osgEarth::as<double>(tokens[2], 0.0), // east
                    osgEarth::as<double>(tokens[3], 0.0)  // north
                    );

                this->getDataExtents().push_back(DataExtent(extent, _minLevel, _maxLevel));

                OE_INFO << LC << "Bounds = " << extent.toString() << std::endl;
            }
        }
        else
        {
            this->getDataExtents().push_back(DataExtent(getProfile()->getExtent(), _minLevel, _maxLevel));
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
    unsigned char *data = _emptyImage->data(0,0);
    memset(data, 0, 4 * size * size);

    return STATUS_OK;
}    


CachePolicy
MBTilesTileSource::getCachePolicyHint(const Profile* targetProfile) const
{
    if ( !targetProfile || targetProfile->isHorizEquivalentTo(getProfile()) )
        return CachePolicy::NO_CACHE;
    else
        return CachePolicy::DEFAULT;
}


osg::Image*
MBTilesTileSource::createImage(const TileKey&    key,
                               ProgressCallback* progress)
{
    Threading::ScopedMutexLock exclusiveLock(_mutex);

    int z = key.getLevelOfDetail();
    int x = key.getTileX();
    int y = key.getTileY();

    if (z < (int)_minLevel)
    {
        return _emptyImage.get();            
    }

    if (z > (int)_maxLevel)
    {
        //If we're at the max level, just return NULL
        return NULL;
    }

    unsigned int numRows, numCols;
    key.getProfile()->getNumTiles(key.getLevelOfDetail(), numCols, numRows);
    y  = numRows - y - 1;

    //Get the image
    sqlite3_stmt* select = NULL;
    std::string query = "SELECT tile_data from tiles where zoom_level = ? AND tile_column = ? AND tile_row = ?";
    int rc = sqlite3_prepare_v2( _database, query.c_str(), -1, &select, 0L );
    if ( rc != SQLITE_OK )
    {
        OE_WARN << LC << "Failed to prepare SQL: " << query << "; " << sqlite3_errmsg(_database) << std::endl;
        return NULL;
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
            osgDB::ReaderWriter::ReadResult rr = _rw->readImage( inputStream, _dbOptions.get() );
            if (rr.validImage())
            {
                result = rr.takeImage();                
            }
        }
    }
    else
    {
        OE_DEBUG << LC << "SQL QUERY failed for " << query << ": " << std::endl;
        valid = false;
    }

    sqlite3_finalize( select );
    return result;
}

bool 
MBTilesTileSource::storeImage(const TileKey&    key,
                              osg::Image*       image,
                              ProgressCallback* progress)
{
    if ( (getMode() & MODE_WRITE) == 0 )
        return false;

    Threading::ScopedMutexLock exclusiveLock(_mutex);

    // encode the data stream:
    std::stringstream buf;
    osgDB::ReaderWriter::WriteResult wr;
    if ( _forceRGB && ImageUtils::hasAlphaChannel(image) )
    {
        osg::ref_ptr<osg::Image> rgb = ImageUtils::convertToRGB8(image);
        wr = _rw->writeImage(*(rgb.get()), buf, _dbOptions.get());
    }
    else
    {
        wr = _rw->writeImage(*image, buf, _dbOptions.get());
    }

    if ( wr.error() )
    {
        OE_WARN << LC << "Image encoding failed: " << wr.message() << std::endl;
        return false;
    }

    std::string value = buf.str();
    
    // compress if necessary:
    if ( _compressor.valid() )
    {
        std::ostringstream output;
        if ( !_compressor->compress(output, value) )
        {
            OE_WARN << LC << "Compressor failed" << std::endl;
            return false;
        }
        value = output.str();
    }

    int z = key.getLOD();
    int x = key.getTileX();
    int y = key.getTileY();

    // flip Y axis
    unsigned int numRows, numCols;
    key.getProfile()->getNumTiles(key.getLevelOfDetail(), numCols, numRows);
    y  = numRows - y - 1;

    // Prep the insert statement:
    sqlite3_stmt* insert = NULL;
    std::string query = "INSERT OR REPLACE INTO tiles (zoom_level, tile_column, tile_row, tile_data) VALUES (?, ?, ?, ?)";
    int rc = sqlite3_prepare_v2( _database, query.c_str(), -1, &insert, 0L );
    if ( rc != SQLITE_OK )
    {
        OE_WARN << LC << "Failed to prepare SQL: " << query << "; " << sqlite3_errmsg(_database) << std::endl;
        return false;
    }

    // bind parameters:
    sqlite3_bind_int( insert, 1, z );
    sqlite3_bind_int( insert, 2, x );
    sqlite3_bind_int( insert, 3, y );

    // bind the data blob:
    sqlite3_bind_blob( insert, 4, value.c_str(), value.length(), SQLITE_STATIC );

    // run the sql.
    bool ok = true;
    int tries = 0;
    do {
        rc = sqlite3_step(insert);
    }
    while (++tries < 100 && (rc == SQLITE_BUSY || rc == SQLITE_LOCKED));

    if (SQLITE_OK != rc && SQLITE_DONE != rc)
    {
#if SQLITE_VERSION_NUMBER >= 3007015
        OE_WARN << LC << "Failed query: " << query << "(" << rc << ")" << sqlite3_errstr(rc) << "; " << sqlite3_errmsg(_database) << std::endl;
#else
        OE_WARN << LC << "Failed query: " << query << "(" << rc << ")" << rc << "; " << sqlite3_errmsg(_database) << std::endl;
#endif        
        ok = false;
    }

    sqlite3_finalize( insert );

    return ok;
}

bool
MBTilesTileSource::getMetaData(const std::string& key, std::string& value)
{
    Threading::ScopedMutexLock exclusiveLock(_mutex);

    //get the metadata
    sqlite3_stmt* select = NULL;
    std::string query = "SELECT value from metadata where name = ?";
    int rc = sqlite3_prepare_v2( _database, query.c_str(), -1, &select, 0L );
    if ( rc != SQLITE_OK )
    {
        OE_WARN << LC << "Failed to prepare SQL: " << query << "; " << sqlite3_errmsg(_database) << std::endl;
        return false;
    }


    bool valid = true;
    std::string keyStr = std::string( key );
    rc = sqlite3_bind_text( select, 1, keyStr.c_str(), keyStr.length(), SQLITE_STATIC );
    if (rc != SQLITE_OK )
    {
        OE_WARN << LC << "Failed to bind text: " << query << "; " << sqlite3_errmsg(_database) << std::endl;
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
MBTilesTileSource::putMetaData(const std::string& key, const std::string& value)
{
    Threading::ScopedMutexLock exclusiveLock(_mutex);

    // prep the insert statement.
    sqlite3_stmt* insert = 0L;
    std::string query = Stringify() << "INSERT OR REPLACE INTO metadata (name,value) VALUES (?,?)";
    if ( SQLITE_OK != sqlite3_prepare_v2(_database, query.c_str(), -1, &insert, 0L) )
    {
        OE_WARN << LC << "Failed to prepare SQL: " << query << "; " << sqlite3_errmsg(_database) << std::endl;
        return false;
    }

    // bind the values:
    if( SQLITE_OK != sqlite3_bind_text(insert, 1, key.c_str(), key.length(), SQLITE_STATIC) )
    {
        OE_WARN << LC << "Failed to bind text: " << query << "; " << sqlite3_errmsg(_database) << std::endl;
        return false;
    }
    if ( SQLITE_OK != sqlite3_bind_text(insert, 2, value.c_str(), value.length(), SQLITE_STATIC) )
    {
        OE_WARN << LC << "Failed to bind text: " << query << "; " << sqlite3_errmsg(_database) << std::endl;
        return false;
    }

    // execute the sql. no idea what a good return value should be :/
    sqlite3_step( insert );
    sqlite3_finalize( insert );
    return true;
}

void
MBTilesTileSource::computeLevels()
{        
    Threading::ScopedMutexLock exclusiveLock(_mutex);

    osg::Timer_t startTime = osg::Timer::instance()->tick();
    sqlite3_stmt* select = NULL;
    std::string query = "SELECT min(zoom_level), max(zoom_level) from tiles";
    int rc = sqlite3_prepare_v2( _database, query.c_str(), -1, &select, 0L );
    if ( rc != SQLITE_OK )
    {
        OE_WARN << LC << "Failed to prepare SQL: " << query << "; " << sqlite3_errmsg(_database) << std::endl;
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
MBTilesTileSource::createTables()
{
    Threading::ScopedMutexLock exclusiveLock(_mutex);

    // https://github.com/mapbox/mbtiles-spec/blob/master/1.2/spec.md

    std::string query =
        "CREATE TABLE IF NOT EXISTS metadata ("
        " name  text,"
        " value text)";

    if (SQLITE_OK != sqlite3_exec(_database, query.c_str(), 0L, 0L, 0L))
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

    if (SQLITE_OK != sqlite3_exec(_database, query.c_str(), 0L, 0L, &errorMsg))
    {
        OE_WARN << LC << "Failed to create table [tiles]: " << errorMsg << std::endl;
        sqlite3_free( errorMsg );
        return false;
    }

    // create an index
    query =
        "CREATE UNIQUE INDEX tile_index ON tiles ("
        " zoom_level, tile_column, tile_row)";

    if (SQLITE_OK != sqlite3_exec(_database, query.c_str(), 0L, 0L, &errorMsg))
    {
        OE_WARN << LC << "Failed to create index on table [tiles]: " << errorMsg << std::endl;
        sqlite3_free( errorMsg );
        // keep going...
        // return false; 
    }

    // TODO: support "grids" and "grid_data" tables if necessary.

    return true;
}

std::string
MBTilesTileSource::getExtension() const 
{
    return _tileFormat;
}
