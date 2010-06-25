/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include "Sqlite3CacheOptions"

#include <osgEarth/FileUtils>
#include <osgEarth/TaskService>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#if OSG_MIN_VERSION_REQUIRED(2,9,5)
#  include <osgDB/Options>
#endif
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>
#include <cstring>
#include <fstream>

// for the compressor stuff
#if OSG_MIN_VERSION_REQUIRED(2,9,8)
#  define USE_SERIALIZERS
#  include <osgDB/Serializer>
#endif

#include <sqlite3.h>

using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace OpenThreads;

#define LC "[Sqlite3Cache] "

#define MAX_SIZE_TABLE 40 // in MB
#define USE_TRANSACTIONS
#define USE_L2_CACHE

#define SPLIT_DB_FILE
#define SPLIT_LAYER_DB
#define UPDATE_ACCESS_TIMES
#define UPDATE_ACCESS_TIMES_POOL
//#define MONITOR_THREAD_HEALTH

// --------------------------------------------------------------------------

// opens a database connection with default settings.
static
sqlite3* openDatabase( const std::string& path, bool serialized )
{
    sqlite3* db = 0L;

    // not sure if SHAREDCACHE is necessary or wise 
    int flags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE;
    flags |= serialized ? SQLITE_OPEN_FULLMUTEX : SQLITE_OPEN_NOMUTEX;

    int rc = sqlite3_open_v2( path.c_str(), &db, flags, 0L );

    if ( rc != 0 )
    {
        OE_WARN << LC << "Failed to open cache \"" << path << "\": " << sqlite3_errmsg(db) << std::endl;
        return 0L;
    }

    // make sure that writes actually finish
    sqlite3_busy_timeout( db, 60000 );

    return db;
}

// --------------------------------------------------------------------------

// a slightly customized Cache class that will support asynchronous writes
struct AsyncCache : public Cache
{
public:
    virtual void setImageSync(
        const TileKey* key,
        const std::string& layerName,
        const std::string& format,
        osg::Image* image ) =0;
};


// --------------------------------------------------------------------------

struct MetadataRecord
{
    std::string   _layerName;
    std::string   _format;
    int           _tileSize;
    osg::ref_ptr<const Profile> _profile;
    std::string   _compressor;
};

/**
 * This database table holds one record for each cached layer in the database.
 */
struct MetadataTable
{
    MetadataTable() { }

    bool initialize( sqlite3* db )
    {
        std::string sql =
            "CREATE TABLE IF NOT EXISTS metadata ("
            "layer varchar(255) PRIMARY KEY UNIQUE, "
            "format varchar(255), "
            "compressor varchar(64), "
            "tilesize int, "
            "srs varchar(1024), "
            "xmin double, "
            "ymin double, "
            "xmax double, "
            "ymax double, "
            "tw int, "
            "th int )";

        OE_INFO << LC << "SQL = " << sql << std::endl;

        char* errMsg = 0L;
        int err = sqlite3_exec( db, sql.c_str(), 0L, 0L, &errMsg );
        if ( err != SQLITE_OK )
        {
            OE_WARN << LC << "[Sqlite3Cache] Creating metadata: " << errMsg << std::endl;
            sqlite3_free( errMsg );
            return false;
        }

        // prep the insert/select statement SQL strings:
        _insertSQL = 
            "INSERT OR REPLACE INTO metadata "
            "(layer,format,compressor,tilesize,srs,xmin,ymin,xmax,ymax,tw,th) "
            "VALUES (?,?,?,?,?,?,?,?,?,?,?)";

        _selectSQL =
            "SELECT layer,format,compressor,tilesize,srs,xmin,ymin,xmax,ymax,tw,th "
            "FROM metadata WHERE layer = ?";

        _statsLoaded = 0;
        _statsStored = 0;
        return true;
    }

    bool store( const MetadataRecord& rec, sqlite3* db )
    {
        sqlite3_stmt* insert = 0;
        int rc = sqlite3_prepare_v2( db, _insertSQL.c_str(), _insertSQL.length(), &insert, 0L );
        if ( rc != SQLITE_OK )
        {
            OE_WARN 
                << LC << "Error preparing SQL: " 
                << sqlite3_errmsg( db )
                << "(SQL: " << _insertSQL << ")"
                << std::endl;
            return false;
        }

        sqlite3_bind_text( insert, 1, rec._layerName.c_str(), -1, 0L );
        sqlite3_bind_text( insert, 2, rec._format.c_str(), -1, 0L );
        sqlite3_bind_text( insert, 3, rec._compressor.c_str(), -1, 0L );
        sqlite3_bind_int ( insert, 4, rec._tileSize );
        sqlite3_bind_text( insert, 5, rec._profile->getSRS()->getInitString().c_str(), -1, 0L );
        sqlite3_bind_double( insert, 6, rec._profile->getExtent().xMin() );
        sqlite3_bind_double( insert, 7, rec._profile->getExtent().yMin() );
        sqlite3_bind_double( insert, 8, rec._profile->getExtent().xMax() );
        sqlite3_bind_double( insert, 9, rec._profile->getExtent().yMax() );
        unsigned int tw, th;
        rec._profile->getNumTiles( 0, tw, th );
        sqlite3_bind_int( insert, 10, tw );
        sqlite3_bind_int( insert, 11, th );

        bool success;

        rc = sqlite3_step(insert);
        if ( rc != SQLITE_DONE )
        {
            OE_WARN << LC << "SQL INSERT failed: " << sqlite3_errmsg( db )
                << "; SQL = " << _insertSQL
                << std::endl;
            success = false;
        }
        else
        {
            OE_INFO << LC << "Stored metadata record for \"" << rec._layerName << "\"" << std::endl;
            success = true;
        }

        sqlite3_finalize( insert );
        if (success)
            _statsStored++;
        return success;
    }

    bool load( const std::string& key, sqlite3* db, MetadataRecord& output )
    {
        bool success = true;

        sqlite3_stmt* select = 0L;
        int rc = sqlite3_prepare_v2( db, _selectSQL.c_str(), _selectSQL.length(), &select, 0L );
        if ( rc != SQLITE_OK )
        {
            OE_WARN 
                << LC << "Error preparing SQL: " 
                << sqlite3_errmsg( db )
                << "(SQL: " << _insertSQL << ")"
                << std::endl;
            return false;
        }

        sqlite3_bind_text( select, 1, key.c_str(), -1, 0L );

        rc = sqlite3_step( select );
        if ( rc == SQLITE_ROW ) 
        {
            // got a result            
            output._layerName = (char*)sqlite3_column_text( select, 0 );
            output._format = (char*)sqlite3_column_text( select, 1 );
            output._compressor = (char*)sqlite3_column_text( select, 2 );
            output._tileSize = sqlite3_column_int( select, 3 );
            ProfileConfig pconf;
            pconf.srsString() = (char*)sqlite3_column_text( select, 4 );
            pconf.bounds() = Bounds(
                sqlite3_column_double( select, 5 ),
                sqlite3_column_double( select, 6 ),
                sqlite3_column_double( select, 7 ),
                sqlite3_column_double( select, 8 ) );
            pconf.numTilesWideAtLod0() = sqlite3_column_int( select, 9 );
            pconf.numTilesHighAtLod0() = sqlite3_column_int( select, 10 );
            output._profile = Profile::create( pconf );
            success = true;
        }
        else
        {
            // no result
            OE_INFO << "NO metadata record found for \"" << key << "\"" << std::endl;
            success = false;
        }

        sqlite3_finalize( select );
        if (success)
            _statsLoaded++;
        return success;
    }

    bool remove( const std::string& key )
    {
        //TODO
        return false;
    }

    std::string _insertSQL;
    std::string _selectSQL;
    int _statsLoaded;
    int _statsStored;
 };

// --------------------------------------------------------------------------

struct ImageRecord
{
    osg::ref_ptr<const TileKey> _key;
    int _created;
    int _accessed;
    osg::ref_ptr<osg::Image> _image;
};

/**
 * Database table that holds one record per cached imagery tile in a layer. The layer name
 * is the table name.
 */
struct LayerTable : public osg::Referenced
{
    LayerTable( const MetadataRecord& meta, sqlite3* db )
        : _meta(meta)
    {
        // create the table and load the processors.
        if ( ! initialize( db ) )
        {
            return;
        }

        // initialize the SELECT statement for fetching records
        std::stringstream buf;
#ifdef SPLIT_DB_FILE
        buf << "SELECT created,accessed,size FROM \"" << _meta._layerName << "\" WHERE key = ?";
#else
        buf << "SELECT created,accessed,data FROM \"" << _meta._layerName << "\" WHERE key = ?";
#endif
        _selectSQL = buf.str();

        // initialize the UPDATE statement for updating the timestamp of an accessed record
        buf.str("");
        buf << "UPDATE \"" << _meta._layerName << "\" SET accessed = ? "
            << "WHERE key = ?";
        _updateTimeSQL = buf.str();

        buf.str("");
        buf << "UPDATE \"" << _meta._layerName << "\" SET accessed = ? "
            << "WHERE key in ( ? )";
        _updateTimePoolSQL = buf.str();
        
        // initialize the INSERT statement for writing records.
        buf.str("");
        buf << "INSERT OR REPLACE INTO \"" << _meta._layerName << "\" "
#ifdef SPLIT_DB_FILE
            << "(key,created,accessed,size) VALUES (?,?,?,?)";
#else
            << "(key,created,accessed,data) VALUES (?,?,?,?)";
#endif
        _insertSQL = buf.str();

        // initialize the DELETE statements for purging old records.
        buf.str("");
        buf << "DELETE FROM \"" << _meta._layerName << "\" "
            << "INDEXED BY \"" << _meta._layerName << "_lruindex\" "
            << "WHERE accessed < ?";
        _purgeSQL = buf.str();
        
        buf.str("");
        buf << "DELETE FROM \""  << _meta._layerName << "\" WHERE key in (SELECT key FROM \"" << _meta._layerName << "\" WHERE \"accessed\" < ? limit ?)";
        _purgeLimitSQL = buf.str();          

        buf.str("");
        buf << "SELECT key FROM \"" << _meta._layerName << "\" WHERE \"accessed\" < ? limit ?";
        _purgeSelect = buf.str();

        _statsLoaded = 0;
        _statsStored = 0;
    }


    int getTableSize(sqlite3* db) 
    {
#ifdef SPLIT_DB_FILE
        std::string query = "select sum(size) from \"" + _meta._layerName + "\";";
#else
        std::string query = "select sum(length(data)) from \"" + _meta._layerName + "\";";
#endif
        sqlite3_stmt* select = 0L;
        int rc = sqlite3_prepare_v2( db, query.c_str(), query.length(), &select, 0L );
        if ( rc != SQLITE_OK )
        {
            OE_WARN << LC << "Failed to prepare SQL: " << query << "; " << sqlite3_errmsg(db) << std::endl;
            return -1;
        }

        rc = sqlite3_step( select );
        if ( rc != SQLITE_ROW)
        {
            OE_WARN << LC << "SQL QUERY failed for " << query << ": " 
                << sqlite3_errmsg( db ) //<< "; tries=" << (1000-tries)
                << ", rc = " << rc << std::endl;
            sqlite3_finalize( select );
            return -1;
        }
        int size = sqlite3_column_int(select, 0);
        sqlite3_finalize( select );
        return size;
    }

    int getNbEntry(sqlite3* db) {
        std::string query = "select count(*) from \"" + _meta._layerName + "\";";
        sqlite3_stmt* select = 0L;
        int rc = sqlite3_prepare_v2( db, query.c_str(), query.length(), &select, 0L );
        if ( rc != SQLITE_OK )
        {
            OE_WARN << LC << "Failed to prepare SQL: " << query << "; " << sqlite3_errmsg(db) << std::endl;
            return -1;
        }

        rc = sqlite3_step( select );
        if ( rc != SQLITE_ROW)
        {
            OE_WARN << LC << "SQL QUERY failed for " << query << ": " 
                << sqlite3_errmsg( db ) //<< "; tries=" << (1000-tries)
                << ", rc = " << rc << std::endl;
            sqlite3_finalize( select );
            return -1;
        }
        int nbItems = sqlite3_column_int(select, 0);
        sqlite3_finalize( select );
        return nbItems;
    }

    void checkAndPurgeIfNeeded(sqlite3* db )
    {
        int maxSize = MAX_SIZE_TABLE * 1024 * 1024; // 40Mb for this table
        int size = getTableSize(db);
        if (size < 0 || size < 1.2 * maxSize)
            return;
            
        ::time_t t = ::time(0L);
        int nbElements = getNbEntry(db);
        float averageSize = size * 1.0 / nbElements;
        float diffSize = size - maxSize;
        int maxElementToRemove = static_cast<int>(ceil(diffSize/averageSize));
        OE_WARN << _meta._layerName <<  " : "  << size/1024/1024 << " MB " << " try to remove " << maxElementToRemove << " / " <<  nbElements << " from  " << _meta._layerName << std::endl;
        purge(t, maxElementToRemove, db);
    }

    bool store( const ImageRecord& rec, sqlite3* db )
    {
        displayStats();

        sqlite3_stmt* insert = 0L;
        int rc = sqlite3_prepare_v2( db, _insertSQL.c_str(), _insertSQL.length(), &insert, 0L );
        if ( rc != SQLITE_OK )
        {
            OE_WARN 
                << LC << "Error preparing SQL: " 
                << sqlite3_errmsg( db )
                << "(SQL: " << _insertSQL << ")"
                << std::endl;
            return false;
        }

        // bind the key string:
        std::string keyStr = rec._key->str();
        sqlite3_bind_text( insert, 1, keyStr.c_str(), keyStr.length(), SQLITE_STATIC );
        sqlite3_bind_int(  insert, 2, rec._created );
        sqlite3_bind_int(  insert, 3, rec._accessed );

        // serialize the image:
#ifdef SPLIT_DB_FILE
        std::stringstream outStream;
        _rw->writeImage( *rec._image.get(), outStream, _rwOptions.get() );
        std::string outBuf = outStream.str();
        std::string fname = _meta._layerName + "_" + keyStr+".osgb";
        {
            std::ofstream file(fname.c_str(), std::ios::out | std::ios::binary);
            if (file.is_open()) {
                file.write(outBuf.c_str(), outBuf.length());
            }
        }
        sqlite3_bind_int( insert, 4, outBuf.length() );
#else
        std::stringstream outStream;
        _rw->writeImage( *rec._image.get(), outStream, _rwOptions.get() );
        std::string outBuf = outStream.str();
        sqlite3_bind_blob( insert, 4, outBuf.c_str(), outBuf.length(), SQLITE_STATIC );
#endif

        // write to the database:
        rc = sqlite3_step( insert );

        if ( rc != SQLITE_DONE )
        {
            OE_WARN << LC << "SQL INSERT failed for key " << rec._key->str() << ": " 
                << sqlite3_errmsg( db ) //<< "; tries=" << (1000-tries)
                << ", rc = " << rc << std::endl;
            sqlite3_finalize( insert );
            return false;
        }
        else
        {
            OE_DEBUG << LC << "cache INSERT tile " << rec._key->str() << std::endl;
            sqlite3_finalize( insert );
            _statsStored++;
            return true;
        }
    }

    bool updateAccessTime( const TileKey* key, int newTimestamp, sqlite3* db )
    { 
        sqlite3_stmt* update = 0L;
        int rc = sqlite3_prepare_v2( db, _updateTimeSQL.c_str(), _updateTimeSQL.length(), &update, 0L );
        if ( rc != SQLITE_OK )
        {
            OE_WARN << LC << "Failed to prepare SQL " << _updateTimeSQL << "; " << sqlite3_errmsg(db) << std::endl;
            return false;
        }

        bool success = true;
        sqlite3_bind_int( update, 1, newTimestamp );
        std::string keyStr = key->str();
        sqlite3_bind_text( update, 2, keyStr.c_str(), keyStr.length(), SQLITE_STATIC );
        rc = sqlite3_step( update );
        if ( rc != SQLITE_DONE )
        {
            OE_WARN << LC << "Failed to update timestamp for " << key->str() << " on layer " << _meta._layerName << " rc = " << rc << std::endl;
            success = false;
        }

        sqlite3_finalize( update );
        return success;
    }

    bool updateAccessTimePool( const std::string&  keyStr, int newTimestamp, sqlite3* db )
    {
        //OE_WARN << LC << "update access times " << _meta._layerName << " " << keyStr << std::endl;
        sqlite3_stmt* update = 0L;
        int rc = sqlite3_prepare_v2( db, _updateTimePoolSQL.c_str(), _updateTimePoolSQL.length(), &update, 0L );
        if ( rc != SQLITE_OK )
        {
            OE_WARN << LC << "Failed to prepare SQL " << _updateTimePoolSQL << "; " << sqlite3_errmsg(db) << std::endl;
            return false;
        }

        bool success = true;
        sqlite3_bind_int( update, 1, newTimestamp );
        sqlite3_bind_text( update, 2, keyStr.c_str(), keyStr.length(), SQLITE_STATIC );
        rc = sqlite3_step( update );
        if ( rc != SQLITE_DONE )
        {
            OE_WARN << LC << "Failed to update timestamp for " << keyStr << " on layer " << _meta._layerName << " rc = " << rc << std::endl;
            success = false;
        }

        sqlite3_finalize( update );
        return success;
    }

    bool load( const TileKey* key, ImageRecord& output, sqlite3* db )
    {
        displayStats();
        int imageBufLen = 0;
        
        sqlite3_stmt* select = 0L;
        int rc = sqlite3_prepare_v2( db, _selectSQL.c_str(), _selectSQL.length(), &select, 0L );
        if ( rc != SQLITE_OK )
        {
            OE_WARN << LC << "Failed to prepare SQL: " << _selectSQL << "; " << sqlite3_errmsg(db) << std::endl;
            return false;
        }

        std::string keyStr = key->str();
        sqlite3_bind_text( select, 1, keyStr.c_str(), keyStr.length(), SQLITE_STATIC );

        rc = sqlite3_step( select );
        if ( rc != SQLITE_ROW ) // == SQLITE_DONE ) // SQLITE_DONE means "no more rows"
        {
            // cache miss
            OE_DEBUG << LC << "Cache MISS on tile " << key->str() << std::endl;
            sqlite3_finalize(select);
            return false;
        }

        // copy the timestamps:
        output._created  = sqlite3_column_int( select, 0 );
        output._accessed = sqlite3_column_int( select, 1 );

#ifdef SPLIT_DB_FILE
        std::string fname(keyStr);
        osgDB::ReaderWriter::ReadResult rr = _rw->readImage( _meta._layerName + "_" +fname+".osgb" );
#else
        // the pointer returned from _blob gets freed internally by sqlite, supposedly
        const char* data = (const char*)sqlite3_column_blob( select, 2 );
        imageBufLen = sqlite3_column_bytes( select, 2 );

        // deserialize the image from the buffer:
        std::string imageString( data, imageBufLen );
        std::stringstream imageBufStream( imageString );
        osgDB::ReaderWriter::ReadResult rr = _rw->readImage( imageBufStream );
#endif
        if ( rr.error() )
        {
            OE_WARN << LC << "Failed to read image from database: " << rr.message() << std::endl;
        }
        else
        {
            output._image = rr.takeImage();
            output._key = key;
            OE_DEBUG << LC << "Cache HIT on tile " << key->str() << std::endl;
        }

        sqlite3_finalize(select);

        _statsLoaded++;
        return output._image.valid();
    }

    void displayStats()
    {
        osg::Timer_t t = osg::Timer::instance()->tick();
        if (osg::Timer::instance()->delta_s( _statsLastCheck, t) > 10.0) {
            double d = osg::Timer::instance()->delta_s(_statsStartTimer, t);
            OE_WARN << _meta._layerName << " time " << d << " stored " << _statsStored << " rate " << _statsStored * 1.0 / d << std::endl;
            OE_WARN << _meta._layerName << " time " << d << " loaded " << _statsLoaded << " rate " << _statsLoaded * 1.0 / d << std::endl;
            _statsLastCheck = t;
        }
    }

    bool remove( const std::string& key, sqlite3* db )
    {
        //TODO
        return false;
    }

    bool purge( int utcTimeStamp, int maxToRemove, sqlite3* db )
    {
        if ( maxToRemove < 0 )
            return false;

        sqlite3_stmt* purge = 0L;
        
        int rc;
#if 0
        if ( maxToRemove < 0 )
        {
            rc = sqlite3_prepare_v2( db, _purgeSQL.c_str(), _purgeSQL.length(), &purge, 0L );
            if ( rc != SQLITE_OK )
            {
                OE_WARN << LC << "Failed to prepare SQL: " << _purgeSQL << "; " << sqlite3_errmsg(db) << std::endl;
                return false;
            }
        }
        else
#endif
        {
#ifdef SPLIT_DB_FILE
            {
                std::vector<std::string> deleteFiles;
                sqlite3_stmt* selectPurge = 0L;
                rc = sqlite3_prepare_v2( db, _purgeSelect.c_str(), _purgeSelect.length(), &selectPurge, 0L);
                if ( rc != SQLITE_OK )
                {
                    OE_WARN << LC << "Failed to prepare SQL: " << _purgeSelect << "; " << sqlite3_errmsg(db) << std::endl;
                    return false;
                }
                sqlite3_bind_int( selectPurge, 2, maxToRemove );
                sqlite3_bind_int( selectPurge, 1, utcTimeStamp );

                rc = sqlite3_step( selectPurge );
                if ( rc != SQLITE_ROW && rc != SQLITE_DONE)
                {
                    OE_WARN << LC << "SQL QUERY failed for " << _purgeSelect << ": " 
                            << sqlite3_errmsg( db ) //<< "; tries=" << (1000-tries)
                            << ", rc = " << rc << std::endl;
                    sqlite3_finalize( selectPurge );
                    return false;
                }
                while (rc == SQLITE_ROW) {
                    std::string f((const char*)sqlite3_column_text( selectPurge, 0 ));
                    deleteFiles.push_back(f);
                    rc = sqlite3_step( selectPurge );
                }
                if (rc != SQLITE_DONE) {
                    OE_WARN << LC << "SQL QUERY failed for " << _purgeSelect << ": " 
                            << sqlite3_errmsg( db ) //<< "; tries=" << (1000-tries)
                            << ", rc = " << rc << std::endl;
                    sqlite3_finalize( selectPurge );
                    return false;
                }
                sqlite3_finalize( selectPurge );
                while (!deleteFiles.empty()) {
                    std::string fname = _meta._layerName + "_" + deleteFiles.back() +".osgb";
                    int run = unlink(fname.c_str());
                    if (run) {
                        OE_WARN << "Error while removing file " << fname << std::endl;
                    }
                    deleteFiles.pop_back();
                }
            }
#endif
            rc = sqlite3_prepare_v2( db, _purgeLimitSQL.c_str(), _purgeLimitSQL.length(), &purge, 0L );
            if ( rc != SQLITE_OK )
            {
                OE_WARN << LC << "Failed to prepare SQL: " << _purgeLimitSQL << "; " << sqlite3_errmsg(db) << std::endl;
                return false;
            }
            sqlite3_bind_int( purge, 2, maxToRemove );
        }

        sqlite3_bind_int( purge, 1, utcTimeStamp );

        rc = sqlite3_step( purge );
        if ( rc != SQLITE_DONE )
        {
            // cache miss
            OE_DEBUG << LC << "Error purging records from \"" << _meta._layerName << "\"; " << sqlite3_errmsg(db) << std::endl;
            sqlite3_finalize(purge);
            return false;
        }

        sqlite3_finalize(purge);
        return true;
    }

    /** Initializes the layer by creating the layer's table (if necessary), loading the
        appropriate reader-writer for the image data, and initializing the compressor
        if necessary. */
    bool initialize( sqlite3* db )
    {
        // first create the table if it does not already exist:
        std::stringstream buf;
        buf << "CREATE TABLE IF NOT EXISTS \"" << _meta._layerName << "\" ("
            << "key char(64) PRIMARY KEY UNIQUE, "
            << "created int, "
            << "accessed int, "
#ifdef SPLIT_DB_FILE
            << "size int )";
#else
            << "data blob )";
#endif
        std::string sql = buf.str();

        OE_INFO << LC << "SQL = " << sql << std::endl;

        char* errMsg = 0L;
        int rc = sqlite3_exec( db, sql.c_str(), 0L, 0L, &errMsg );
        if ( rc != SQLITE_OK )
        {
            OE_WARN << LC << "Creating layer \"" << _meta._layerName << "\": " << errMsg << std::endl;
            sqlite3_free( errMsg );
            return false;
        }

        // create an index on the time-last-accessed column
        buf.str("");
        buf << "CREATE INDEX IF NOT EXISTS \"" 
            << _meta._layerName << "_lruindex\" "
            << "ON \"" << _meta._layerName << "\" (accessed)";
        sql = buf.str();

        OE_INFO << LC << "SQL = " << sql << std::endl;

        rc = sqlite3_exec( db, sql.c_str(), 0L, 0L, &errMsg );
        if ( rc != SQLITE_OK )
        {
            OE_WARN << LC << "Creating index for layer \"" << _meta._layerName << "\": " << errMsg << std::endl;
            sqlite3_free( errMsg );
            //return false;
        }

        // next load the appropriate ReaderWriter:

#if OSG_MIN_VERSION_REQUIRED(2,9,5)
        _rw = osgDB::Registry::instance()->getReaderWriterForMimeType( _meta._format );
        if ( !_rw.valid() )
#endif
            _rw = osgDB::Registry::instance()->getReaderWriterForExtension( _meta._format );
        if ( !_rw.valid() )
        {
            OE_WARN << LC << "Creating layer: Cannot initialize ReaderWriter for format \"" 
                << _meta._format << "\"" << std::endl;
            return false;
        }

        if ( !_meta._compressor.empty() )
            _rwOptions = new osgDB::ReaderWriter::Options( "Compressor=" + _meta._compressor );

        _statsLastCheck = _statsStartTimer = osg::Timer::instance()->tick();
        return true;
    }

    std::string _selectSQL;
    std::string _insertSQL;
    std::string _updateTimeSQL;
    std::string _updateTimePoolSQL;
 
    std::string _purgeSelect;
    std::string _purgeSQL;
    std::string _purgeLimitSQL;
    MetadataRecord _meta;

    osg::ref_ptr<osgDB::ReaderWriter> _rw;
    osg::ref_ptr<osgDB::ReaderWriter::Options> _rwOptions;

    osg::Timer_t _statsStartTimer;
    osg::Timer_t _statsLastCheck;
    int _statsLoaded;
    int _statsStored;

};

// --------------------------------------------------------------------------

typedef std::map<std::string,osg::ref_ptr<LayerTable> > LayerTablesByName;

// --------------------------------------------------------------------------

//TODO: might want to move this up out of this plugin at some point.

struct AsyncPurge : public TaskRequest {
    AsyncPurge( const std::string& layerName, int olderThanUTC, Cache* cache )
        : _layerName(layerName), _olderThanUTC(olderThanUTC), _cache(cache) { }

    void operator()( ProgressCallback* progress ) { 
        osg::ref_ptr<Cache> cache = _cache.get();
        if ( cache.valid() )
            cache->purge( _layerName, _olderThanUTC, false );
    }

    std::string _layerName;
    int _olderThanUTC;
    osg::observer_ptr<Cache> _cache;
};

struct AsyncInsert : public TaskRequest {
    AsyncInsert( const TileKey* key, const std::string& layerName, const std::string& format, osg::Image* image, AsyncCache* cache )
        : _key(key), _layerName(layerName), _format(format), _image(image), _cache(cache) { }

    void operator()( ProgressCallback* progress ) {
        osg::ref_ptr<AsyncCache> cache = _cache.get();
        if ( cache.valid() )
            cache->setImageSync( _key.get(), _layerName, _format, _image.get() );
    }

    std::string _layerName, _format;
    osg::ref_ptr<const TileKey> _key;
    osg::ref_ptr<osg::Image> _image;
    osg::observer_ptr<AsyncCache> _cache;
};

class Sqlite3Cache;
struct AsyncUpdateAccessTime : public TaskRequest {
    AsyncUpdateAccessTime( const TileKey* key, const std::string& layerName, int timeStamp, Sqlite3Cache* cache );
    void operator()( ProgressCallback* progress );

    osg::ref_ptr<const TileKey> _key;
    std::string _layerName;
    int _timeStamp;
    osg::observer_ptr<Sqlite3Cache> _cache;
};



struct AsyncUpdateAccessTimePool : public TaskRequest {
    AsyncUpdateAccessTimePool( const std::string& layerName, Sqlite3Cache* cache );
    void addEntry(const TileKey* key, int timeStamp);
    void operator()( ProgressCallback* progress );
    const std::string& getLayerName() { return _layerName; }
    int getNbEntry() const { return _keys.size(); }
    std::map<std::string, int> _keys;
    std::string _layerName;
    std::string _keyStr;
    int _timeStamp;
    osg::observer_ptr<Sqlite3Cache> _cache;
};

// --------------------------------------------------------------------------

struct ThreadTable {
    ThreadTable(LayerTable* table, sqlite3* db) : _table(table), _db(db) { }
    LayerTable* _table;
    sqlite3* _db;
};

class Sqlite3Cache : public AsyncCache
{
public:

    Sqlite3Cache( const PluginOptions* options ) : AsyncCache(), _db(0L)
    {
        _nbRequest = 0;
        _settings = dynamic_cast<const Sqlite3CacheOptions*>( options );
        if ( !_settings.valid() )
            _settings = new Sqlite3CacheOptions( options );

        OE_INFO << LC << "settings: " << options->config().toString() << std::endl;

        if ( sqlite3_threadsafe() == 0 )
        {
            OE_WARN << LC << "SQLITE3 IS NOT COMPILED IN THREAD-SAFE MODE" << std::endl;
            // TODO: something in this unlikely condition
        }

        // enabled shared cache mode.
        //sqlite3_enable_shared_cache( 1 );

#ifdef USE_L2_CACHE
        _L2cache = new MemCache();
        _L2cache->setMaxNumTilesInCache( 64 );
        OE_INFO << LC << "Using L2 memory cache" << std::endl;
#endif

        _db = openDatabase( _settings->path().value(), _settings->serialized().value() );

        if ( _db )
        {
            if ( ! _metadata.initialize( _db ) )
                _db = 0L;
        }

        if ( _db && _settings->asyncWrites() == true )
        {
            _writeService = new osgEarth::TaskService( "Sqlite3Cache Write Service", 1 );
        }
    }

    // just here to satisfy the osg::Object requirements
    Sqlite3Cache() { }
    Sqlite3Cache( const Sqlite3Cache& rhs, const osg::CopyOp& op ) { }
    META_Object(osgEarth,Sqlite3Cache);

public: // Cache interface
    
    /**
     * Gets whether the given TileKey is cached or not
     */
    bool isCached( const TileKey* key, const std::string& layerName, const std::string& format ) const
    {
        // this looks ineffecient, but usually when isCached() is called, getImage() will be
        // called soon thereafter. And this call will load it into the L2 cache so the subsequent
        // getImage call will not hit the DB again.
        osg::ref_ptr<osg::Image> temp = const_cast<Sqlite3Cache*>(this)->getImage( key, layerName, format );
        return temp.valid();
    }

    /**
     * Store the cache profile for the given profile.
     */
    void storeLayerProperties(
        const std::string& layerName, const Profile* profile,
        const std::string& format, unsigned int tileSize)
    {
        if ( !_db ) return;

        if ( layerName.empty() || profile == 0L || format.empty() )
        {
            OE_WARN << "ILLEGAL: cannot cache a layer without a layer name" << std::endl;
            return;
        }

        ScopedLock<Mutex> lock( _tableListMutex ); // b/c we're using the base db handle
#ifdef SPLIT_LAYER_DB
        sqlite3* db = getOrCreateMetaDbForThread();
#else
        sqlite3* db = getOrCreateDbForThread();
#endif
        if ( !db )
            return;

        //OE_INFO << "Storing metadata for layer \"" << layerName << "\"" << std::endl;

        MetadataRecord rec;
        rec._layerName = layerName;
        rec._profile = profile;
        rec._tileSize = tileSize;

#ifdef USE_SERIALIZERS
        rec._format = "osgb";
        rec._compressor = "zlib";
#else
        rec._format = format;
#endif

        _metadata.store( rec, db );
    }

    /**
     * Loads the cache profile for the given layer.
     */
    const Profile* loadLayerProperties(
        const std::string& layerName,
        std::string& out_format,
        unsigned int& out_tileSize )
    {
        if ( !_db ) return 0L;

        ScopedLock<Mutex> lock( _tableListMutex ); // b/c we're using the base db handle

#ifdef SPLIT_LAYER_DB
        sqlite3* db = getOrCreateMetaDbForThread();
#else
        sqlite3* db = getOrCreateDbForThread();
#endif
        if ( !db )
            return 0L;

        OE_INFO << LC << "Loading metadata for layer \"" << layerName << "\"" << std::endl;

        MetadataRecord rec;
        if ( _metadata.load( layerName, db, rec ) )
        {
            out_format = rec._format;
            out_tileSize = rec._tileSize;
            return rec._profile.release();
        }
        return 0L;
    }

    /**
     * Gets the cached image for the given TileKey
     */
    osg::Image* getImage( const TileKey* key,  const std::string& layerName, const std::string& format )
    {
        if ( !_db ) return 0L;


        // first try the L2 cache.
        if ( _L2cache.valid() )
        {
            osg::Image* result = _L2cache->getImage( key, layerName, format );
            if ( result )
                return result;
        }

        if (_nbRequest > 100) {
            int t = (int)::time(0L);
            purge(layerName, t, _settings->asyncWrites().value() );
            _nbRequest = 0;
        }
        _nbRequest++;

        // next check the deferred-write queue.
        if ( _settings->asyncWrites() == true )
        {
            ScopedLock<Mutex> lock( _pendingWritesMutex );
            std::string name = key->str() + layerName;
            std::map<std::string,osg::ref_ptr<AsyncInsert> >::iterator i = _pendingWrites.find(name);
            if ( i != _pendingWrites.end() )
            {
                // todo: update the access time, or let it slide?
                OE_DEBUG << LC << "Got key that is write-queued: " << key->str() << std::endl;
                return i->second->_image.get();
            }
        }

        // finally, try to query the database.
        ThreadTable tt = getTable(layerName);
        if ( tt._table )
        {
            ImageRecord rec;
            if (!tt._table->load( key, rec, tt._db ))
                return 0;

            // load it into the L2 cache
            osg::Image* result = rec._image.release();

            if ( result && _L2cache.valid() )
                _L2cache->setImage( key, layerName, format, result );

#ifdef UPDATE_ACCESS_TIMES

#ifdef UPDATE_ACCESS_TIMES_POOL
            // update the last-access time
            int t = (int)::time(0L);
            {
                ScopedLock<Mutex> lock( _pendingUpdateMutex );
                osg::ref_ptr<AsyncUpdateAccessTimePool> pool;
                std::map<std::string,osg::ref_ptr<AsyncUpdateAccessTimePool> >::iterator i = _pendingUpdates.find(layerName);
                if ( i != _pendingUpdates.end() )
                {
                    i->second->addEntry(key, t);
                    pool = i->second;
                    OE_DEBUG << LC << "Add key " << key << " to existing layer batch " << layerName << std::endl;
                } else {
                    pool = new AsyncUpdateAccessTimePool(layerName, this);
                    pool->addEntry(key, t);
                    _pendingUpdates[layerName] = pool.get();
                }
                if (pool.valid()) {
                    if (pool->getNbEntry() > 100) {
                        _writeService->add(pool.get());
                        _pendingUpdates.erase(layerName);
                    }
                }
            }
#else
            // update the last-access time
            int t = (int)::time(0L);
            _writeService->add( new AsyncUpdateAccessTime(  key, layerName, t, this ) );
#endif

#endif // UPDATE_ACCESS_TIMES

            return result;
        }
        else
        {
            OE_WARN << LC << "What, no layer table?" << std::endl;
        }
        return 0L;
    }

    /**
     * Sets the cached image for the given TileKey
     */
    void setImage( const TileKey* key, const std::string& layerName, const std::string& format, osg::Image* image )
    {        
        if ( !_db ) return;

        if ( _settings->asyncWrites() == true )
        {
            // the "pending writes" table is here so that we don't try to write data to
            // the cache more than once when using an asynchronous write service.
            ScopedLock<Mutex> lock( _pendingWritesMutex );
            std::string name = key->str() + layerName;
            if ( _pendingWrites.find(name) == _pendingWrites.end() )
            {
                AsyncInsert* req = new AsyncInsert(key, layerName, format, image, this);
                _pendingWrites[name] = req;
                _writeService->add( req );
            }
            else
            {
                //NOTE: this should probably never happen.
                OE_WARN << LC << "Tried to setImage; already in queue: " << key->str() << std::endl;
            }
        }
        else
        {
            setImageSync( key, layerName, format, image );
        }
    }

    /**
     * Purges records from the database.
     */
    bool purge( const std::string& layerName, int olderThanUTC, bool async )
    {
        if ( !_db ) false;

        // purge the L2 cache first:
        if ( _L2cache.valid() )
            _L2cache->purge( layerName, olderThanUTC, async );

        if ( async == true && _settings->asyncWrites() == true )
        {
            _writeService->add( new AsyncPurge(layerName, olderThanUTC, this) );
        }
        else
        {
            ThreadTable tt = getTable( layerName );
            if ( tt._table )
            {
                tt._table->checkAndPurgeIfNeeded(tt._db );
            }
        }
        return true;
    }

    /**
     * updateAccessTime records on the database.
     */
    bool updateAccessTimeSync( const std::string& layerName, const TileKey* key, int newTimestamp )
    {
        if ( !_db ) false;

        ThreadTable tt = getTable(layerName);
        if ( tt._table )
        {
            tt._table->updateAccessTime( key, newTimestamp, tt._db );
        }
        return true;
    }

    /**
     * updateAccessTime records on the database.
     */
    bool updateAccessTimeSyncPool( const std::string& layerName, const std::string& keys, int newTimestamp )
    {
        if ( !_db ) false;

        ThreadTable tt = getTable(layerName);
        if ( tt._table )
        {
            tt._table->updateAccessTimePool( keys, newTimestamp, tt._db );
        }

        {
            //ScopedLock<Mutex> lock( _pendingUpdateMutex );
            //std::string name = layerName;
            //_pendingUpdates.erase( name );
        }
        return true;
    }

private:

    void setImageSync( const TileKey* key, const std::string& layerName, const std::string& format, osg::Image* image )
    {
        ThreadTable tt = getTable(layerName);
        if ( tt._table )
        {
            ::time_t t = ::time(0L);
            ImageRecord rec;
            rec._key = key;
            rec._created = (int)t;
            rec._accessed = (int)t;
            rec._image = image;

            tt._table->store( rec, tt._db );
        }

        if ( _settings->asyncWrites() == true )
        {
            ScopedLock<Mutex> lock( _pendingWritesMutex );
            std::string name = key->str() + layerName;
            _pendingWrites.erase( name );

            if ( _writeService->getNumRequests() == 0 )
            {
                //OE_DEBUG << "Write service queue is empty." << std::endl;
            }
            OE_INFO << LC << "Pending writes: " << std::dec << _writeService->getNumRequests() << std::endl;
        }            
    }

#ifdef SPLIT_LAYER_DB
    sqlite3* getOrCreateDbForThread(const std::string& layer)
    {
        sqlite3* db = 0L;

        // this method assumes the thread already holds a lock on _tableListMutex, which
        // doubles to protect _dbPerThread

        Thread* thread = Thread::CurrentThread();
        std::map<Thread*,sqlite3*>::const_iterator k = _dbPerThreadLayers[layer].find(thread);
        if ( k == _dbPerThreadLayers[layer].end() )
        {
            db = openDatabase( layer + _settings->path().value(), _settings->serialized().value() );
            if ( db )
            {
                _dbPerThreadLayers[layer][thread] = db;
                OE_INFO << LC << "Created DB handle " << std::hex << db << " for thread " << thread << std::endl;
            }
            else
            {
                OE_WARN << LC << "Failed to open DB on thread " << thread << std::endl;
            }
        }
        else
        {
            db = k->second;
        }

        return db;
    }


    sqlite3* getOrCreateMetaDbForThread()
    {
        sqlite3* db = 0L;

        // this method assumes the thread already holds a lock on _tableListMutex, which
        // doubles to protect _dbPerThread

        Thread* thread = Thread::CurrentThread();
        std::map<Thread*,sqlite3*>::const_iterator k = _dbPerThreadMeta.find(thread);
        if ( k == _dbPerThreadMeta.end() )
        {
            db = openDatabase( _settings->path().value(), _settings->serialized().value() );
            if ( db )
            {
                _dbPerThreadMeta[thread] = db;
                OE_INFO << LC << "Created DB handle " << std::hex << db << " for thread " << thread << std::endl;
            }
            else
            {
                OE_WARN << LC << "Failed to open DB on thread " << thread << std::endl;
            }
        }
        else
        {
            db = k->second;
        }

        return db;
    }

#else
    sqlite3* getOrCreateDbForThread()
    {
        sqlite3* db = 0L;

        // this method assumes the thread already holds a lock on _tableListMutex, which
        // doubles to protect _dbPerThread

        Thread* thread = Thread::CurrentThread();
        std::map<Thread*,sqlite3*>::const_iterator k = _dbPerThread.find(thread);
        if ( k == _dbPerThread.end() )
        {
            db = openDatabase( _settings->path().value(), _settings->serialized().value() );
            if ( db )
            {
                _dbPerThread[thread] = db;
                OE_INFO << LC << "Created DB handle " << std::hex << db << " for thread " << thread << std::endl;
            }
            else
            {
                OE_WARN << LC << "Failed to open DB on thread " << thread << std::endl;
            }
        }
        else
        {
            db = k->second;
        }

        return db;
    }
#endif

    // gets the layer table for the specified layer name, creating it if it does
    // not already exist...
    ThreadTable getTable( const std::string& layerName )
    {
        ScopedLock<Mutex> lock( _tableListMutex );

#ifdef SPLIT_LAYER_DB
        sqlite3* db = getOrCreateDbForThread(layerName);
#else
        sqlite3* db = getOrCreateDbForThread();
#endif
        if ( !db )
            return ThreadTable( 0L, 0L );

        LayerTablesByName::iterator i = _tables.find(layerName);
        if ( i == _tables.end() )
        {
            MetadataRecord meta;
#ifdef SPLIT_LAYER_DB
            sqlite3* metadb = getOrCreateMetaDbForThread();
            if ( !_metadata.load( layerName, metadb, meta ) )
#else
            if ( !_metadata.load( layerName, db, meta ) )
#endif
            {
                OE_WARN << LC << "Cannot operate on \"" << layerName << "\" because metadata does not exist."
                    << std::endl;
                return ThreadTable( 0L, 0L );
            }

            _tables[layerName] = new LayerTable( meta, db );
            OE_INFO << LC << "New LayerTable for " << layerName << std::endl;
        }
        return ThreadTable( _tables[layerName].get(), db );
    }

private:

    osg::ref_ptr<const Sqlite3CacheOptions> _settings;
    osg::ref_ptr<osgDB::ReaderWriter> _defaultRW;
    Mutex             _tableListMutex;
    MetadataTable     _metadata;
    LayerTablesByName _tables;

    bool _useAsyncWrites;
    osg::ref_ptr<TaskService> _writeService;
    Mutex _pendingWritesMutex;

    std::map<std::string, osg::ref_ptr<AsyncInsert> > _pendingWrites;

    Mutex _pendingUpdateMutex;
    std::map<std::string, osg::ref_ptr<AsyncUpdateAccessTimePool> > _pendingUpdates;

    sqlite3* _db;
    std::map<Thread*,sqlite3*> _dbPerThread;

    std::map<std::string, std::map<Thread*,sqlite3*> > _dbPerThreadLayers;
    std::map<Thread*,sqlite3*> _dbPerThreadMeta;

    osg::ref_ptr<MemCache> _L2cache;

    int _count;
    int _nbRequest;
};



AsyncUpdateAccessTime::AsyncUpdateAccessTime( const TileKey* key, const std::string& layerName, int timeStamp, Sqlite3Cache* cache ) : _key(key), _layerName(layerName), _timeStamp(timeStamp), _cache(cache) { }

void AsyncUpdateAccessTime::operator()( ProgressCallback* progress ) 
{ 
    osg::ref_ptr<Sqlite3Cache> cache = _cache.get();
    if ( cache.valid() ) {
        //OE_WARN << "AsyncUpdateAccessTime will process " << _key << std::endl;
        cache->updateAccessTimeSync( _layerName, _key.get() , _timeStamp );
    }
}


AsyncUpdateAccessTimePool::AsyncUpdateAccessTimePool(const std::string& layerName, Sqlite3Cache* cache) : _layerName(layerName), _cache(cache) {}
void AsyncUpdateAccessTimePool::addEntry(const TileKey* key, int timeStamp)
{
    const std::string& keyStr = key->str();
    if (_keys.find(keyStr) == _keys.end()) {
        _keys[keyStr] = 1;
        if (_keyStr.empty())
            _keyStr = keyStr;
        else
            _keyStr += ", " + keyStr;
    } else {
        //OE_WARN << "key " << keyStr << " already in batch" << std::endl;
    }
    _timeStamp = timeStamp;
}
void AsyncUpdateAccessTimePool::operator()( ProgressCallback* progress ) 
{ 
    osg::ref_ptr<Sqlite3Cache> cache = _cache.get();
    if ( cache.valid() ) {
        OE_WARN << "AsyncUpdateAccessTimePool will process " << _keys.size() << std::endl;
        cache->updateAccessTimeSyncPool( _layerName, _keyStr , _timeStamp );
    }
}


/**
 * This driver defers loading of the source data to the appropriate OSG plugin. You
 * must explicity set an override profile when using this driver.
 *
 * For example, use this driver to load a simple jpeg file; then set the profile to
 * tell osgEarth its projection.
 */
class Sqlite3CacheFactory : public osgDB::ReaderWriter
{
public:
    Sqlite3CacheFactory()
    {
        supportsExtension( "osgearth_cache_sqlite3", "Sqlite3 Cache for osgEarth" );
    }

    virtual const char* className()
    {
        return "Sqlite3 Cache for osgEarth";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return ReadResult( new Sqlite3Cache( static_cast<const PluginOptions*>(options) ) );
    }
};

REGISTER_OSGPLUGIN(osgearth_cache_sqlite3, Sqlite3CacheFactory)

