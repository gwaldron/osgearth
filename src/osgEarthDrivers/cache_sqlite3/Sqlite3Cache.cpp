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
#include <osgDB/FileUtils>
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

#define USE_TRANSACTIONS
#define USE_L2_CACHE

//#define SPLIT_DB_FILE
//#define SPLIT_LAYER_DB
#define UPDATE_ACCESS_TIMES
#define UPDATE_ACCESS_TIMES_POOL
#define MAX_REQUEST_TO_RUN_PURGE 100

#define PURGE_GENERAL
//#define INSERT_POOL

//#define MONITOR_THREAD_HEALTH

// --------------------------------------------------------------------------

// opens a database connection with default settings.
static
sqlite3* openDatabase( const std::string& path, bool serialized )
{
    //Try to create the path if it doesn't exist
    std::string dirPath = osgDB::getFilePath(path);    

    //If the path doesn't currently exist or we can't create the path, don't cache the file
    if (!osgDB::fileExists(dirPath) && !osgDB::makeDirectory(dirPath))
    {
        OE_WARN << LC << "Couldn't create path " << dirPath << std::endl;
    }

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
    AsyncCache(const CacheOptions& options =CacheOptions()): Cache(options) { }
    virtual void setImageSync(
        const TileKey& key,
        const CacheSpec& spec,
        const osg::Image* image ) =0;
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

        OE_DEBUG << LC << "SQL = " << sql << std::endl;

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
            OE_DEBUG << LC << "Stored metadata record for \"" << rec._layerName << "\"" << std::endl;
            success = true;
        }

        sqlite3_finalize( insert );
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
            ProfileOptions pconf;
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
            OE_DEBUG << "NO metadata record found for \"" << key << "\"" << std::endl;
            success = false;
        }

        sqlite3_finalize( select );
        return success;
    }


    bool loadAllLayers( sqlite3* db, std::vector<std::string>& output )
    {
        bool success = true;

        sqlite3_stmt* select = 0L;
        std::string selectLayersSQL = "select layer from \"metadata\"";
        int rc = sqlite3_prepare_v2( db, selectLayersSQL.c_str(), selectLayersSQL.length(), &select, 0L );
        if ( rc != SQLITE_OK )
        {
            OE_WARN 
                << LC << "Error preparing SQL: " 
                << sqlite3_errmsg( db )
                << "(SQL: " << _insertSQL << ")"
                << std::endl;
            return false;
        }

        success = true;
        rc = sqlite3_step( select );
        while (rc == SQLITE_ROW) {
            output.push_back((char*)sqlite3_column_text( select, 0 ));
            rc = sqlite3_step( select );
        }

        if (rc != SQLITE_DONE)
        {
            // no result
            OE_WARN << "NO layers found in metadata" << std::endl;
            success = false;
        }

        sqlite3_finalize( select );
        return success;
    }

    std::string _insertSQL;
    std::string _selectSQL;
 };

// --------------------------------------------------------------------------

struct ImageRecord
{
    ImageRecord( const TileKey& key ) : _key(key) { }
    TileKey _key;
    int _created;
    int _accessed;
    osg::ref_ptr<const osg::Image> _image;
};

#ifdef INSERT_POOL
class Sqlite3Cache;
struct AsyncInsertPool : public TaskRequest {
    struct Entry {
        TileKey _key;
        osg::ref_ptr<osg::Image> _image;
        std::string _format;
        Entry() {}
        Entry(const TileKey& key, const std::string& format, osg::Image* img) : _key(key), _format(format), _image(img) {}
    };

    typedef std::map<std::string, Entry> PoolContainer;

    AsyncInsertPool(const std::string& layerName, Sqlite3Cache* cache );

    void addEntry( const TileKey& key, const std::string& format, osg::Image* image)
    {
        const std::string& keyStr = key.str();
        if (_pool.find(keyStr) != _pool.end())
            return;
        _pool[keyStr] = Entry(key, format, image);
    }

    osg::Image* findImage(const std::string& key)
    {
        PoolContainer::iterator it = _pool.find(key);
        if (it != _pool.end()) {
            return it->second._image.get();
        }
        return 0;
    }

    void operator()( ProgressCallback* progress );
    
    PoolContainer _pool;
    std::string _layerName;
    osg::observer_ptr<Sqlite3Cache> _cache;
};
#endif

/**
 * Database table that holds one record per cached imagery tile in a layer. The layer name
 * is the table name.
 */
struct LayerTable : public osg::Referenced
{
    LayerTable( const MetadataRecord& meta, sqlite3* db )
        : _meta(meta)
    {        
        _tableName = "layer_" + _meta._layerName;
        // create the table and load the processors.
        if ( ! initialize( db ) )
        {
            return;
        }

        // initialize the SELECT statement for fetching records
        std::stringstream buf;
#ifdef SPLIT_DB_FILE
        buf << "SELECT created,accessed,size FROM \"" << tableName << "\" WHERE key = ?";
#else
        buf << "SELECT created,accessed,data FROM \"" << _tableName << "\" WHERE key = ?";
#endif
        _selectSQL = buf.str();

        // initialize the UPDATE statement for updating the timestamp of an accessed record
        buf.str("");
        buf << "UPDATE \"" << _tableName << "\" SET accessed = ? "
            << "WHERE key = ?";
        _updateTimeSQL = buf.str();

        buf.str("");
        buf << "UPDATE \"" << _tableName << "\" SET accessed = ? "
            << "WHERE key in ( ? )";
        _updateTimePoolSQL = buf.str();
        
        // initialize the INSERT statement for writing records.
        buf.str("");
        buf << "INSERT OR REPLACE INTO \"" << _tableName << "\" "
#ifdef SPLIT_DB_FILE
            << "(key,created,accessed,size) VALUES (?,?,?,?)";
#else
            << "(key,created,accessed,data) VALUES (?,?,?,?)";
#endif
        _insertSQL = buf.str();

        // initialize the DELETE statements for purging old records.
        buf.str("");
        buf << "DELETE FROM \"" << _tableName << "\" "
            << "INDEXED BY \"" << _tableName << "_lruindex\" "
            << "WHERE accessed < ?";
        _purgeSQL = buf.str();
        
        buf.str("");
        buf << "DELETE FROM \""  << _tableName << "\" WHERE key in (SELECT key FROM \"" << _tableName << "\" WHERE \"accessed\" < ? limit ?)";
        _purgeLimitSQL = buf.str();          

        buf.str("");
        buf << "SELECT key FROM \"" << _tableName << "\" WHERE \"accessed\" < ? limit ?";
        _purgeSelect = buf.str();

        _statsLoaded = 0;
        _statsStored = 0;
        _statsDeleted = 0;
    }


    sqlite3_int64 getTableSize(sqlite3* db)
    {
#ifdef SPLIT_DB_FILE
        std::string query = "select sum(size) from \"" + _tableName + "\";";
#else
        std::string query = "select sum(length(data)) from \"" + _tableName + "\";";
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
        sqlite3_int64 size = sqlite3_column_int(select, 0);
        sqlite3_finalize( select );
        return size;
    }

    int getNbEntry(sqlite3* db) {
        std::string query = "select count(*) from \"" + _tableName + "\";";
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

    void checkAndPurgeIfNeeded(sqlite3* db, unsigned int maxSize )
    {
        int size = getTableSize(db);
        OE_DEBUG << _meta._layerName <<  std::dec << " : "  << size/1024/1024 << " MB" << std::endl;
        if (size < 0 || size < 1.2 * maxSize)
            return;
            
        ::time_t t = ::time(0L);
        int nbElements = getNbEntry(db);
        float averageSize = size * 1.0 / nbElements;
        float diffSize = size - maxSize;
        int maxElementToRemove = static_cast<int>(ceil(diffSize/averageSize));
        OE_DEBUG << _meta._layerName <<  " try to remove " << std::dec << maxElementToRemove << " / " <<  nbElements << " to save place" << std::endl;
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
        std::string keyStr = rec._key.str();
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
            OE_WARN << LC << "SQL INSERT failed for key " << rec._key.str() << ": " 
                << sqlite3_errmsg( db ) //<< "; tries=" << (1000-tries)
                << ", rc = " << rc << std::endl;
            sqlite3_finalize( insert );
            return false;
        }
        else
        {
            OE_DEBUG << LC << "cache INSERT tile " << rec._key.str() << std::endl;
            sqlite3_finalize( insert );
            _statsStored++;
            return true;
        }
    }

#ifdef INSERT_POOL
    bool beginStore(sqlite3* db, sqlite3_stmt*& insert) 
    {
        int rc;
        rc = sqlite3_exec( db, "BEGIN TRANSACTION", NULL, NULL, NULL);
        if (rc != 0) {
            OE_WARN << LC << "Failed to begin transaction batch of insert for " << _meta._layerName << " "  << sqlite3_errmsg(db) << std::endl;
            return false;
        }

        // prepare for multiple inserts
        rc = sqlite3_prepare_v2( db, _insertSQL.c_str(), _insertSQL.length(), &insert, 0L );
        if ( rc != SQLITE_OK )
        {
            OE_WARN 
                << LC << "Error preparing SQL: " 
                << sqlite3_errmsg( db )
                << "(SQL: " << _insertSQL << ")"
                << std::endl;
            return false;
        }
        return true;
    }

    bool storePool(sqlite3* db, const AsyncInsertPool::PoolContainer& entries, sqlite3_stmt* insert)
    {
        ::time_t t = ::time(0L);
        sqlite3_bind_int(  insert, 2, t );
        sqlite3_bind_int(  insert, 3, t );
        int rc;
        for (AsyncInsertPool::PoolContainer::const_iterator it = entries.begin(); it != entries.end(); ++it) {
            
            // bind the key string:
            std::string keyStr = it->first;
            sqlite3_bind_text( insert, 1, keyStr.c_str(), keyStr.length(), SQLITE_STATIC );

            // serialize the image:
#ifdef SPLIT_DB_FILE
            std::stringstream outStream;
            _rw->writeImage( * (it)->second._image.get(), outStream, _rwOptions.get() );
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
            _rw->writeImage( * (it)->second._image.get(), outStream, _rwOptions.get() );
            std::string outBuf = outStream.str();
            sqlite3_bind_blob( insert, 4, outBuf.c_str(), outBuf.length(), SQLITE_STATIC );
#endif
            rc = sqlite3_step(insert);   // executes the INSERT
            if ( rc != SQLITE_DONE )
            {
                OE_WARN << LC << "SQL INSERT failed for key " << keyStr << ": " 
                        << sqlite3_errmsg( db ) //<< "; tries=" << (1000-tries)
                        << ", rc = " << rc << std::endl;
                sqlite3_finalize(insert); // clean up prepared statement
                return false;
            }
            sqlite3_reset(insert);  // reset the statement 
        }

        _statsStored += entries.size();
        return true;
    }

    bool endStore(sqlite3* db, sqlite3_stmt* insert)
    {
        int rc = 0;
        rc = sqlite3_finalize(insert); // clean up prepared statement
        rc = sqlite3_exec( db, "COMMIT", NULL, NULL, NULL);  // COMMIT all dirty pages at once
        if (rc != 0) {
            OE_WARN << LC << "Failed to commit batch of insert for " << _meta._layerName << " "  << sqlite3_errmsg(db) << std::endl;
            return false;
        }
        
        return true;
    }
#endif

    bool updateAccessTime( const TileKey& key, int newTimestamp, sqlite3* db )
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
        std::string keyStr = key.str();
        sqlite3_bind_text( update, 2, keyStr.c_str(), keyStr.length(), SQLITE_STATIC );
        rc = sqlite3_step( update );
        if ( rc != SQLITE_DONE )
        {
            OE_WARN << LC << "Failed to update timestamp for " << key.str() << " on layer " << _meta._layerName << " rc = " << rc << std::endl;
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

    bool load( const TileKey& key, ImageRecord& output, sqlite3* db )
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

        std::string keyStr = key.str();
        sqlite3_bind_text( select, 1, keyStr.c_str(), keyStr.length(), SQLITE_STATIC );

        rc = sqlite3_step( select );
        if ( rc != SQLITE_ROW ) // == SQLITE_DONE ) // SQLITE_DONE means "no more rows"
        {
            // cache miss
            OE_DEBUG << LC << "Cache MISS on tile " << key.str() << std::endl;
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
            OE_DEBUG << LC << "Cache HIT on tile " << key.str() << std::endl;
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
            OE_DEBUG << _meta._layerName << " time " << d << " stored " << std::dec << _statsStored << " rate " << _statsStored * 1.0 / d << std::endl;
            OE_DEBUG << _meta._layerName << " time " << d << " loaded " << std::dec  << _statsLoaded << " rate " << _statsLoaded * 1.0 / d << std::endl;
            OE_DEBUG << _meta._layerName << " time " << d << " deleted " << std::dec  << _statsDeleted << " rate " << _statsDeleted * 1.0 / d << std::endl;
            _statsLastCheck = t;
        }
    }

    bool purge( int utcTimeStamp, int maxToRemove, sqlite3* db )
    {
        if ( maxToRemove < 0 )
            return false;

        sqlite3_stmt* purge = 0L;
        
        int rc;
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
        _statsDeleted += maxToRemove;
        return true;
    }

    /** Initializes the layer by creating the layer's table (if necessary), loading the
        appropriate reader-writer for the image data, and initializing the compressor
        if necessary. */
    bool initialize( sqlite3* db )
    {
        // first create the table if it does not already exist:
        std::stringstream buf;
        buf << "CREATE TABLE IF NOT EXISTS \"" << _tableName << "\" ("
            << "key char(64) PRIMARY KEY UNIQUE, "
            << "created int, "
            << "accessed int, "
#ifdef SPLIT_DB_FILE
            << "size int )";
#else
            << "data blob )";
#endif
        std::string sql = buf.str();

        OE_DEBUG << LC << "SQL = " << sql << std::endl;

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
            << _tableName << "_lruindex\" "
            << "ON \"" << _tableName << "\" (accessed)";
        sql = buf.str();

        OE_DEBUG << LC << "SQL = " << sql << std::endl;

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
    std::string _tableName;

    osg::ref_ptr<osgDB::ReaderWriter> _rw;
    osg::ref_ptr<osgDB::ReaderWriter::Options> _rwOptions;

    osg::Timer_t _statsStartTimer;
    osg::Timer_t _statsLastCheck;

    int _statsLoaded;
    int _statsStored;
    int _statsDeleted;

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
    AsyncInsert( const TileKey& key, const CacheSpec& spec, const osg::Image* image, AsyncCache* cache )
        : _cacheSpec(spec), _key(key), _image(image), _cache(cache) { }

    void operator()( ProgressCallback* progress ) {
        osg::ref_ptr<AsyncCache> cache = _cache.get();
        if ( cache.valid() )
            cache->setImageSync( _key, _cacheSpec, _image.get() );
    }

    CacheSpec _cacheSpec;
    TileKey _key;
    osg::ref_ptr<const osg::Image> _image;
    osg::observer_ptr<AsyncCache> _cache;
};

class Sqlite3Cache;
struct AsyncUpdateAccessTime : public TaskRequest
{
    AsyncUpdateAccessTime( const TileKey& key, const std::string& cacheId, int timeStamp, Sqlite3Cache* cache );
    void operator()( ProgressCallback* progress );

    TileKey _key;
    std::string _cacheId;
    int _timeStamp;
    osg::observer_ptr<Sqlite3Cache> _cache;
};



struct AsyncUpdateAccessTimePool : public TaskRequest
{
    AsyncUpdateAccessTimePool( const std::string& cacheId, Sqlite3Cache* cache );
    void addEntry(const TileKey& key, int timeStamp);
    void addEntryInternal(const TileKey& key);

    void operator()( ProgressCallback* progress );
    const std::string& getCacheId() { return _cacheId; }
    int getNbEntry() const { return _keys.size(); }
    std::map<std::string, int> _keys;
    std::string _cacheId;
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
    Sqlite3Cache( const CacheOptions& options ) 
      : AsyncCache(options), _options(options),  _db(0L)
    {                
        if ( _options.path().get().empty() || options.getReferenceURI().empty() )
            _databasePath = _options.path().get();
        else
        {
           _databasePath = osgEarth::getFullPath( options.getReferenceURI(), _options.path().get() );
        }

        
        setName( "sqlite3" );

        _nbRequest = 0;

        //_settings = dynamic_cast<const Sqlite3CacheOptions*>( options );
        //if ( !_settings.valid() )
        //    _settings = new Sqlite3CacheOptions( options );

        OE_INFO << LC << "options: " << _options.getConfig().toString() << std::endl;

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
        
        _db = openDatabase( _databasePath, _options.serialized().value() );

        if ( _db )
        {
            if ( ! _metadata.initialize( _db ) )
                _db = 0L;
        }

        if ( _db && _options.asyncWrites() == true )
        {
            _writeService = new osgEarth::TaskService( "Sqlite3Cache Write Service", 1 );
        }

        
        if (!_metadata.loadAllLayers( _db, _layersList )) {
            OE_WARN << "can't read layers in meta data" << std::endl;
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
    bool isCached( const TileKey& key, const CacheSpec& spec ) const
    {
        // this looks ineffecient, but usually when isCached() is called, getImage() will be
        // called soon thereafter. And this call will load it into the L2 cache so the subsequent
        // getImage call will not hit the DB again.
        osg::ref_ptr<const osg::Image> temp;
        return const_cast<Sqlite3Cache*>(this)->getImage( key, spec, temp );
    }

    /**
     * Store the cache profile for the given profile.
     */
    virtual void storeProperties( const CacheSpec& spec, const Profile* profile, unsigned int tileSize ) 
    {
        if ( !_db ) return;

        if ( spec.cacheId().empty() || profile == 0L || spec.format().empty() )
        {
            OE_WARN << "ILLEGAL: cannot cache a layer without a layer id" << std::endl;
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
        rec._layerName = spec.cacheId();
        rec._profile = profile;
        rec._tileSize = tileSize;

#ifdef USE_SERIALIZERS
        rec._format = "osgb";
        rec._compressor = "zlib";
#else
        rec._format = spec.format();
#endif

        _metadata.store( rec, db );
    }

    /**
     * Loads the cache profile for the given layer.
     */
    virtual bool loadProperties( 
        const std::string&           cacheId, 
        CacheSpec&                   out_spec, 
        osg::ref_ptr<const Profile>& out_profile,
        unsigned int&                out_tileSize ) 
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

        OE_DEBUG << LC << "Loading metadata for layer \"" << cacheId << "\"" << std::endl;

        MetadataRecord rec;
        if ( _metadata.load( cacheId, db, rec ) )
        {
            out_spec = CacheSpec( rec._layerName, rec._format );
            out_tileSize = rec._tileSize;
            out_profile = rec._profile;
        }
        return 0L;
    }

    /**
     * Gets the cached image for the given TileKey
     */
    bool getImage( const TileKey& key, const CacheSpec& spec, osg::ref_ptr<const osg::Image>& out_image )
    {
        if ( !_db ) return false;

        // wait if we are purging the db
        ScopedLock<Mutex> lock2( _pendingPurgeMutex );

        // first try the L2 cache.
        if ( _L2cache.valid() )
        {
            if ( _L2cache->getImage( key, spec, out_image ) )
                return true;
        }

        // next check the deferred-write queue.
        if ( _options.asyncWrites() == true )
        {
#ifdef INSERT_POOL
            ScopedLock<Mutex> lock( _pendingWritesMutex );
            std::string name = layerName;
            std::map<std::string, osg::ref_ptr<AsyncInsertPool> >::iterator it = _pendingWrites.find(name);
            if (it != _pendingWrites.end()) {
                AsyncInsertPool* p = it->second.get();
                if (p) {
                    osg::Image* img = p->findImage(key.str());
                    if (img) {
                        // todo: update the access time, or let it slide?
                        OE_DEBUG << LC << "Got key that is write-queued: " << key.str() << std::endl;
                        return img;
                    }
                }
            }
#else
            ScopedLock<Mutex> lock( _pendingWritesMutex );
            std::string name = key.str() + spec.cacheId(); //layerName;
            std::map<std::string,osg::ref_ptr<AsyncInsert> >::iterator i = _pendingWrites.find(name);
            if ( i != _pendingWrites.end() )
            {
                // todo: update the access time, or let it slide?
                OE_DEBUG << LC << "Got key that is write-queued: " << key.str() << std::endl;
                out_image = i->second->_image.get();
                return out_image.valid();
                //return i->second->_image.get();
            }
#endif
        }

        // finally, try to query the database.
        ThreadTable tt = getTable( spec.cacheId() ); //layerName);
        if ( tt._table )
        {
            ImageRecord rec( key );
            if (!tt._table->load( key, rec, tt._db ))
                return false;

            // load it into the L2 cache
            out_image = rec._image.release();

            if ( out_image.valid() && _L2cache.valid() )
                _L2cache->setImage( key, spec, out_image.get() );

#ifdef UPDATE_ACCESS_TIMES

#ifdef UPDATE_ACCESS_TIMES_POOL
            // update the last-access time
            int t = (int)::time(0L);
            {
                ScopedLock<Mutex> lock( _pendingUpdateMutex );
                osg::ref_ptr<AsyncUpdateAccessTimePool> pool;
                std::map<std::string,osg::ref_ptr<AsyncUpdateAccessTimePool> >::iterator i = _pendingUpdates.find( spec.cacheId() ); //layerName);
                if ( i != _pendingUpdates.end() )
                {
                    i->second->addEntry(key, t);
                    pool = i->second;
                    OE_DEBUG << LC << "Add key " << key.str() << " to existing layer batch " << spec.name() << std::endl;
                } else {
                    pool = new AsyncUpdateAccessTimePool(spec.cacheId(), this);
                    pool->addEntry(key, t);
                    _pendingUpdates[spec.cacheId()] = pool.get();
                    _writeService->add(pool.get());
                }
            }
#else
            // update the last-access time
            int t = (int)::time(0L);
            _writeService->add( new AsyncUpdateAccessTime(  key, layerName, t, this ) );
#endif

#endif // UPDATE_ACCESS_TIMES

            return out_image.valid();
        }
        else
        {
            OE_DEBUG << LC << "What, no layer table?" << std::endl;
        }
        return false;
    }

    /**
     * Sets the cached image for the given TileKey
     */
    void setImage( const TileKey& key, const CacheSpec& spec, const osg::Image* image )
    {        
        if ( !_db ) return;

        if ( _options.asyncWrites() == true )
        {
            // the "pending writes" table is here so that we don't try to write data to
            // the cache more than once when using an asynchronous write service.
            ScopedLock<Mutex> lock( _pendingWritesMutex );
#ifdef INSERT_POOL
            std::string name = layerName;
            std::map<std::string, osg::ref_ptr<AsyncInsertPool> >::iterator it = _pendingWrites.find(name);
            if ( it == _pendingWrites.end() )
            {
                AsyncInsertPool* req = new AsyncInsertPool(layerName, this);
                req->addEntry(key, format, image);
                _pendingWrites[name] = req;
                _writeService->add( req );
            }
            else
            {
                it->second->addEntry(key, format, image);
            }
#else
            std::string name = key.str() + spec.cacheId();
            if ( _pendingWrites.find(name) == _pendingWrites.end() )
            {
                AsyncInsert* req = new AsyncInsert(key, spec, image, this);
                _pendingWrites[name] = req;
                _writeService->add( req );
            }
            else
            {
                //NOTE: this should probably never happen.
                OE_WARN << LC << "Tried to setImage; already in queue: " << key.str() << std::endl;
            }
#endif
        }
        else
        {

            setImageSync( key, spec, image );
        }
    }

    /**
     * Purges records from the database.
     */
    bool purge( const std::string& layerName, int olderThanUTC, bool async )
    {
        if ( !_db ) return false;

        // purge the L2 cache first:
        if ( async == true && _options.asyncWrites() == true )
        {
#ifdef PURGE_GENERAL
            if (!_pendingPurges.empty())
                return false;
            ScopedLock<Mutex> lock( _pendingPurgeMutex );
            AsyncPurge* req = new AsyncPurge(layerName, olderThanUTC, this);
            _writeService->add( req);
            _pendingPurges[layerName] = req;
#else
            if (_pendingPurges.find(layerName) != _pendingPurges.end()) {
                return false;
            } else {
                ScopedLock<Mutex> lock( _pendingPurgeMutex );
                AsyncPurge* req = new AsyncPurge(layerName, olderThanUTC, this);
                _writeService->add( req);
                _pendingPurges[layerName] = req;
            }
#endif
        }
        else
        {
#ifdef PURGE_GENERAL
            ScopedLock<Mutex> lock( _pendingPurgeMutex );

            sqlite3_int64 limit = _options.maxSize().value() * 1024 * 1024;
            std::map<std::string, std::pair<sqlite3_int64,int> > layers;
            sqlite3_int64 totalSize = 0;
            for (unsigned int i = 0; i < _layersList.size(); ++i) {
                ThreadTable tt = getTable( _layersList[i] );
                if ( tt._table ) {
                    sqlite3_int64 size = tt._table->getTableSize(tt._db);
                    layers[_layersList[i] ].first = size;
                    layers[_layersList[i] ].second = tt._table->getNbEntry(tt._db);
                    totalSize += size;
                }
            }
            OE_INFO << LC << "SQlite cache size " << totalSize/(1024*1024) << " MB" << std::endl;
            if (totalSize > 1.2 * limit) {
                sqlite3_int64 diff = totalSize - limit;
                for (unsigned int i = 0; i < _layersList.size(); ++i) {
                    float ratio = layers[_layersList[i] ].first * 1.0 / (float)(totalSize);
                    int sizeToRemove = (int)floor(ratio * diff);
                    if (sizeToRemove > 0) {
                        if (sizeToRemove / 1024 > 1024) {
                            OE_DEBUG << "Try to remove " << sizeToRemove/(1024*1024) << " MB in " << _layersList[i] << std::endl;
                        } else {
                            OE_DEBUG << "Try to remove " << sizeToRemove/1024 << " KB in " << _layersList[i] << std::endl;
                        }

                        if ( _L2cache.valid() )
                            _L2cache->purge( _layersList[i], olderThanUTC, async );
                        ThreadTable tt = getTable(_layersList[i]);
                        if ( tt._table ) {
                            float averageSizePerElement = layers[_layersList[i] ].first * 1.0 /layers[_layersList[i] ].second;
                            int nb = (int)floor(sizeToRemove / averageSizePerElement);
                            if (nb ) {
                                OE_DEBUG << "remove " << nb << " / " << layers[_layersList[i] ].second << " elements in " << _layersList[i] << std::endl;
                                tt._table->purge(olderThanUTC, nb, tt._db);
                            }
                        }
                    }
                }
            }
            _pendingPurges.clear();
            displayPendingOperations();

#else
            ScopedLock<Mutex> lock( _pendingPurgeMutex );
            if ( _L2cache.valid() )
                _L2cache->purge( layerName, olderThanUTC, async );

            ThreadTable tt = getTable( layerName );
            if ( tt._table )
            {
                _pendingPurges.erase( layerName );

                unsigned int maxsize = _options.getSize(layerName);
                tt._table->checkAndPurgeIfNeeded(tt._db, maxsize * 1024 * 1024);
                displayPendingOperations();
            }
#endif
        }
        return true;
    }

    /**
     * updateAccessTime records on the database.
     */
    bool updateAccessTimeSync( const std::string& layerName, const TileKey& key, int newTimestamp )
    {
        if ( !_db ) return false;

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
        if ( !_db ) return false;

        ThreadTable tt = getTable(layerName);
        if ( tt._table )
        {
            tt._table->updateAccessTimePool( keys, newTimestamp, tt._db );
        }

        {
            ScopedLock<Mutex> lock( _pendingUpdateMutex );
            _pendingUpdates.erase( layerName );
            displayPendingOperations();
        }
        return true;
    }

#ifdef INSERT_POOL
    void setImageSyncPool( AsyncInsertPool* pool, const std::string& layerName)
    {
        ScopedLock<Mutex> lock( _pendingWritesMutex );
        const AsyncInsertPool::PoolContainer& entries = pool->_pool;
        OE_WARN << "write " << entries.size() << std::endl;
        ThreadTable tt = getTable(layerName);
        if ( tt._table )
        {
            sqlite3_stmt* insert;
            if (!tt._table->beginStore( tt._db, insert )) {
                return;
            }
            if (!tt._table->storePool( tt._db, entries, insert )) {
                return;
            }
            if (!tt._table->endStore( tt._db, insert )) {
                return;
            }
        }
        _pendingWrites.erase( layerName );
        displayPendingOperations();
    }
#endif

private:

    void displayPendingOperations() {
        if (_pendingWrites.size())
            OE_DEBUG<< LC << "pending insert " << _pendingWrites.size() << std::endl;
        if (_pendingUpdates.size())
            OE_DEBUG << LC << "pending update " << _pendingUpdates.size() << std::endl;
        if (_pendingPurges.size())
            OE_DEBUG << LC << "pending purge " << _pendingPurges.size() << std::endl;
        //OE_INFO << LC << "Pending writes: " << std::dec << _writeService->getNumRequests() << std::endl;
    }

    void setImageSync( const TileKey& key, const CacheSpec& spec, const osg::Image* image )
    {
        if (_options.maxSize().value() > 0 && _nbRequest > MAX_REQUEST_TO_RUN_PURGE) {
            int t = (int)::time(0L);
            purge(spec.cacheId(), t, _options.asyncWrites().value() );
            _nbRequest = 0;
        }
        _nbRequest++;

        ThreadTable tt = getTable( spec.cacheId() );
        if ( tt._table )
        {
            ::time_t t = ::time(0L);
            ImageRecord rec( key );
            rec._created = (int)t;
            rec._accessed = (int)t;
            rec._image = image;

            tt._table->store( rec, tt._db );
        }

        if ( _options.asyncWrites() == true )
        {
            ScopedLock<Mutex> lock( _pendingWritesMutex );
            std::string name = key.str() + spec.cacheId();
            _pendingWrites.erase( name );
            displayPendingOperations();
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
            db = openDatabase( layer + _options.path().value(), _options.serialized().value() );
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
            db = openDatabase( _options.path().value(), _options.serialized().value() );
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
            db = openDatabase( _databasePath, _options.serialized().value() );
            if ( db )
            {
                _dbPerThread[thread] = db;
                OE_DEBUG << LC << "Created DB handle " << std::hex << db << " for thread " << thread << std::endl;
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
    ThreadTable getTable( const std::string& tableName )
    {
        ScopedLock<Mutex> lock( _tableListMutex );

#ifdef SPLIT_LAYER_DB
        sqlite3* db = getOrCreateDbForThread(tableName);
#else
        sqlite3* db = getOrCreateDbForThread();
#endif
        if ( !db )
            return ThreadTable( 0L, 0L );

        LayerTablesByName::iterator i = _tables.find(tableName);
        if ( i == _tables.end() )
        {
            MetadataRecord meta;
#ifdef SPLIT_LAYER_DB
            sqlite3* metadb = getOrCreateMetaDbForThread();
            if ( !_metadata.load( tableName, metadb, meta ) )
#else
            if ( !_metadata.load( tableName, db, meta ) )
#endif
            {
                OE_WARN << LC << "Cannot operate on \"" << tableName << "\" because metadata does not exist."
                    << std::endl;
                return ThreadTable( 0L, 0L );
            }

            _tables[tableName] = new LayerTable( meta, db );
            OE_DEBUG << LC << "New LayerTable for " << tableName << std::endl;
        }
        return ThreadTable( _tables[tableName].get(), db );
    }

private:

    const Sqlite3CacheOptions _options;
    //osg::ref_ptr<const Sqlite3CacheOptions> _settings;
    osg::ref_ptr<osgDB::ReaderWriter> _defaultRW;
    Mutex             _tableListMutex;
    MetadataTable     _metadata;
    LayerTablesByName _tables;

    bool _useAsyncWrites;
    osg::ref_ptr<TaskService> _writeService;
    Mutex _pendingWritesMutex;

#ifdef INSERT_POOL
    std::map<std::string, osg::ref_ptr<AsyncInsertPool> > _pendingWrites;
#else
    std::map<std::string, osg::ref_ptr<AsyncInsert> > _pendingWrites;
#endif
    Mutex _pendingUpdateMutex;
    std::map<std::string, osg::ref_ptr<AsyncUpdateAccessTimePool> > _pendingUpdates;

    Mutex _pendingPurgeMutex;
    std::map<std::string, osg::ref_ptr<AsyncPurge> > _pendingPurges;

    sqlite3* _db;
    std::map<Thread*,sqlite3*> _dbPerThread;

    std::map<std::string, std::map<Thread*,sqlite3*> > _dbPerThreadLayers;
    std::map<Thread*,sqlite3*> _dbPerThreadMeta;

    osg::ref_ptr<MemCache> _L2cache;

    int _count;
    int _nbRequest;

    std::vector<std::string> _layersList;
    std::string _databasePath;
};



AsyncUpdateAccessTime::AsyncUpdateAccessTime( const TileKey& key, const std::string& cacheId, int timeStamp, Sqlite3Cache* cache ) : 
_key(key), _cacheId(cacheId), _timeStamp(timeStamp), _cache(cache)
{
    //nop
}

void AsyncUpdateAccessTime::operator()( ProgressCallback* progress ) 
{ 
    osg::ref_ptr<Sqlite3Cache> cache = _cache.get();
    if ( cache.valid() ) {
        //OE_WARN << "AsyncUpdateAccessTime will process " << _key << std::endl;
        cache->updateAccessTimeSync( _cacheId, _key , _timeStamp );
    }
}


AsyncUpdateAccessTimePool::AsyncUpdateAccessTimePool(const std::string& cacheId, Sqlite3Cache* cache) :
_cacheId(cacheId), _cache(cache)
{
    //nop
}

void AsyncUpdateAccessTimePool::addEntry(const TileKey& key, int timeStamp)
{
    unsigned int lod = key.getLevelOfDetail();
    addEntryInternal(key);
    if (lod > 0) {
        TileKey previous = key;
        for (int i = (int)lod-1; i >= 0; --i) {
            TileKey ancestor = previous.createAncestorKey(i);
            if (ancestor.valid())
                addEntryInternal(ancestor);
            previous = ancestor;
        }
    }
    _timeStamp = timeStamp;
}

void AsyncUpdateAccessTimePool::addEntryInternal(const TileKey& key)
{
    const std::string& keyStr = key.str();
    if (_keys.find(keyStr) == _keys.end()) {
        _keys[keyStr] = 1;
        if (_keyStr.empty())
            _keyStr = keyStr;
        else
            _keyStr += ", " + keyStr;
    }
}

void AsyncUpdateAccessTimePool::operator()( ProgressCallback* progress ) 
{ 
    osg::ref_ptr<Sqlite3Cache> cache = _cache.get();
    if ( cache.valid() ) {
        //OE_INFO << "AsyncUpdateAccessTimePool will process " << _keys.size() << std::endl;
        cache->updateAccessTimeSyncPool( _cacheId, _keyStr , _timeStamp );
    }
}


#ifdef INSERT_POOL
AsyncInsertPool::AsyncInsertPool(const std::string& layerName, Sqlite3Cache* cache ) : _layerName(layerName), _cache(cache) { }
void AsyncInsertPool::operator()( ProgressCallback* progress )
{
    osg::ref_ptr<Sqlite3Cache> cache = _cache.get();
    if ( cache.valid() ) {
        cache->setImageSyncPool( this, _layerName );
    }
}
#endif

//------------------------------------------------------------------------

/**
 * This driver defers loading of the source data to the appropriate OSG plugin. You
 * must explicity set an override profile when using this driver.
 *
 * For example, use this driver to load a simple jpeg file; then set the profile to
 * tell osgEarth its projection.
 */
class Sqlite3CacheFactory : public CacheDriver
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

        return ReadResult( new Sqlite3Cache( getCacheOptions(options) ) );
    }
};

REGISTER_OSGPLUGIN(osgearth_cache_sqlite3, Sqlite3CacheFactory)

