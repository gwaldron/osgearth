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
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>
#include <cstring>

// for the compressor stuff
#if OSG_MIN_VERSION_REQUIRED(2,9,8)
#  define USE_COMPRESSOR
#  include <osgDB/Serializer>
#endif

#include <sqlite/sqlite3.h>

using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace OpenThreads;

#define LC "[Sqlite3Cache] "

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
        _db = db;

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
        int err = sqlite3_exec( _db, sql.c_str(), 0L, 0L, &errMsg );
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

    bool store( const MetadataRecord& rec )
    {
        sqlite3_stmt* insert = 0;
        int rc = sqlite3_prepare_v2( _db, _insertSQL.c_str(), _insertSQL.length(), &insert, 0L );
        if ( rc != SQLITE_OK )
        {
            OE_WARN 
                << LC << "Error preparing SQL: " 
                << sqlite3_errmsg( _db )
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
            OE_WARN << LC << "SQL INSERT failed: " << sqlite3_errmsg( _db )
                << "; SQL = " << _insertSQL
                << std::endl;
            success = false;
        }
        else
        {
            success = true;
        }

        sqlite3_finalize( insert );
        return success;
    }

    bool load( const std::string& key, MetadataRecord& output )
    {
        bool success = true;

        sqlite3_stmt* select = 0L;
        int rc = sqlite3_prepare_v2( _db, _selectSQL.c_str(), _selectSQL.length(), &select, 0L );
        if ( rc != SQLITE_OK )
        {
            OE_WARN 
                << LC << "Error preparing SQL: " 
                << sqlite3_errmsg( _db )
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
        return success;
    }

    bool remove( const std::string& key )
    {
        //TODO
        return false;
    }

    sqlite3* _db;
    std::string _insertSQL;
    std::string _selectSQL;
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
        : _meta(meta), _db(db)
    {
        // create the table and load the processors.
        if ( ! initialize() )
        {
            _db = 0L;
            return;
        }

        // initialize the SELECT statement for fetching records
        std::stringstream buf;
        buf << "SELECT created,accessed,data FROM \"" << _meta._layerName << "\" WHERE key = ?";
        _selectSQL = buf.str();
        
        // initialize the INSERT statement for writing records.
        buf.str("");
        buf << "INSERT OR REPLACE INTO \"" << _meta._layerName << "\" "
            << "(key,created,accessed,data) VALUES (?,?,?,?)";
        _insertSQL = buf.str();
    }

    ~LayerTable()
    {
        _db = 0L;
    }

    bool store( const ImageRecord& rec )
    {
        if ( !_db ) return false;

        sqlite3_stmt* insert = 0L;    
        int rc = sqlite3_prepare_v2( _db, _insertSQL.c_str(), _insertSQL.length(), &insert, 0L );
        if ( rc != SQLITE_OK )
        {
            OE_WARN 
                << LC << "Error preparing SQL: " 
                << sqlite3_errmsg( _db )
                << "(SQL: " << _insertSQL << ")"
                << std::endl;
            return false;
        }

        // bind the key string:
        sqlite3_bind_text( insert, 1, rec._key->str().c_str(), -1, SQLITE_TRANSIENT );
        sqlite3_bind_int(  insert, 2, rec._created );
        sqlite3_bind_int(  insert, 3, rec._accessed );

        // serialize the image:
        std::stringstream outStream;
        _rw->writeImage( *rec._image.get(), outStream );
        std::string outBuf = outStream.str();
        sqlite3_bind_blob( insert, 4, outBuf.c_str(), outBuf.length(), SQLITE_STATIC );

        // write to the database:
        rc = sqlite3_step( insert );
        if ( rc != SQLITE_DONE )
        {
            OE_WARN << LC << "SQL INSERT failed for key " << rec._key->str() << ": " 
                << sqlite3_errmsg( _db ) << std::endl;
            sqlite3_finalize( insert );
            return false;
        }
        else
        {
            OE_DEBUG << LC << "cache INSERT tile " << rec._key->str() << std::endl;
            sqlite3_finalize( insert );
            return true;
        }
    }

    bool updateLastAccessTime( const std::string& key, int newTimestamp )
    {
        if ( !_db ) return false;
        //TODO
        return false;
    }

    bool load( const TileKey* key, ImageRecord& output )
    {
        if ( !_db ) return false;

        char* imageBuf = 0L;
        int imageBufLen = 0;
        
        sqlite3_stmt* select = 0L;
        int rc = sqlite3_prepare_v2( _db, _selectSQL.c_str(), _selectSQL.length(), &select, 0L );
        if ( rc != SQLITE_OK )
        {
            OE_WARN << LC << "Failed to prepare SQL: " << _selectSQL << "; " << sqlite3_errmsg(_db) << std::endl;
            return false;
        }

        sqlite3_bind_text( select, 1, key->str().c_str(), key->str().length(), SQLITE_TRANSIENT );

        rc = sqlite3_step( select );
        if ( rc == SQLITE_DONE ) // SQLITE_DONE means "no more rows"
        {
            // cache miss
            OE_DEBUG << LC << "Cache MISS on tile " << key->str() << std::endl;
            sqlite3_finalize(select);
            return false;
        }

        // copy the timestamps:
        output._created  = sqlite3_column_int( select, 0 );
        output._accessed = sqlite3_column_int( select, 1 );

        // copy the blob data into a temporary buffer:
        imageBufLen = sqlite3_column_bytes( select, 2 );
        imageBuf = new char[imageBufLen];
        memcpy( imageBuf, (const char*)sqlite3_column_blob( select, 2 ), imageBufLen );          

        // deserialize the image from the buffer:
        // TODO: decompression
        std::string imageString( imageBuf, imageBufLen );
        std::stringstream imageBufStream( imageString );
        osgDB::ReaderWriter::ReadResult rr = _rw->readImage( imageBufStream );

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

        delete [] imageBuf;
        sqlite3_finalize(select);

        return output._image.valid();
    }

    bool remove( const std::string& key )
    {
        if ( !_db ) return false;
        //TODO
        return false;
    }

    bool removeOlderThan( int timestamp )
    {
        if ( !_db ) return false;
        //TODO
        return false;
    }

    /** Initializes the layer by creating the layer's table (if necessary), loading the
        appropriate reader-writer for the image data, and initializing the compressor
        if necessary. */
    bool initialize()
    {
        // first create the table if it does not already exist:
        std::stringstream buf;
        buf << "CREATE TABLE IF NOT EXISTS \"" << _meta._layerName << "\" ("
            << "key char(64) PRIMARY KEY UNIQUE, "
            << "created int, "
            << "accessed int, "
            << "data blob )";
        std::string sql = buf.str();

        OE_INFO << LC << "SQL = " << sql << std::endl;

        char* errMsg = 0L;
        int rc = sqlite3_exec( _db, sql.c_str(), 0L, 0L, &errMsg );
        if ( rc != SQLITE_OK )
        {
            OE_WARN << LC << "Creating layer: " << errMsg << std::endl;
            sqlite3_free( errMsg );
            return false;
        }

        // next load the appropriate ReaderWriter:
        _rw = osgDB::Registry::instance()->getReaderWriterForMimeType( _meta._format );
        if ( !_rw.valid() )
            _rw = osgDB::Registry::instance()->getReaderWriterForExtension( _meta._format );
        if ( !_rw.valid() )
        {
            OE_WARN << LC << "Creating layer: Cannot initialize ReaderWriter for format \"" 
                << _meta._format << "\"" << std::endl;
            return false;
        }

        // next load the compressor if applicable:
#ifdef USE_COMPRESSOR
        if ( !_meta._compressor.empty() )
        {
            _comp = osgDB::Registry::instance()->getObjectWrapperManager()->findCompressor( _meta._compressor );
            if ( !_comp )
            {
                OE_WARN << LC << "Layer \"" << _meta._layerName << "\": unable to find compressor \""
                    << _meta._compressor << "\"" << std::endl;
                return false;
            }
        }
#endif

        return true;
    }

    sqlite3* _db;
    std::string _selectSQL;
    std::string _insertSQL;
    MetadataRecord _meta;
    osg::ref_ptr<osgDB::ReaderWriter> _rw;
    osg::ref_ptr<osgDB::BaseCompressor> _comp;
};

// --------------------------------------------------------------------------

typedef std::map<std::string,osg::ref_ptr<LayerTable> > LayerTablesByName;

// --------------------------------------------------------------------------

class Sqlite3Cache : public Cache
{
public:

    Sqlite3Cache( const PluginOptions* options ) : Cache(), _db(0L)
    {
        _settings = dynamic_cast<const Sqlite3CacheOptions*>( options );
        if ( !_settings.valid() )
            _settings = new Sqlite3CacheOptions( options );

        OE_INFO << LC << "settings: " << options->config().toString() << std::endl;

        if ( sqlite3_open( _settings->path()->c_str(), &_db ) != 0 )
        {
            OE_WARN << LC << "failed to open or create database at " << _settings->path() << std::endl;
        }

        if ( _db )
        {
            _metadata.initialize( _db );
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
        //TODO- do something better
        osg::ref_ptr<osg::Image> image = const_cast<Sqlite3Cache*>(this)->getImage( key, layerName, format );
        return image.valid();
    }

    /**
     * Store the cache profile for the given profile.
     */
    void storeLayerProperties(
        const std::string& layerName, const Profile* profile,
        const std::string& format, unsigned int tileSize)
    {
        if ( layerName.empty() || profile == 0L || format.empty() )
        {
            OE_WARN << "ILLEGAL: cannot cache a layer without a layer name" << std::endl;
            return;
        }

        OE_INFO << "Storing metadata for layer \"" << layerName << "\"" << std::endl;

        MetadataRecord rec;
        rec._format = format;
        rec._layerName = layerName;
        rec._profile = profile;
        rec._tileSize = tileSize;

        _metadata.store( rec );
    }

    /**
     * Loads the cache profile for the given layer.
     */
    const Profile* loadLayerProperties(
        const std::string& layerName,
        std::string& out_format,
        unsigned int& out_tileSize )
    {
        OE_INFO << "Loading metadata for layer \"" << layerName << "\"" << std::endl;

        MetadataRecord rec;
        if ( _metadata.load( layerName, rec ) )
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
        LayerTable* table = getTable(layerName);
        if ( table )
        {
            ImageRecord rec;
            if ( table->load( key, rec ) )
                return rec._image.release();
        }
        return 0L;
    }

    /**
     * Sets the cached image for the given TileKey
     */
    void setImage( const TileKey* key, const std::string& layerName, const std::string& format, osg::Image* image )
    {
        LayerTable* table = getTable(layerName);
        if ( table )
        {
            ::time_t t = ::time(NULL);
            ImageRecord rec;
            rec._key = key;
            rec._created = (int)t;
            rec._accessed = (int)t;
            rec._image = image;

            table->store( rec );
        }
    }

private:

    // gets the layer table for the specified layer name, creating it if it does
    // not already exist...
    LayerTable* getTable( const std::string& layerName )
    {
        ScopedLock<Mutex> lock( _tableListMutex );

        LayerTablesByName::iterator i = _tables.find(layerName);
        if ( i == _tables.end() )
        {
            MetadataRecord meta;
            if ( !_metadata.load( layerName, meta ) )
            {
                OE_WARN << LC << "Cannot operate on \"" << layerName << "\" because metadata does not exist."
                    << std::endl;
                return 0L;
            }

            _tables[layerName] = new LayerTable( meta, _db );
        }
        return _tables[layerName].get();
    }

private:

    osg::ref_ptr<const Sqlite3CacheOptions> _settings;
    osg::ref_ptr<osgDB::ReaderWriter> _defaultRW;
    sqlite3*        _db;
    Mutex             _tableListMutex;
    MetadataTable     _metadata;
    LayerTablesByName _tables;
};


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

