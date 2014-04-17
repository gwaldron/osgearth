/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include "LevelDBCache"
#include <osg/Object>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>
#include <osgEarth/Cache>
#include <osgEarth/StringUtils>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/URI>
#include <osgEarth/Registry>
#include <leveldb/db.h>
#include <leveldb/write_batch.h>
#include <string>

#define LEVELDB_CACHE_VERSION 1

using namespace osgEarth;
using namespace osgEarth::Threading;
using namespace osgEarth::Drivers::LevelDBCache;

namespace
{
    // adapters for all the osg read functions...
    struct Reader {
        osgDB::ReaderWriter* _rw;
        osgDB::Options*      _op;
        Reader(osgDB::ReaderWriter* rw, osgDB::Options* op) : _rw(rw), _op(op) { }
        virtual osgDB::ReaderWriter::ReadResult read(std::istream& in) const = 0;
    };
    struct ImageReader : public Reader {
        ImageReader(osgDB::ReaderWriter* rw, osgDB::Options* op) : Reader(rw, op) { }
        osgDB::ReaderWriter::ReadResult read(std::istream& in) const { return _rw->readImage(in, _op); }
    };
    struct NodeReader : public Reader {
        NodeReader(osgDB::ReaderWriter* rw, osgDB::Options* op) : Reader(rw, op) { }
        osgDB::ReaderWriter::ReadResult read(std::istream& in) const { return _rw->readNode(in, _op); }
    };
    struct ObjectReader : public Reader {
        ObjectReader(osgDB::ReaderWriter* rw, osgDB::Options* op) : Reader(rw, op) { }
        osgDB::ReaderWriter::ReadResult read(std::istream& in) const { return _rw->readObject(in, _op); }
    };


    /** 
     * Cache that stores data in the local file system.
     */
    class LevelDBCacheImpl : public osgEarth::Cache
    {
    public:
        LevelDBCacheImpl() { } // unused
        LevelDBCacheImpl( const LevelDBCacheImpl& rhs, const osg::CopyOp& op ) { } // unused
        META_Object( osgEarth, LevelDBCacheImpl );

        /**
         * Constructs a new file system cache.
         * @param options Options structure that comes from a serialized description of 
         *        the object.
         */
        LevelDBCacheImpl( const CacheOptions& options );

    public: // Cache interface

        CacheBin* addBin( const std::string& binID );

        CacheBin* getOrCreateDefaultBin();

    protected:

        void init();

        std::string _rootPath;
        bool        _active;
    };

    /** 
     * Cache bin implementation for a LevelDBCache.
     * You don't need to create this object directly; use LevelDBCache::createBin instead.
    */
    class LevelDBCacheBin : public CacheBin
    {
    public:
        LevelDBCacheBin( const std::string& name, const std::string& rootPath );

        virtual ~LevelDBCacheBin();

    public: // CacheBin interface

        ReadResult readObject(const std::string& key, TimeStamp minTime);

        ReadResult readImage(const std::string& key, TimeStamp minTime);

        ReadResult readNode(const std::string& key, TimeStamp minTime);

        ReadResult readString(const std::string& key, TimeStamp minTime);

        bool write(const std::string& key, const osg::Object* object, const Config& meta);

        bool remove(const std::string& key);

        bool touch(const std::string& key);

        RecordStatus getRecordStatus(const std::string& key, TimeStamp minTime);

        bool purge();

        bool compact();
        
        unsigned getStorageSize();

        Config readMetadata();

        bool writeMetadata( const Config& meta );
        
        bool readSlice(const std::string& key, const DateTime& t, std::string& output);

    protected:
        bool purgeDirectory( const std::string& dir );

        bool binValidForReading(bool silent =true);

        bool binValidForWriting(bool silent =false);

        std::string getValidKey(const std::string&);

        void open();

        bool                              _ok;
        bool                              _binPathExists;
        std::string                       _metaPath;       // full path to the bin's metadata file
        std::string                       _binPath;        // full path to the bin's root folder
        osg::ref_ptr<osgDB::ReaderWriter> _rw;
        osg::ref_ptr<osgDB::Options>      _rwOptions;
        Threading::ReadWriteMutex         _rwmutex;
        leveldb::DB*                      _db;

        ReadResult read(const std::string& key, TimeStamp minTime, const Reader& reader);
    };

    void encodeMeta(const Config& meta, std::string& out)
    {
        out = Stringify() << meta.toJSON(false);
    }

    void decodeMeta(const std::string& in, Config& meta)
    {
        std::istringstream inmeta(in);
        inmeta >> std::noskipws;
        std::stringstream buf;
        buf << inmeta.rdbuf();
        std::string bufStr;
        bufStr = buf.str();
        meta.fromJSON( bufStr );
    }
}


//------------------------------------------------------------------------

#undef  LC
#define LC "[LevelDBCache] "

//#undef  OE_DEBUG
//#define OE_DEBUG OE_INFO

#define TIME_FIELD "cache.datetime"

namespace
{

    LevelDBCacheImpl::LevelDBCacheImpl( const CacheOptions& options ) :
        osgEarth::Cache( options ),
        _active        ( true )
    {
        LevelDBCacheOptions fsco( options );
        if ( !fsco.rootPath().isSet() )
        {
            // read the root path from ENV is necessary:
            const char* cachePath = ::getenv(OSGEARTH_ENV_CACHE_PATH);
            if ( cachePath )
            {
                fsco.rootPath() = cachePath;
            }
        }

        if ( fsco.rootPath().isSet() )
        {
            _rootPath = URI( *fsco.rootPath(), options.referrer() ).full();
            init();
        }
        else
        {
            _active = false;
            OE_WARN << LC << "Illegal: no root path set for cache!" << std::endl;
        }
    }

    void
    LevelDBCacheImpl::init()
    {
        // ensure there's a folder for the cache.
        if ( !osgDB::fileExists(_rootPath) )
        {
            if (osgDB::makeDirectory( _rootPath ) == false)
            {
                OE_WARN << LC << "Oh no, failed to create root cache folder \"" << _rootPath << "\""
                   << std::endl;
                _active = false;
            }
        }
    }

    CacheBin*
    LevelDBCacheImpl::addBin( const std::string& name )
    {
        return _active ?
            _bins.getOrCreate(name, new LevelDBCacheBin( name, _rootPath )) :
            0L;
    }

    CacheBin*
    LevelDBCacheImpl::getOrCreateDefaultBin()
    {
        if ( !_active )
            return 0L;

        static Threading::Mutex s_defaultBinMutex;
        if ( !_defaultBin.valid() )
        {
            Threading::ScopedMutexLock lock( s_defaultBinMutex );
            if ( !_defaultBin.valid() ) // double-check
            {
                _defaultBin = new LevelDBCacheBin("__default", _rootPath);
            }
        }
        return _defaultBin.get();
    }

    //------------------------------------------------------------------------


    std::string
    LevelDBCacheBin::getValidKey(const std::string& key)
    {
        return key;
    }

    bool
    LevelDBCacheBin::binValidForReading(bool silent)
    {
        bool ok = _db != 0L;
        if ( !ok && !silent )
        {
            OE_WARN << LC << "Failed to locate cache bin at [" << _binPath << "]" << std::endl;
        }
        return ok;
    }

    bool
    LevelDBCacheBin::binValidForWriting(bool silent)
    {
        bool ok = _db != 0L;
        if ( !ok && !silent )
        {
            OE_WARN << LC << "Failed to locate cache bin at [" << _binPath << "]" << std::endl;
        }
        return ok;
    }

    std::string dataKey(const std::string& key)
    {
        return "data!" + key;
    }

    std::string dataStart()
    {
        return "data!"; // + getID() + "!";
    }

    std::string dataEnd()
    {
        return "data!\xff";
    }

    std::string metaKey(const std::string& key)
    {
        return "meta!" + key;
    }

    std::string metaStart()
    {
        return "meta!";
    }

    std::string metaEnd()
    {
        return "meta!\xff";
    }

    std::string timeKey(const DateTime& t, const std::string& key)
    {
        return "time!" + t.asISO8601() + "!" + key;
    }

    std::string timeStart()
    {
        return "time!";
    }

    std::string timeEnd()
    {
        return "time!\xff";
    }

    LevelDBCacheBin::LevelDBCacheBin(const std::string& binID,
                                     const std::string& rootPath) :
      osgEarth::CacheBin( binID ),
      _binPathExists    ( false ),
      _db               ( 0L )
    {
        std::string name = osgEarth::toLegalFileName(binID);

        _binPath  = osgDB::concatPaths( rootPath, name );
        _metaPath = osgDB::concatPaths( rootPath, name + ".json" );

        _rw = osgDB::Registry::instance()->getReaderWriterForExtension( "osgb" );

        CachePolicy::NO_CACHE.apply(_rwOptions.get());

        open();
    }

    LevelDBCacheBin::~LevelDBCacheBin()
    {
        if (_db)
        {
           delete _db;
           _db = 0L;
        }
    }

    void
    LevelDBCacheBin::open()
    {
        leveldb::Options options;
        options.create_if_missing = true;
        leveldb::Status status = leveldb::DB::Open(options, _binPath, &_db);
        if ( !status.ok() )
        {
            OE_WARN << LC << "Failed to open or create cache bin at " << _binPath << std::endl;
            if ( _db )
            {
                delete _db;
                _db = 0L;
            }
        }
    }

    ReadResult
    LevelDBCacheBin::readImage(const std::string& key, TimeStamp minTime) {
        return read(key, minTime, ImageReader(_rw.get(), _rwOptions.get()));
    }

    ReadResult
    LevelDBCacheBin::readObject(const std::string& key, TimeStamp minTime) {
        return read(key, minTime, ObjectReader(_rw.get(), _rwOptions.get()));
    }

    ReadResult
    LevelDBCacheBin::readNode(const std::string& key, TimeStamp minTime) {
        return read(key, minTime, NodeReader(_rw.get(), _rwOptions.get()));
    }

    ReadResult
    LevelDBCacheBin::read(const std::string& key, TimeStamp minTime, const Reader& reader)
    {
        if ( !binValidForReading() ) 
            return ReadResult(ReadResult::RESULT_NOT_FOUND);

        Config metadata;
        leveldb::Status status;
        leveldb::ReadOptions ro;

        // first read the metadata record.
        std::string metavalue;
        status = _db->Get( ro, metaKey(key), &metavalue );
        if ( status.ok() )
        {
            decodeMeta(metavalue, metadata);

            // Check for expiration:
            TimeStamp minValidTime = std::max(minTime, getMinValidTime());
            if ( minValidTime > 0 )
            {
                DateTime t( metadata.value(TIME_FIELD) );
                if ( t.asTimeStamp() < minValidTime )
                {
                    OE_DEBUG << LC << "Tile " << key << " found but expired!" << std::endl;
                    return ReadResult(ReadResult::RESULT_EXPIRED);
                }
            }
        }
        
        // next read the data record.
        std::string datakey = dataKey(key);
        std::string datavalue;
        status = _db->Get( ro, datakey, &datavalue );
        if ( !status.ok() )
        {
            // main record not found for some reason.
            return ReadResult(ReadResult::RESULT_NOT_FOUND);
        }

        // finally, decode the OSGB stream into an object.
        std::istringstream datastream(datavalue);
        osgDB::ReaderWriter::ReadResult r = reader.read(datastream);
        if ( !r.success() )
        {
            return ReadResult(ReadResult::RESULT_READER_ERROR);
        }

        OE_DEBUG << LC << "Read (" << key << ") from cache bin " << getID() << std::endl;
        return ReadResult( r.getObject(), metadata );
    }

    ReadResult
    LevelDBCacheBin::readString(const std::string& key, TimeStamp minTime)
    {
        ReadResult r = readObject(key, minTime);
        if ( r.succeeded() )
        {
            if ( r.get<StringObject>() )
                return r;
            else
                return ReadResult();
        }
        else
        {
            return r;
        }
    }

    bool
    LevelDBCacheBin::write(const std::string& key, const osg::Object* object, const Config& meta)
    {
        if ( !binValidForWriting() || !object ) 
            return false;
        
        osgDB::ReaderWriter::WriteResult r;
        bool objWriteOK = false;

        std::string       data;
        std::stringstream datastream;

        if ( dynamic_cast<const osg::Image*>(object) )
        {
            r = _rw->writeImage( *static_cast<const osg::Image*>(object), datastream, _rwOptions.get() );
            objWriteOK = r.success();
        }
        else if ( dynamic_cast<const osg::Node*>(object) )
        {
            r = _rw->writeNode( *static_cast<const osg::Node*>(object), datastream, _rwOptions.get() );
            objWriteOK = r.success();
        }
        else
        {
            r = _rw->writeObject( *object, datastream );
            objWriteOK = r.success();
        }

        if (objWriteOK)
        {
            DateTime now;
            leveldb::WriteBatch batch;

            // write the data:
            data = datastream.str();
            batch.Put( dataKey(key), data );

            // write the timestamp index:
            batch.Put( timeKey(now, key), key );

            // write the metadata:
            Config metadata(meta);
            metadata.set( TIME_FIELD, now.asISO8601() );
            encodeMeta( metadata, data );
            batch.Put( metaKey(key), data );

            objWriteOK = _db->Write( leveldb::WriteOptions(), &batch ).ok();

            if ( objWriteOK )
            {
                OE_DEBUG << LC << "Wrote (" << dataKey(key) << ") to cache bin " << getID() << std::endl;
            }
        }

        
        if ( !objWriteOK )
        {
            OE_WARN << LC << "FAILED to write \"" << key << "\" to cache bin " << getID()
                << "; msg = \"" << r.message() << "\"" << std::endl;
        }

        return objWriteOK;
    }

    CacheBin::RecordStatus
    LevelDBCacheBin::getRecordStatus(const std::string& key, TimeStamp minTime)
    {
        if ( !binValidForReading() ) 
            return STATUS_NOT_FOUND;

        leveldb::Status status;
        leveldb::ReadOptions ro;

        // read the metadata record.
        std::string metavalue;
        status = _db->Get( ro, metaKey(key), &metavalue );
        if ( status.ok() )
        {
            // Check for expiration:
            TimeStamp minValidTime = std::max(minTime, getMinValidTime());
            if ( minValidTime > 0 )
            {
                Config metadata;
                decodeMeta(metavalue, metadata);
                DateTime t( metadata.value(TIME_FIELD) );
                if ( t.asTimeStamp() < minValidTime )
                {
                    return STATUS_EXPIRED;
                }
            }
            return STATUS_OK;
        }
        else
        {
            return STATUS_NOT_FOUND;
        }
    }

    bool
    LevelDBCacheBin::remove(const std::string& key)
    {
        if ( !binValidForReading() )
           return false;

        // first read in the time from the metadata record.
        std::string metavalue;
        if ( _db->Get(leveldb::ReadOptions(), metaKey(key), &metavalue).ok() == false )
            return false;

        Config metadata;
        decodeMeta(metavalue, metadata);
        DateTime t(metadata.value(TIME_FIELD));

        leveldb::WriteBatch batch;
        batch.Delete( dataKey(key) );
        batch.Delete( metaKey(key) );
        batch.Delete( timeKey(t, key) );
        
        leveldb::Status status = _db->Write(leveldb::WriteOptions(), &batch);
        if ( !status.ok() )
        {
            OE_WARN << LC << "Failed to remove (" << key << ") from cache" << std::endl;
            return false;
        }

        return true;
    }

    bool
    LevelDBCacheBin::touch(const std::string& key)
    {
        if ( !binValidForWriting() )
           return false;

        // first read in the time from the metadata record.
        std::string metavalue;
        if ( _db->Get(leveldb::ReadOptions(), metaKey(key), &metavalue).ok() == false )
            return false;

        Config metadata;
        decodeMeta(metavalue, metadata);
        DateTime oldtime(metadata.value(TIME_FIELD));
        
        leveldb::WriteBatch batch;

        // In a transaction, update the metadata record with the current time.
        std::string newtime = DateTime().asISO8601();
        metadata.set(TIME_FIELD, newtime);
        encodeMeta(metadata, metavalue);
        batch.Put(metaKey(key), metavalue);

        // ...remove the old time index record:
        batch.Delete( timeKey(oldtime, key) );

        // ...and write a new time index record.
        batch.Put( timeKey(newtime, key), key );

        leveldb::Status status = _db->Write(leveldb::WriteOptions(), &batch);
        if ( !status.ok() )
        {
            OE_WARN << LC << "Failed to touch (" << key << ")" << std::endl;
        }
        return status.ok();
    }

    bool
    LevelDBCacheBin::purge()
    {
        if ( !binValidForWriting() )
           return false;

        delete _db;
        _db = 0L;

        leveldb::Status status = leveldb::DestroyDB( _binPath, leveldb::Options() );
        if ( !status.ok() )
        {
            OE_WARN << LC << "Failed to purge (" << _binPath << ")" << std::endl;
            return false;
        }

        // reopen it.
        open();

        return true;
    }

    bool
    LevelDBCacheBin::compact()
    {
        if ( !binValidForWriting() )
           return false;

        // This could take a while.
        _db->CompactRange(0L, 0L);

        return false;
    }

    unsigned
    LevelDBCacheBin::getStorageSize()
    {
        if ( !binValidForReading() )
           return false;

        leveldb::Range ranges[3];
        uint64_t       sizes[3];

        ranges[0].start = dataStart(), ranges[0].limit = dataEnd();
        ranges[1].start = metaStart(), ranges[1].limit = metaEnd();
        ranges[2].start = timeStart(), ranges[2].limit = timeEnd();
        sizes[0] = sizes[1] = sizes[2] = 0;

        _db->GetApproximateSizes( ranges, 3, sizes );
        return sizes[0] + sizes[1] + sizes[2];
    }

    Config
    LevelDBCacheBin::readMetadata()
    {
        if ( !binValidForReading() )
           return Config();

        ScopedReadLock sharedLock( _rwmutex );
        
        Config conf;
        conf.fromJSON( URI(_metaPath).getString(_rwOptions.get()) );

        return conf;
    }

    bool
    LevelDBCacheBin::writeMetadata( const Config& conf )
    {
        if ( !binValidForWriting() ) return false;

        ScopedWriteLock exclusiveLock( _rwmutex );

        // inject the cache version
        Config mutableConf(conf);
        mutableConf.set("leveldb.cache_version", LEVELDB_CACHE_VERSION);

        std::fstream output( _metaPath.c_str(), std::ios_base::out );
        if ( output.is_open() )
        {
            output << mutableConf.toJSON(true);
            output.flush();
            output.close();
            return true;
        }
        return false;
    }
}

//------------------------------------------------------------------------

/**
 * This driver defers loading of the source data to the appropriate OSG plugin. You
 * must explicity set an override profile when using this driver.
 *
 * For example, use this driver to load a simple jpeg file; then set the profile to
 * tell osgEarth its projection.
 */
class LevelDBCacheDriver : public osgEarth::CacheDriver
{
public:
    LevelDBCacheDriver()
    {
        supportsExtension( "osgearth_cache_leveldb", "leveldb cache for osgEarth" );
    }

    virtual const char* className()
    {
        return "leveldb cache for osgEarth";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return ReadResult( new LevelDBCacheImpl( getCacheOptions(options) ) );
    }
};

REGISTER_OSGPLUGIN(osgearth_cache_LevelDB, LevelDBCacheDriver)
