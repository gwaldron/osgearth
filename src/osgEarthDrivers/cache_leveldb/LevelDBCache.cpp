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
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>
#include <osgEarth/Cache>
#include <osgEarth/StringUtils>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/URI>
#include <osgEarth/Registry>
#include <leveldb/db.h>
#include <string>

using namespace osgEarth;
using namespace osgEarth::Threading;
using namespace osgEarth::Drivers::LevelDBCache;

namespace
{
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

        Config readMetadata();

        bool writeMetadata( const Config& meta );

    protected:
        bool purgeDirectory( const std::string& dir );

        bool binValidForReading(bool silent =true);

        bool binValidForWriting(bool silent =false);

        std::string getValidKey(const std::string&);

        bool                              _ok;
        bool                              _binPathExists;
        std::string                       _metaPath;       // full path to the bin's metadata file
        std::string                       _binPath;        // full path to the bin's root folder
        osg::ref_ptr<osgDB::ReaderWriter> _rw;
        osg::ref_ptr<osgDB::Options>      _rwOptions;
        Threading::ReadWriteMutex         _rwmutex;
        leveldb::DB*                      _db;
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

namespace
{
    LevelDBCacheImpl::LevelDBCacheImpl( const CacheOptions& options ) :
      osgEarth::Cache( options )
    {
        LevelDBCacheOptions fsco( options );
        _rootPath = URI( *fsco.rootPath(), options.referrer() ).full();
        init();
    }

    void
    LevelDBCacheImpl::init()
    {
        //nop
    }

    CacheBin*
    LevelDBCacheImpl::addBin( const std::string& name )
    {
        return _bins.getOrCreate(name, new LevelDBCacheBin( name, _rootPath ));
    }

    CacheBin*
    LevelDBCacheImpl::getOrCreateDefaultBin()
    {
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
#if 0
#ifdef OSGEARTH_HAVE_ZLIB
        _rwOptions = Registry::instance()->cloneOrCreateOptions();
        _rwOptions->setOptionString( "Compressor=zlib" );
#endif
#endif
        CachePolicy::NO_CACHE.apply(_rwOptions.get());

        leveldb::Options options;
        options.create_if_missing = true;
        leveldb::Status status = leveldb::DB::Open(options, _binPath, &_db);
        if ( !status.ok() )
        {
           OE_WARN << LC << "Failed to open or create cache bin at " << _binPath << std::endl;
           if ( _db )
              delete _db;
        }
    }

    LevelDBCacheBin::~LevelDBCacheBin()
    {
        if (_db)
           delete _db;
    }

    ReadResult
    LevelDBCacheBin::readImage(const std::string& key, TimeStamp minTime)
    {
        if ( !binValidForReading() ) 
            return ReadResult(ReadResult::RESULT_NOT_FOUND);

        std::string value;
        leveldb::Status status = _db->Get(leveldb::ReadOptions(), key, &value);

        if (!status.ok())
            return ReadResult(ReadResult::RESULT_NOT_FOUND);

        //TODO: expiry.
        //if ( osgEarth::getLastModifiedTime(path) < std::max(minTime, getMinValidTime()) )
        //    return ReadResult( ReadResult::RESULT_EXPIRED );

        std::istringstream input(value);
        osgDB::ReaderWriter::ReadResult r = _rw->readImage(input, _rwOptions.get());
        if ( !r.success() )
           return ReadResult();

        // read metadata if it exists.
        Config meta;
        
        std::string metakey = key + ".meta";
        status = _db->Get(leveldb::ReadOptions(), metakey, &value);
        if (status.ok())
        {
           decodeMeta(value, meta);
        }

        return ReadResult( r.getImage(), meta );
    }

    ReadResult
    LevelDBCacheBin::readObject(const std::string& key, TimeStamp minTime)
    {
        if ( !binValidForReading() ) 
            return ReadResult(ReadResult::RESULT_NOT_FOUND);

        std::string value;
        leveldb::Status status = _db->Get(leveldb::ReadOptions(), key, &value);

        if (!status.ok())
            return ReadResult(ReadResult::RESULT_NOT_FOUND);

        //TODO: expiry.
        //if ( osgEarth::getLastModifiedTime(path) < std::max(minTime, getMinValidTime()) )
        //    return ReadResult( ReadResult::RESULT_EXPIRED );

        std::istringstream input(value);
        osgDB::ReaderWriter::ReadResult r = _rw->readObject(input, _rwOptions.get());
        if ( !r.success() )
           return ReadResult();

        // read metadata if it exists.
        Config meta;
        
        std::string metakey = key + ".meta";
        status = _db->Get(leveldb::ReadOptions(), metakey, &value);
        if (status.ok())
        {
           decodeMeta(value, meta);
        }

        return ReadResult( r.getObject(), meta );
    }

    ReadResult
    LevelDBCacheBin::readNode(const std::string& key, TimeStamp minTime)
    {
        if ( !binValidForReading() ) 
            return ReadResult(ReadResult::RESULT_NOT_FOUND);

        std::string value;
        leveldb::Status status = _db->Get(leveldb::ReadOptions(), key, &value);

        if (!status.ok())
            return ReadResult(ReadResult::RESULT_NOT_FOUND);

        //TODO: expiry.
        //if ( osgEarth::getLastModifiedTime(path) < std::max(minTime, getMinValidTime()) )
        //    return ReadResult( ReadResult::RESULT_EXPIRED );

        std::istringstream input(value);
        osgDB::ReaderWriter::ReadResult r = _rw->readNode(input, _rwOptions.get());
        if ( !r.success() )
           return ReadResult();

        // read metadata if it exists.
        Config meta;
        
        std::string metakey = key + ".meta";
        status = _db->Get(leveldb::ReadOptions(), metakey, &value);
        if (status.ok())
        {
           decodeMeta(value, meta);
        }

        return ReadResult( r.getNode(), meta );
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
    LevelDBCacheBin::write( const std::string& key, const osg::Object* object, const Config& meta )
    {
        if ( !binValidForWriting() || !object ) 
            return false;
        
        osgDB::ReaderWriter::WriteResult r;
        bool objWriteOK = false;

        std::string       value;
        std::stringstream buf;

         if ( dynamic_cast<const osg::Image*>(object) )
         {
               r = _rw->writeImage( *static_cast<const osg::Image*>(object), buf, _rwOptions.get() );
               objWriteOK = r.success();
         }
         else if ( dynamic_cast<const osg::Node*>(object) )
         {
               r = _rw->writeNode( *static_cast<const osg::Node*>(object), buf, _rwOptions.get() );
               objWriteOK = r.success();
         }
         else
         {
               r = _rw->writeObject( *object, buf );
               objWriteOK = r.success();
         }

         if (objWriteOK)
         {
            value = buf.str();
            leveldb::Status status = _db->Put(leveldb::WriteOptions(), key, value);
            objWriteOK = status.ok();

            // write metadata
            if ( !meta.empty() && objWriteOK )
            {
                std::string metakey = key + ".meta";
                encodeMeta( metakey, value );
                _db->Put(leveldb::WriteOptions(), metakey, value);
            }
        }

        if ( objWriteOK )
        {
            OE_DEBUG << LC << "Wrote \"" << key << "\" to cache bin " << getID() << std::endl;
        }
        else
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

        std::string value;
        leveldb::Status status = _db->Get(leveldb::ReadOptions(), key, &value);
        if ( !status.ok() )
           return STATUS_NOT_FOUND;

#if 0 // TODO: expiry.
        TimeStamp oldestValidTime = std::max(minTime, getMinValidTime());
        struct stat s;
        ::stat( path.c_str(), &s );
        return s.st_mtime >= oldestValidTime ? STATUS_OK : STATUS_EXPIRED;
#else
        return STATUS_OK;
#endif
    }

    bool
    LevelDBCacheBin::remove(const std::string& key)
    {
        if ( !binValidForReading() )
           return false;

        leveldb::Status status = _db->Delete(leveldb::WriteOptions(), key);
        return status.ok();
    }

    bool
    LevelDBCacheBin::touch(const std::string& key)
    {
        // not yet implemented
        return false;
    }

    bool
    LevelDBCacheBin::purge()
    {
        if ( !binValidForReading() )
           return false;

        // NYI.
        return false;
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

        std::fstream output( _metaPath.c_str(), std::ios_base::out );
        if ( output.is_open() )
        {
            output << conf.toJSON(true);
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
