/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include "FileSystemCache"
#include <osgEarth/Cache>
#include <osgEarth/StringUtils>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/XmlUtils>
#include <osgEarth/URI>
#include <osgEarth/FileUtils>
#include <osgEarth/StringUtils>
#include <osgEarth/Registry>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <fstream>
#include <sys/stat.h>

using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace osgEarth::Threading;

#ifndef _WIN32
#   include <unistd.h>
#endif

#define OSG_FORMAT "osgb"
#define OSG_EXT   ".osgb"
#define OSG_COMPRESS

namespace
{
    /** 
     * Cache that stores data in the local file system.
     */
    class FileSystemCache : public Cache
    {
    public:
        FileSystemCache() { } // unused
        FileSystemCache( const FileSystemCache& rhs, const osg::CopyOp& op ) { } // unused
        META_Object( osgEarth, FileSystemCache );

        /**
         * Constructs a new file system cache.
         * @param options Options structure that comes from a serialized description of 
         *        the object.
         */
        FileSystemCache( const CacheOptions& options );

    public: // Cache interface

        CacheBin* addBin( const std::string& binID );

        CacheBin* getOrCreateDefaultBin();

    protected:

        void init();

        std::string _rootPath;
    };

    /** 
     * Cache bin implementation for a FileSystemCache.
     * You don't need to create this object directly; use FileSystemCache::createBin instead.
    */
    class FileSystemCacheBin : public CacheBin
    {
    public:
        FileSystemCacheBin( const std::string& name, const std::string& rootPath );

    public: // CacheBin interface

        ReadResult readObject(const std::string& key, const osgDB::Options* dbo);

        ReadResult readImage(const std::string& key, const osgDB::Options* dbo);

        //ReadResult readNode(const std::string& key, const osgDB::Options* dbo);

        ReadResult readString(const std::string& key, const osgDB::Options* dbo);

        bool write(const std::string& key, const osg::Object* object, const Config& meta, const osgDB::Options* dbo);

        bool remove(const std::string& key);

        bool touch(const std::string& key);

        RecordStatus getRecordStatus(const std::string& key);

        bool clear();

        Config readMetadata();

        bool writeMetadata( const Config& meta );

        std::string getHashedKey(const std::string&) const;

    protected:
        bool purgeDirectory( const std::string& dir );

        bool binValidForReading(bool silent =true);

        bool binValidForWriting(bool silent =false);

        const osgDB::Options* mergeOptions(const osgDB::Options* in);

        bool                              _ok;
        bool                              _binPathExists;
        std::string                       _metaPath;       // full path to the bin's metadata file
        std::string                       _binPath;        // full path to the bin's root folder
        osg::ref_ptr<osgDB::ReaderWriter> _rw;
        osg::ref_ptr<osgDB::Options>      _zlibOptions;
        mutable Threading::Mutex          _mutex;
    };

    void writeMeta( const std::string& fullPath, const Config& meta )
    {
        std::ofstream outmeta( fullPath.c_str() );
        if ( outmeta.is_open() )
        {
            outmeta << meta.toJSON();
            outmeta.flush();
            outmeta.close();
        }
    }

    void readMeta( const std::string& fullPath, Config& meta )
    {
        std::ifstream inmeta( fullPath.c_str() );
        if ( inmeta.is_open() )
        {
            inmeta >> std::noskipws;
            std::stringstream buf;
            buf << inmeta.rdbuf();
            std::string bufStr;
            bufStr = buf.str();
            meta.fromJSON( bufStr );
        }
    }
}


//------------------------------------------------------------------------

#undef  LC
#define LC "[FileSystemCache] "

//#undef  OE_DEBUG
//#define OE_DEBUG OE_INFO

namespace
{
    FileSystemCache::FileSystemCache( const CacheOptions& options ) :
    Cache( options )
    {
        FileSystemCacheOptions fsco( options );

        // read the root path from ENV is necessary:
        if ( !fsco.rootPath().isSet())
        {           
            const char* cachePath = ::getenv(OSGEARTH_ENV_CACHE_PATH);
            if ( cachePath )
                fsco.rootPath() = cachePath;
        }

        _rootPath = URI( *fsco.rootPath(), options.referrer() ).full();
        init();
    }

    void
    FileSystemCache::init()
    {
        OE_INFO << LC << "Opened a filesystem cache at \"" << _rootPath << "\"\n";
    }

    CacheBin*
    FileSystemCache::addBin( const std::string& name )
    {
        return _bins.getOrCreate( name, new FileSystemCacheBin( name, _rootPath ) );
    }

    CacheBin*
    FileSystemCache::getOrCreateDefaultBin()
    {
        static Threading::Mutex s_defaultBinMutex;
        if ( !_defaultBin.valid() )
        {
            Threading::ScopedMutexLock lock( s_defaultBinMutex );
            if ( !_defaultBin.valid() ) // double-check
            {
                _defaultBin = new FileSystemCacheBin( "__default", _rootPath );
            }
        }
        return _defaultBin.get();
    }

    //------------------------------------------------------------------------

    std::string
    FileSystemCacheBin::getHashedKey(const std::string& key) const
    {
        if ( getHashKeys() )
        {
            unsigned hash = osgEarth::hashString(key);
            unsigned b1 = (hash & 0xfff00000) >> 20;
            unsigned b2 = (hash & 0x000fff00) >> 8;
            unsigned b3 = (hash & 0x000000ff);
            return Stringify() << std::hex << std::setfill('0') << std::setw(3) << b1 << "/" << b2 << "/" << std::setw(2) << b3;
        }
        else
        {
            return osgEarth::toLegalFileName(key);
        }
    }

    bool
    FileSystemCacheBin::binValidForReading(bool silent)
    {
        if ( !_rw.valid() )
        {
            _ok = false;
        }
        else if ( !_binPathExists )
        {
            if ( osgDB::fileExists(_binPath) )
            {
                // ready to go
                _binPathExists = true;
                _ok = true;
            }
            else if ( _ok )
            {
                // one-time error.
                if ( !silent )
                {
                    OE_WARN << LC << "Failed to locate cache bin at [" << _binPath << "]" << std::endl;
                }
                _ok = false;
            }
        }

        return _ok;
    }

    bool
    FileSystemCacheBin::binValidForWriting(bool silent)
    {
        if ( !_rw.valid() )
        {
            _ok = false;
        }
        else if ( !_binPathExists )
        {
            osgEarth::makeDirectoryForFile( _metaPath );

            if ( osgDB::fileExists(_binPath) )
            {
                // ready to go
                _binPathExists = true;
                _ok = true;
            }
            else
            {
                // one-time error.
                if ( !silent )
                {
                    OE_WARN << LC << "FAILED to find or create cache bin at [" << _metaPath << "]" << std::endl;
                }
                _ok = false;
            }
        }

        return _ok;
    }

    FileSystemCacheBin::FileSystemCacheBin(const std::string&   binID,
                                           const std::string&   rootPath) :
    CacheBin            ( binID ),
    _binPathExists      ( false ),
    _ok( true )
    {
        _binPath = osgDB::concatPaths( rootPath, binID );
        _metaPath = osgDB::concatPaths( _binPath, "osgearth_cacheinfo.json" );

        _rw = osgDB::Registry::instance()->getReaderWriterForExtension(OSG_FORMAT);

#ifdef OSG_COMPRESS
#ifdef OSGEARTH_HAVE_ZLIB
        _zlibOptions = Registry::instance()->cloneOrCreateOptions();
        _zlibOptions->setPluginStringData("Compressor", "zlib");
#endif        
#endif
    }

    const osgDB::Options*
    FileSystemCacheBin::mergeOptions(const osgDB::Options* dbo)
    {
        if (!dbo)
        {
            return _zlibOptions.get();
        }
        else if (!_zlibOptions.valid())
        {
            return dbo;
        }
        else
        {
            osgDB::Options* merged = Registry::cloneOrCreateOptions(dbo);
            merged->setPluginStringData("Compressor", "zlib");
            return merged;
        }
    }

    ReadResult
    FileSystemCacheBin::readImage(const std::string& key, const osgDB::Options* readOptions)
    {
        if ( !binValidForReading() ) 
            return ReadResult(ReadResult::RESULT_NOT_FOUND);

        // mangle "key" into a legal path name
        URI fileURI( getHashedKey(key), _metaPath );
        std::string path = fileURI.full() + OSG_EXT;

        if ( !osgDB::fileExists(path) )
            return ReadResult( ReadResult::RESULT_NOT_FOUND );

        osgEarth::TimeStamp timeStamp = osgEarth::getLastModifiedTime(path);     

        osg::ref_ptr<const osgDB::Options> dbo = mergeOptions(readOptions);

        osgDB::ReaderWriter::ReadResult r;
        {
            ScopedMutexLock lock(_mutex);

            r = _rw->readImage( path, dbo.get() );
            if ( !r.success() )
                return ReadResult();

            // read metadata
            Config meta;
            std::string metafile = fileURI.full() + ".meta";
            if ( osgDB::fileExists(metafile) )
                readMeta( metafile, meta );

            ReadResult rr( r.getImage(), meta );
            rr.setLastModifiedTime(timeStamp);
            return rr;            
        }
    }

    ReadResult
    FileSystemCacheBin::readObject(const std::string& key, const osgDB::Options* readOptions)
    {
        if ( !binValidForReading() ) 
            return ReadResult(ReadResult::RESULT_NOT_FOUND);

        // mangle "key" into a legal path name
        URI fileURI( getHashedKey(key), _metaPath );
        std::string path = fileURI.full() + OSG_EXT;

        if ( !osgDB::fileExists(path) )
            return ReadResult( ReadResult::RESULT_NOT_FOUND );

        osgEarth::TimeStamp timeStamp = osgEarth::getLastModifiedTime(path);

        osg::ref_ptr<const osgDB::Options> dbo = mergeOptions(readOptions);

        osgDB::ReaderWriter::ReadResult r;
        {
            ScopedMutexLock lock(_mutex);

            r = _rw->readObject( path, dbo.get() );
            if ( !r.success() )
                return ReadResult();

            // read metadata
            Config meta;
            std::string metafile = fileURI.full() + ".meta";
            if ( osgDB::fileExists(metafile) )
                readMeta( metafile, meta );

            ReadResult rr( r.getObject(), meta );
            rr.setLastModifiedTime(timeStamp);
            return rr;            
        }
    }

    ReadResult
    FileSystemCacheBin::readString(const std::string& key, const osgDB::Options* readOptions)
    {
        ReadResult r = readObject(key, readOptions);
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
    FileSystemCacheBin::write(const std::string& key, const osg::Object* object, const Config& meta, const osgDB::Options* writeOptions)
    {
        if ( !binValidForWriting() || !object ) 
            return false;

        // convert the key into a legal filename:
        URI fileURI( getHashedKey(key), _metaPath );
        
        osgDB::ReaderWriter::WriteResult r;

        bool objWriteOK = false;
        {
            // prevent cache contention:
            ScopedMutexLock lock(_mutex);

            // make a home for it..
            if ( !osgDB::fileExists( osgDB::getFilePath(fileURI.full()) ) )
                osgEarth::makeDirectoryForFile( fileURI.full() );

            osg::ref_ptr<const osgDB::Options> dbo = mergeOptions(writeOptions);

            if ( dynamic_cast<const osg::Image*>(object) )
            {
                std::string filename = fileURI.full() + OSG_EXT;
                r = _rw->writeImage( *static_cast<const osg::Image*>(object), filename, dbo.get() );
                objWriteOK = r.success();
            }
            else if ( dynamic_cast<const osg::Node*>(object) )
            {
                std::string filename = fileURI.full() + OSG_EXT;
                r = _rw->writeNode(*static_cast<const osg::Node*>(object), filename, dbo.get());
                objWriteOK = r.success();
            }
            else
            {
                std::string filename = fileURI.full() + OSG_EXT;
                r = _rw->writeObject(*object, filename, dbo.get());
                objWriteOK = r.success();
            }

            // write metadata
            if ( !meta.empty() && objWriteOK )
            {
                std::string metaname = fileURI.full() + ".meta";
                writeMeta( metaname, meta );
            }
        }

        if ( objWriteOK )
        {
            OE_DEBUG << LC << "Wrote \"" << key << "\" to cache bin [" << getID() << "] path=" << fileURI.full() << "." << OSG_EXT << std::endl;
        }
        else
        {
            OE_WARN << LC << "FAILED to write \"" << key << "\" to cache bin " << getID()
                << "; msg = \"" << r.message() << "\"" << std::endl;
        }

        return objWriteOK;
    }

    CacheBin::RecordStatus
    FileSystemCacheBin::getRecordStatus(const std::string& key)
    {
        if ( !binValidForReading() ) 
            return STATUS_NOT_FOUND;

        URI fileURI( getHashedKey(key), _metaPath );
        std::string path( fileURI.full() + OSG_EXT );
        if ( !osgDB::fileExists(path) )
            return STATUS_NOT_FOUND;

        return STATUS_OK;
    }

    bool
    FileSystemCacheBin::remove(const std::string& key)
    {
        if ( !binValidForReading() ) return false;
        URI fileURI( getHashedKey(key), _metaPath );
        std::string path( fileURI.full() + OSG_EXT );

        ScopedMutexLock lock(_mutex);
        return ::unlink( path.c_str() ) == 0;
    }

    bool
    FileSystemCacheBin::touch(const std::string& key)
    {
        if ( !binValidForReading() ) return false;
        URI fileURI( getHashedKey(key), _metaPath );
        std::string path( fileURI.full() + OSG_EXT );

        ScopedMutexLock lock(_mutex);
        return osgEarth::touchFile( path );
    }

    bool
    FileSystemCacheBin::purgeDirectory( const std::string& dir )
    {
        if ( !binValidForReading() ) return false;

        ScopedMutexLock lock(_mutex);

        bool allOK = true;
        osgDB::DirectoryContents dc = osgDB::getDirectoryContents( dir );

        for( osgDB::DirectoryContents::iterator i = dc.begin(); i != dc.end(); ++i )
        {
            int ok = 0;
            std::string full = osgDB::concatPaths(dir, *i);
            
            if ( full.find( getID() ) != std::string::npos ) // safety latch
            {
                osgDB::FileType type = osgDB::fileType( full );

                if ( type == osgDB::DIRECTORY && i->compare(".") != 0 && i->compare("..") != 0 )
                {
                    purgeDirectory( full );

                    ok = ::unlink( full.c_str() );
                    OE_DEBUG << LC << "Unlink: " << full << std::endl;
                }
                else if ( type == osgDB::REGULAR_FILE )
                {
                    if ( full != _metaPath )
                    {
                        ok = ::unlink( full.c_str() );
                        OE_DEBUG << LC << "Unlink: " << full << std::endl;
                    }
                }

                if ( ok != 0 )
                    allOK = false;
            }
        }

        return allOK;
    }

    bool
    FileSystemCacheBin::clear()
    {
        if ( !binValidForReading() )
            return false;

        ScopedMutexLock lock(_mutex);
        std::string binDir = osgDB::getFilePath( _metaPath );
        return purgeDirectory( binDir );
    }

    Config
    FileSystemCacheBin::readMetadata()
    {
        if ( !binValidForReading() ) return Config();
        
        ScopedMutexLock lock(_mutex);

        Config conf;
        conf.fromJSON( URI(_metaPath).getString(_zlibOptions.get()) );

        return conf;
    }

    bool
    FileSystemCacheBin::writeMetadata( const Config& conf )
    {
        if ( !binValidForWriting() ) return false;
        
        ScopedMutexLock lock(_mutex);

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
class FileSystemCacheDriver : public CacheDriver
{
public:
    FileSystemCacheDriver()
    {
        supportsExtension( "osgearth_cache_filesystem", "File system cache for osgEarth" );
    }

    virtual const char* className() const
    {
        return "File system cache for osgEarth";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return ReadResult( new FileSystemCache( getCacheOptions(options) ) );
    }
};

REGISTER_OSGPLUGIN(osgearth_cache_filesystem, FileSystemCacheDriver)
