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
#include "FileSystemCache"
#include <osgEarth/Cache>
#include <osgEarth/StringUtils>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/XmlUtils>
#include <osgEarth/URI>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <fstream>

using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace osgEarth::Threading;

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

        std::string            _rootPath;
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

        ReadResult readObject( const std::string& key, double maxAge =DBL_MAX );

        ReadResult readImage( const std::string& key, double maxAge =DBL_MAX );

        ReadResult readNode( const std::string& key, double maxAge =DBL_MAX );

        ReadResult readString( const std::string& key, double maxAge =DBL_MAX );

        bool write( const std::string& key, const osg::Object* object, const Config& meta );

        bool writeString( const std::string& key, const std::string& buffer, const Config& meta );

        bool isCached( const std::string& key, double maxAge =DBL_MAX );

        bool purge();

        Config readMetadata();

        bool writeMetadata( const Config& meta );

    protected:
        bool purgeDirectory( const std::string& dir );

        bool                              _ok;
        std::string                       _metaPath;
        osg::ref_ptr<osgDB::ReaderWriter> _rw;
        osg::ref_ptr<osgDB::Options>      _rwOptions;
        Threading::ReadWriteMutex         _rwmutex;
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
        _rootPath = URI( *fsco.rootPath(), options.referrer() ).full();
        init();
    }

    void
    FileSystemCache::init()
    {
        osgDB::makeDirectory( _rootPath );
        if ( !osgDB::fileExists( _rootPath ) )
        {
            OE_WARN << LC << "FAILED to create root folder for cache at \"" << _rootPath << "\"" << std::endl;
            _ok = false;
        }
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

    FileSystemCacheBin::FileSystemCacheBin(const std::string&   binID,
                                           const std::string&   rootPath) :
    CacheBin ( binID ),
    _ok      ( true )
    {
        std::string binPath = osgDB::concatPaths( rootPath, binID );
        _metaPath = osgDB::concatPaths( binPath, "osgearth_cacheinfo.json" );

        OE_INFO << LC << "Initializing cache bin: " << _metaPath << std::endl;
        osgDB::makeDirectoryForFile( _metaPath );
        if ( !osgDB::fileExists( binPath ) )
        {
            OE_WARN << LC << "FAILED to create folder for cache bin at \"" << binPath << "\"" << std::endl;
            _ok = false;
        }
        else
        {
            _rw = osgDB::Registry::instance()->getReaderWriterForExtension( "osgb" );
#ifdef OSGEARTH_HAVE_ZLIB
            _rwOptions = new osgDB::ReaderWriter::Options( "Compressor=zlib" );
#endif
        }
    }

    ReadResult
    FileSystemCacheBin::readImage(const std::string& key, double maxAge)
    {
        if ( !_ok ) return 0L;

        //todo: handle maxAge

        // mangle "key" into a legal path name
        URI fileURI( toLegalFileName(key), _metaPath );

        osgDB::ReaderWriter::ReadResult r;
        {
            ScopedReadLock sharedLock( _rwmutex );
            r = _rw->readImage( fileURI.full() + ".osgb", _rwOptions.get() );
            if ( r.success() )
            {
                // read metadata
                Config meta;
                std::string metafile = fileURI.full() + ".meta";
                if ( osgDB::fileExists(metafile) )
                    readMeta( metafile, meta );

                return ReadResult( r.getImage(), meta );
            }
        }

        return ReadResult(); //error
    }

    ReadResult
    FileSystemCacheBin::readObject(const std::string& key, double maxAge)
    {
        if ( !_ok ) return 0L;

        //todo: handle maxAge

        // mangle "key" into a legal path name
        URI fileURI( toLegalFileName(key), _metaPath );

        osgDB::ReaderWriter::ReadResult r;
        {
            ScopedReadLock sharedLock( _rwmutex );
            r = _rw->readObject( fileURI.full() + ".osgb", _rwOptions.get() );
            if ( r.success() )
            {
                // read metadata
                Config meta;
                std::string metafile = fileURI.full() + ".meta";
                if ( osgDB::fileExists(metafile) )
                    readMeta( metafile, meta );

                // TODO: read metadata
                return ReadResult( r.getObject(), meta );
            }
        }

        return ReadResult();
    }

    ReadResult
    FileSystemCacheBin::readNode(const std::string& key, double maxAge)
    {
        if ( !_ok ) return 0L;

        //todo: handle maxAge

        // mangle "key" into a legal path name
        URI fileURI( toLegalFileName(key), _metaPath );

        osgDB::ReaderWriter::ReadResult r;
        {
            ScopedReadLock sharedLock( _rwmutex );
            r = _rw->readNode( fileURI.full() + ".osgb", _rwOptions.get() );
            if ( r.success() )
            {            
                // read metadata
                Config meta;
                std::string metafile = fileURI.full() + ".meta";
                if ( osgDB::fileExists(metafile) )
                    readMeta( metafile, meta );

                return ReadResult( r.getNode(), meta );
            }
        }

        return ReadResult();
    }

    ReadResult
    FileSystemCacheBin::readString(const std::string& key, double maxAge)
    {
        Config      meta;
        std::string output;

        if ( !_ok ) return 0L;

        //todo: handle maxAge

        //todo: mangle "key" into a legal path name
        URI fileURI( toLegalFileName(key), _metaPath );

        bool readOK = false;
        if ( osgDB::fileExists( fileURI.full() ) )
        {
            ScopedReadLock sharedLock( _rwmutex );
            std::ifstream infile( fileURI.full().c_str() );
            if ( infile.is_open() )
            {
                infile >> std::noskipws;
                std::stringstream buf;
                buf << infile.rdbuf();
		        output = buf.str();
                readOK = true;
            }
            
            // read metadata
            std::string metafile = fileURI.full() + ".meta";
            if ( osgDB::fileExists(metafile) )
            {
                readMeta( metafile, meta );
            }
        }

        if ( output.size() > 0 )
        {
            OE_DEBUG << LC << "Read \"" << key << "\" from cache bin " << getID() << std::endl;
        }
        else
        {
            OE_DEBUG << LC << "Failed to read \"" << key << "\" from cache bin " << getID() << std::endl;
        }

        return readOK ? ReadResult( new StringObject(output), meta ) : ReadResult();
    }

    bool
    FileSystemCacheBin::write( const std::string& key, const osg::Object* object, const Config& meta )
    {
        if ( !_ok || !object ) return false;

        // convert the key into a legal filename:
        URI fileURI( toLegalFileName(key), _metaPath );

        bool objWriteOK = false;
        {
            // prevent cache contention:
            ScopedWriteLock exclusiveLock( _rwmutex );

            // make a home for it..
            if ( !osgDB::fileExists( osgDB::getFilePath(fileURI.full()) ) )
                osgDB::makeDirectoryForFile( fileURI.full() );

            // write it.  
            osgDB::ReaderWriter::WriteResult r;      

            if ( dynamic_cast<const osg::Image*>(object) )
            {
                std::string filename = fileURI.full() + ".osgb";
                r = _rw->writeImage( *static_cast<const osg::Image*>(object), filename, _rwOptions.get() );
                objWriteOK = r.success();
            }
            else if ( dynamic_cast<const osg::Node*>(object) )
            {
                std::string filename = fileURI.full() + ".osgb";
                r = _rw->writeNode( *static_cast<const osg::Node*>(object), filename, _rwOptions.get() );
                objWriteOK = r.success();
            }
            else if ( dynamic_cast<const StringObject*>(object) )
            {
                const StringObject* so = static_cast<const StringObject*>( object );
                std::ofstream outfile( fileURI.full().c_str() );
                if ( outfile.is_open() )
                {
                    outfile << so->getString();
                    outfile.flush();
                    outfile.close();
                    objWriteOK = true;
                }
            }
            else
            {
                std::string filename = fileURI.full() + ".osgb";
                r = _rw->writeObject( *object, filename );
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
            OE_DEBUG << LC << "Wrote \"" << key << "\" to cache bin " << getID() << std::endl;
        }
        else
        {
            OE_WARN << LC << "FAILED to write \"" << key << "\" to cache bin " << getID() << std::endl;
        }

        return objWriteOK;
    }

    bool
    FileSystemCacheBin::writeString(const std::string& key, const std::string& buffer, const Config& meta )
    {
        // convert the key into a legal filename:
        URI fileURI( toLegalFileName(key), _metaPath );

        if ( buffer.size() == 0 )
            return false;

        bool ok = false;
        {
            // prevent cache contention:
            ScopedWriteLock exclusiveLock( _rwmutex );

            // make a home for it..
            if ( !osgDB::fileExists( osgDB::getFilePath(fileURI.full()) ) )
                osgDB::makeDirectoryForFile( fileURI.full() );

            // tack on the extension
            std::string filename = fileURI.full() + ".dat";

            std::ofstream outfile( filename.c_str() );
            if ( outfile.is_open() )
            {
                outfile << buffer;
                outfile.flush();
                outfile.close();
                ok = true;
            }

            if ( !meta.empty() )
            {
                std::string metaname = fileURI.full() + ".meta";
                writeMeta( metaname, meta );
            }
        }

        if ( ok )
        {
            OE_DEBUG << LC << "Wrote \"" << key << "\" to cache bin " << getID() << std::endl;
        }
        else
        {
            OE_WARN << LC << "FAILED to write \"" << key << "\" to cache bin "
                << getID() <<  std::endl;
        }

        return ok;
    }

    bool
    FileSystemCacheBin::isCached( const std::string& key, double maxAge )
    {
        if ( !_ok ) return false;

        URI fileURI( toLegalFileName(key), _metaPath );
        return osgDB::fileExists( fileURI.full() + ".osgb" );
    }

    bool
    FileSystemCacheBin::purgeDirectory( const std::string& dir )
    {
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
    FileSystemCacheBin::purge()
    {
        if ( !_ok ) return false;
        {
            ScopedWriteLock exclusiveLock( _rwmutex );
            std::string binDir = osgDB::getFilePath( _metaPath );
            return purgeDirectory( binDir );
        }
    }

    Config
    FileSystemCacheBin::readMetadata()
    {
        if ( !_ok ) return Config();

        ScopedReadLock sharedLock( _rwmutex );
        
        Config conf;
        conf.fromJSON( URI(_metaPath).readString(0L,CachePolicy::NO_CACHE).getString() );

        return conf;
    }

    bool
    FileSystemCacheBin::writeMetadata( const Config& conf )
    {
        if ( !_ok ) return false;

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
class FileSystemCacheDriver : public CacheDriver
{
public:
    FileSystemCacheDriver()
    {
        supportsExtension( "osgearth_cache_filesystem", "File system cache for osgEarth" );
    }

    virtual const char* className()
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
