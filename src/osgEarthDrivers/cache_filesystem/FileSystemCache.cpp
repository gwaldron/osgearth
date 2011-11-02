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

        bool readObject( osg::ref_ptr<osg::Object>& output, const std::string& key, double maxAge =DBL_MAX );

        bool readImage( osg::ref_ptr<osg::Image>& output, const std::string& key, double maxAge =DBL_MAX );

        bool readString( std::string& output, const std::string& key, double maxAge =DBL_MAX );

        bool write( const std::string& key, const osg::Object* object );

        bool write( const std::string& key, const std::string& buffer );

        bool isCached( const std::string& key, double maxAge =DBL_MAX );

        Config readMetadata();

        bool writeMetadata( const Config& meta );

    protected:
        bool                              _ok;
        std::string                       _metaPath;
        osg::ref_ptr<osgDB::ReaderWriter> _rw;
        osg::ref_ptr<osgDB::Options>      _rwOptions;
        Threading::ReadWriteMutex         _rwmutex;
    };
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
        _metaPath = osgDB::concatPaths( binPath, "osgearth_cacheinfo.xml" );

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

    bool
    FileSystemCacheBin::readImage(osg::ref_ptr<osg::Image>& output,
                                  const std::string&        key,
                                  double                    maxAge )
    {
        if ( !_ok ) return 0L;

        //todo: handle maxAge

        //todo: mangle "key" into a legal path name
        URI fileURI( toLegalFileName(key), _metaPath );

        osgDB::ReaderWriter::ReadResult r;
        {
            ScopedReadLock sharedLock( _rwmutex );
            r = _rw->readImage( fileURI.full() + ".osgb", _rwOptions.get() );
        }

        if ( r.getImage() )
        {
            OE_DEBUG << LC << "Read \"" << key << "\" from cache bin " << getID() << std::endl;
        }
        else
        {
            OE_DEBUG << LC << "Failed to read \"" << key << "\" from cache bin " << getID() << std::endl;
        }

        output = r.getImage();
        return output.valid();
    }

    bool
    FileSystemCacheBin::readObject(osg::ref_ptr<osg::Object>& output,
                                   const std::string&         key,
                                   double                     maxAge )
    {
        if ( !_ok ) return 0L;

        //todo: handle maxAge

        //todo: mangle "key" into a legal path name
        URI fileURI( toLegalFileName(key), _metaPath );

        osgDB::ReaderWriter::ReadResult r;
        {
            ScopedReadLock sharedLock( _rwmutex );
            r = _rw->readObject( fileURI.full() + ".osgb", _rwOptions.get() );
        }

        if ( r.getObject() )
        {
            OE_DEBUG << LC << "Read \"" << key << "\" from cache bin " << getID() << std::endl;
        }
        else
        {
            OE_DEBUG << LC << "Failed to read \"" << key << "\" from cache bin " << getID() << std::endl;
        }

        output = r.getObject();
        return output.valid();
    }

    bool
    FileSystemCacheBin::readString(std::string&        output,
                                   const std::string&  key,
                                   double              maxAge )
    {
        output.clear();

        if ( !_ok ) return 0L;

        //todo: handle maxAge

        //todo: mangle "key" into a legal path name
        URI fileURI( toLegalFileName(key), _metaPath );

        {
            ScopedReadLock sharedLock( _rwmutex );
            std::ifstream infile( (fileURI.full() + ".dat").c_str() );
            if ( infile.is_open() )
            {
                infile >> std::noskipws;
                std::stringstream buf;
                buf << infile.rdbuf();
			    std::string bufStr;
		        bufStr = buf.str();
                output = bufStr;
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

        return output.size() > 0;
    }

    bool
    FileSystemCacheBin::write( const std::string& key, const osg::Object* object )
    {
        if ( !_ok ) return false;

        // convert the key into a legal filename:
        URI fileURI( toLegalFileName(key), _metaPath );

        osgDB::ReaderWriter::WriteResult r;
        {
            // prevent cache contention:
            ScopedWriteLock exclusiveLock( _rwmutex );

            // make a home for it..
            if ( !osgDB::fileExists( osgDB::getFilePath(fileURI.full()) ) )
                osgDB::makeDirectoryForFile( fileURI.full() );

            // tack on the extension
            std::string filename = fileURI.full() + ".osgb";

            // write it.        
            if ( dynamic_cast<const osg::Image*>(object) )
                r = _rw->writeImage( *static_cast<const osg::Image*>(object), filename, _rwOptions.get() );
            else if ( dynamic_cast<const osg::Node*>(object) )
                r = _rw->writeNode( *static_cast<const osg::Node*>(object), filename, _rwOptions.get() );
            else
                r = _rw->writeObject( *object, filename );
        }

        if ( r.success() )
        {
            OE_DEBUG << LC << "Wrote \"" << key << "\" to cache bin " << getID() << std::endl;
        }
        else
        {
            OE_WARN << LC << "FAILED to write \"" << key << "\" to cache bin "
                << getID() << "; message: " << r.message() << std::endl;
        }

        return r.success();
    }

    bool
    FileSystemCacheBin::write(const std::string& key, const std::string& buffer )
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

    Config
    FileSystemCacheBin::readMetadata()
    {
        if ( !_ok ) return Config();

        ScopedReadLock sharedLock( _rwmutex );
        osg::ref_ptr<XmlDocument> xml = XmlDocument::load( _metaPath );
        if ( xml.valid() )
        {
            return xml->getConfig();
        }        
        return Config();
    }

    bool
    FileSystemCacheBin::writeMetadata( const Config& conf )
    {
        if ( !_ok ) return false;

        ScopedWriteLock exclusiveLock( _rwmutex );
        XmlDocument xml( conf );
        std::fstream output( _metaPath.c_str(), std::ios_base::out );
        if ( output.is_open() )
            xml.store( output );
        return output.is_open();
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
