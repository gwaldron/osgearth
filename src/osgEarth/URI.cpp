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
#include <osgEarth/URI>
#include <osgEarth/Cache>
#include <osgEarth/CacheBin>
#include <osgEarth/HTTPClient>
#include <osgEarth/Registry>
#include <osgEarth/Progress>
#include <osgEarth/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>
#include <osgDB/ReaderWriter>
#include <osgDB/Archive>
#include <fstream>
#include <sstream>

#define LC "[URI] "

#define OE_TEST OE_NULL
//#define OE_TEST OE_NOTICE

using namespace osgEarth;

//------------------------------------------------------------------------

namespace
{
    /**
     * "Fixes" the osgDB options by disabling the automatic archive caching. Archive caching
     * screws up our URI resolution because with it on, osgDB remembers every archive file
     * you open and effectively puts it in all future search paths :(
     */
    const osgDB::Options*
    fixOptions( const osgDB::Options* input )
    {
        if ( input && input->getObjectCacheHint() == osgDB::Options::CACHE_NONE )
        {
            return input;
        }
        else
        {
            return Registry::instance()->cloneOrCreateOptions( input );
        }
    }
}

//------------------------------------------------------------------------

URIStream::URIStream( const URI& uri ) :
_fileStream( 0L )
{
    if ( osgDB::containsServerAddress(uri.full()) )
    {
        HTTPResponse res = HTTPClient::get( uri.full() );
        if ( res.isOK() )
        {
            std::string buf = res.getPartAsString(0);
            _bufStream.str(buf);
        }
    }
    else
    {
        _fileStream = new std::ifstream( uri.full().c_str() );
    }
}

URIStream::~URIStream()
{
    if ( _fileStream )
        delete _fileStream;
}

URIStream::operator std::istream& ()
{
    static std::istringstream s_nullStream;

    if ( _fileStream )
        return *_fileStream;
    else
        return _bufStream;
}

//------------------------------------------------------------------------

std::string
URIContext::getOSGPath( const std::string& target ) const
{
    return osgEarth::getFullPath( _referrer, target );
}

URIContext
URIContext::add( const URIContext& sub ) const
{
    return URIContext( osgEarth::getFullPath(_referrer, sub._referrer) );
}

URIContext
URIContext::add( const std::string& sub ) const
{
    return URIContext(osgDB::concatPaths( _referrer, sub ));
}

void
URIContext::apply( osgDB::Options* options )
{
    if ( options )
    {
        options->setDatabasePath( _referrer );
        options->setPluginStringData( "osgEarth::URIContext::referrer", _referrer );
    }
}

URIContext::URIContext( const osgDB::Options* options )
{
    if ( options )
    {
        _referrer = options->getPluginStringData( "osgEarth::URIContext::referrer" );

        if ( _referrer.empty() && options->getDatabasePathList().size() > 0 )
        {
            const std::string& front = options->getDatabasePathList().front();
            if ( osgEarth::isArchive(front) )
            {
                _referrer = front + "/";
            }
        }
    }
}

//------------------------------------------------------------------------

URI::URI()
{
    //nop
}

URI::URI( const std::string& location )
{
    _baseURI = location;
    _fullURI = location;
}

URI::URI( const std::string& location, const URIContext& context )
{
    _context = context;
    _baseURI = location;
    _fullURI = context.getOSGPath( _baseURI );
}

URI::URI( const char* location )
{
    _baseURI = std::string(location);
    _fullURI = _baseURI;
}

URI
URI::append( const std::string& suffix ) const
{
    URI result;
    result._baseURI = _baseURI + suffix;
    result._fullURI = _fullURI + suffix;
    result._context = _context;
    return result;
}

bool
URI::isRemote() const
{
    return osgDB::containsServerAddress( _fullURI );
}

namespace
{
    // extracts a CacheBin from the dboptions; if one cannot be found, fall back on the
    // default CacheBin of a Cache found in the dboptions; failing that, call back on
    // the default CacheBin of the registry-wide cache.
    CacheBin* s_getCacheBin( const osgDB::Options* dbOptions )
    {
        const osgDB::Options* o = dbOptions ? dbOptions : Registry::instance()->getDefaultOptions();

        CacheBin* bin = CacheBin::get( o );
        if ( !bin )
        {
            Cache* cache = Cache::get( o );
            if ( !cache )
            {
                cache = Registry::instance()->getCache();
            }

            if ( cache )
            {
                bin = cache->getOrCreateDefaultBin();
            }
        }
        return bin;
    }

    // convert an osgDB::ReaderWriter::ReadResult to an osgEarth::ReadResult
    ReadResult toReadResult( osgDB::ReaderWriter::ReadResult& rr )
    {
        if ( rr.validObject() )
            return ReadResult( rr.getObject() );
        else
            return ReadResult( ReadResult::RESULT_NOT_FOUND ); // TODO: translate codes better
    }

    // Utility to redirect a local file read if it has an archive name in the URI
    ReadResult readStringFile( const std::string& uri, const osgDB::Options* opt )
    {
        osgDB::Registry::ArchiveExtensionList e = osgDB::Registry::instance()->getArchiveExtensions();
        for(osgDB::Registry::ArchiveExtensionList::iterator aitr = e.begin(); aitr != e.end(); ++aitr )
        {
            std::string archiveExtension = "." + (*aitr);
            std::string::size_type positionArchive = uri.find(archiveExtension+'/');
            if (positionArchive == std::string::npos) positionArchive = uri.find(archiveExtension+'\\');
            if (positionArchive != std::string::npos)
            {
                std::string::size_type endArchive = positionArchive + archiveExtension.length();
                std::string archiveName( uri.substr(0, endArchive));
                std::string fileName( uri.substr(endArchive+1, std::string::npos) );

                osgDB::ReaderWriter::ReadResult result = osgDB::Registry::instance()->openArchiveImplementation(
                    archiveName, osgDB::ReaderWriter::READ, 4096, opt );

                if (!result.validArchive()) 
                    return ReadResult(); // error

                osgDB::Archive* archive = result.getArchive();

                osg::ref_ptr<osgDB::ReaderWriter::Options> options = opt ? opt->cloneOptions() : 
                    Registry::instance()->cloneOrCreateOptions();

                options->setDatabasePath(archiveName);

                osgDB::ReaderWriter::ReadResult rr = archive->readObject(fileName, options.get());
                if ( rr.success() )
                {
                    ReadResult result( rr.takeObject() );
                    result.setLastModifiedTime( osgEarth::getLastModifiedTime(uri) );
                    return result;
                }
                else
                {
                    return ReadResult();
                }
            }
        }

        // no archive; just read it normally
        std::ifstream input( uri.c_str() );
        if ( input.is_open() )
        {
            input >> std::noskipws;
            std::stringstream buf;
            buf << input.rdbuf();
            std::string bufStr;
            bufStr = buf.str();
            ReadResult result( new StringObject(bufStr) );
            result.setLastModifiedTime( osgEarth::getLastModifiedTime(uri) );
            return result;
        }

        // no good
        return ReadResult();
    }


    //--------------------------------------------------------------------
    // Read functors (used by the doRead method)

    struct ReadObject
    {
        bool callbackRequestsCaching( URIReadCallback* cb ) const { return !cb || ((cb->cachingSupport() & URIReadCallback::CACHE_OBJECTS) != 0); }
        ReadResult fromCallback( URIReadCallback* cb, const std::string& uri, const osgDB::Options* opt ) { return cb->readObject(uri, opt); }
        ReadResult fromCache( CacheBin* bin, const std::string& key, TimeStamp minTime) { return bin->readObject(key, minTime); }
        ReadResult fromHTTP( const std::string& uri, const osgDB::Options* opt, ProgressCallback* p ) { return HTTPClient::readObject(uri, opt, p); }
        ReadResult fromFile( const std::string& uri, const osgDB::Options* opt ) { return ReadResult(osgDB::readObjectFile(uri, opt)); }
    };

    struct ReadNode
    {
        bool callbackRequestsCaching( URIReadCallback* cb ) const { return !cb || ((cb->cachingSupport() & URIReadCallback::CACHE_NODES) != 0); }
        ReadResult fromCallback( URIReadCallback* cb, const std::string& uri, const osgDB::Options* opt ) { return cb->readNode(uri, opt); }
        ReadResult fromCache( CacheBin* bin, const std::string& key, TimeStamp minTime) { return bin->readObject(key, minTime); }
        ReadResult fromHTTP( const std::string& uri, const osgDB::Options* opt, ProgressCallback* p ) { return HTTPClient::readNode(uri, opt, p); }
        ReadResult fromFile( const std::string& uri, const osgDB::Options* opt ) { return ReadResult(osgDB::readNodeFile(uri, opt)); }
    };

    struct ReadImage
    {
        bool callbackRequestsCaching( URIReadCallback* cb ) const { 
            return !cb || ((cb->cachingSupport() & URIReadCallback::CACHE_IMAGES) != 0); 
        }
        ReadResult fromCallback( URIReadCallback* cb, const std::string& uri, const osgDB::Options* opt ) { 
            ReadResult r = cb->readImage(uri, opt);
            if ( r.getImage() ) r.getImage()->setFileName(uri);
            return r;
        }                
        ReadResult fromCache( CacheBin* bin, const std::string& key, TimeStamp minTime ) { 
            ReadResult r = bin->readImage(key, minTime);
            if ( r.getImage() ) r.getImage()->setFileName( key );
            return r;
        }
        ReadResult fromHTTP( const std::string& uri, const osgDB::Options* opt, ProgressCallback* p ) { 
            ReadResult r = HTTPClient::readImage(uri, opt, p);
            if ( r.getImage() ) r.getImage()->setFileName( uri );
            return r;
        }
        ReadResult fromFile( const std::string& uri, const osgDB::Options* opt ) { 
            ReadResult r = ReadResult(osgDB::readImageFile(uri, opt));
            if ( r.getImage() ) r.getImage()->setFileName( uri );
            return r;
        }
    };

    struct ReadString
    {
        bool callbackRequestsCaching( URIReadCallback* cb ) const { return !cb || ((cb->cachingSupport() & URIReadCallback::CACHE_STRINGS) != 0); }
        ReadResult fromCallback( URIReadCallback* cb, const std::string& uri, const osgDB::Options* opt ) { return cb->readString(uri, opt); }
        ReadResult fromCache( CacheBin* bin, const std::string& key, TimeStamp minTime) { return bin->readString(key, minTime); }
        ReadResult fromHTTP( const std::string& uri, const osgDB::Options* opt, ProgressCallback* p ) { return HTTPClient::readString(uri, opt, p); }
        ReadResult fromFile( const std::string& uri, const osgDB::Options* opt ) { return readStringFile(uri, opt); }
    };

    //--------------------------------------------------------------------
    // MASTER read template function. I templatized this so we wouldn't
    // have 4 95%-identical code paths to maintain...

    template<typename READ_FUNCTOR>
    ReadResult doRead(
        const URI&            inputURI,
        const osgDB::Options* dbOptions,
        ProgressCallback*     progress)
    {        
        //osg::Timer_t startTime = osg::Timer::instance()->tick();

        ReadResult result;

        if ( !inputURI.empty() )
        {
            // establish our IO options:
            const osgDB::Options* localOptions = dbOptions ? dbOptions : Registry::instance()->getDefaultOptions();

            READ_FUNCTOR reader;

            URI uri = inputURI;

            bool gotResultFromCallback = false;

            // check if there's an alias map, and if so, attempt to resolve the alias:
            URIAliasMap* aliasMap = URIAliasMap::from( localOptions );
            if ( aliasMap )
            {
                uri = aliasMap->resolve(inputURI.full(), inputURI.context());
            }

            // check if there's a URI cache in the options.
            URIResultCache* memCache = URIResultCache::from( localOptions );
            if ( memCache )
            {
                URIResultCache::Record rec;
                if ( memCache->get(uri, rec) )
                {
                    result = rec.value();
                }
            }

            if ( result.empty() )
            {
                // see if there's a read callback installed.
                URIReadCallback* cb = Registry::instance()->getURIReadCallback();

                // for a local URI, bypass all the caching logic
                if ( !uri.isRemote() )
                {
                    // try to use the callback if it's set. Callback ignores the caching policy.
                    if ( cb )
                    {
                        // if this returns "not implemented" we fill fall back
                        result = reader.fromCallback( cb, uri.full(), localOptions );

                        if ( result.code() != ReadResult::RESULT_NOT_IMPLEMENTED )
                        {
                            // "not implemented" is the only excuse to fall back.
                            gotResultFromCallback = true;
                        }
                    }

                    if ( !gotResultFromCallback )
                    {
                        // no callback, just read from a local file.
                        result = reader.fromFile( uri.full(), localOptions );
                    }
                }

                // remote URI, consider caching:
                else
                {
                    bool callbackCachingOK = !cb || reader.callbackRequestsCaching(cb);

                    // establish the caching policy.
                    optional<CachePolicy> cp;
                    if ( !Registry::instance()->getCachePolicy( cp, localOptions ) )
                        cp = CachePolicy::DEFAULT;

                    // get a cache bin if we need it:
                    CacheBin* bin = 0L;
                    if ( (cp->usage() != CachePolicy::USAGE_NO_CACHE) && callbackCachingOK )
                    {
                        bin = s_getCacheBin( dbOptions );
                    }

                    // first try to go to the cache if there is one:
                    if ( bin && cp->isCacheReadable() )
                    {
                        result = reader.fromCache( bin, uri.cacheKey(), cp->getMinAcceptTime() );
                        if ( result.succeeded() )
                            result.setIsFromCache(true);
                    }

                    // not in the cache, so proceed to read it from the network.
                    if ( result.empty() )
                    {
                        // Need to do this to support nested PLODs and Proxynodes.
                        osg::ref_ptr<osgDB::Options> remoteOptions =
                            Registry::instance()->cloneOrCreateOptions( localOptions );
                        remoteOptions->getDatabasePathList().push_front( osgDB::getFilePath(uri.full()) );

                        // try to use the callback if it's set. Callback ignores the caching policy.
                        if ( cb )
                        {                
                            result = reader.fromCallback( cb, uri.full(), remoteOptions.get() );

                            if ( result.code() != ReadResult::RESULT_NOT_IMPLEMENTED )
                            {
                                // "not implemented" is the only excuse for falling back
                                gotResultFromCallback = true;
                            }
                        }

                        if ( !gotResultFromCallback )
                        {
                            // still no data, go to the source:
                            if ( result.empty() && cp->usage() != CachePolicy::USAGE_CACHE_ONLY )
                            {
                                result = reader.fromHTTP( uri.full(), remoteOptions.get(), progress );
                            }

                            // write the result to the cache if possible:
                            if ( result.succeeded() && bin && cp->isCacheWriteable() )
                            {
                                bin->write( uri.cacheKey(), result.getObject(), result.metadata() );
                            }
                        }
                    }

                    OE_TEST << LC 
                        << uri.base() << ": " 
                        << (result.succeeded() ? "OK" : "FAILED") 
                        << "; policy=" << cp->usageString()
                        << (result.isFromCache() && result.succeeded() ? "; (from cache)" : "")
                        << std::endl;
                }


                if ( result.getObject() && !gotResultFromCallback )
                {
                    result.getObject()->setName( uri.base() );

                    if ( memCache )
                    {
                        memCache->insert( uri, result );
                    }
                }
            }
        }

        // post-process if there's a post-URI callback.
        URIPostReadCallback* post = URIPostReadCallback::from(dbOptions);
        if ( post )
        {
            (*post)(result);
        }

        /*
        osg::Timer_t endTime = osg::Timer::instance()->tick();

        double time = osg::Timer::instance()->delta_s( startTime, endTime );
        {
            OpenThreads::ScopedLock< OpenThreads::Mutex > lock( s_statsLock );            
            totalTime += time;
            totalRequests += 1;
            double avg = (double)totalRequests / totalTime;
            OE_NOTICE << "total req = " << totalRequests << " totalTime = " << totalTime << " " << avg << " req/s" << std::endl;            
        }
        */

        

        return result;
    }
}

ReadResult
URI::readObject(const osgDB::Options* dbOptions,
                ProgressCallback*     progress ) const
{
    return doRead<ReadObject>( *this, dbOptions, progress );
}

ReadResult
URI::readNode(const osgDB::Options* dbOptions,
              ProgressCallback*     progress ) const
{
    return doRead<ReadNode>( *this, dbOptions, progress );
}

ReadResult
URI::readImage(const osgDB::Options* dbOptions,
               ProgressCallback*     progress ) const
{
    return doRead<ReadImage>( *this, dbOptions, progress );
}

ReadResult
URI::readString(const osgDB::Options* dbOptions,
                ProgressCallback*     progress ) const
{
    return doRead<ReadString>( *this, dbOptions, progress );
}


//------------------------------------------------------------------------

void
URIAliasMap::insert(const std::string& key, const std::string& value)
{
    _map.insert( std::make_pair(key, value) );
}

std::string
URIAliasMap::resolve(const std::string& input, const URIContext& context) const
{
    for(std::map<std::string,std::string>::const_iterator i = _map.begin(); i != _map.end(); ++i )
    {
        std::string source = context.getOSGPath(i->first);
        std::string pattern = context.getOSGPath(input);
        if ( ciEquals(source, pattern) )
            return context.getOSGPath(i->second);
    }
    return input;
}


//------------------------------------------------------------------------


URIAliasMapReadCallback::URIAliasMapReadCallback(const URIAliasMap& aliasMap,
                                                 const URIContext&  context ) : 
_aliasMap( aliasMap ), 
_context ( context )
{
    //nop
}

osgDB::ReaderWriter::ReadResult
URIAliasMapReadCallback::openArchive(const std::string& filename, osgDB::ReaderWriter::ArchiveStatus status, unsigned int indexBlockSizeHint, const osgDB::Options* useObjectCache)
{
    if (osgDB::Registry::instance()->getReadFileCallback()) return osgDB::Registry::instance()->getReadFileCallback()->openArchive(_aliasMap.resolve(filename,_context), status, indexBlockSizeHint, useObjectCache);
    else return osgDB::Registry::instance()->openArchive(_aliasMap.resolve(filename,_context), status, indexBlockSizeHint, useObjectCache);
}

osgDB::ReaderWriter::ReadResult 
URIAliasMapReadCallback::readObject(const std::string& filename, const osgDB::Options* options)
{
    if (osgDB::Registry::instance()->getReadFileCallback()) return osgDB::Registry::instance()->getReadFileCallback()->readObject(_aliasMap.resolve(filename,_context),options);
    else return osgDB::Registry::instance()->readObjectImplementation(_aliasMap.resolve(filename,_context),options);
}

osgDB::ReaderWriter::ReadResult 
URIAliasMapReadCallback::readImage(const std::string& filename, const osgDB::Options* options)
{
    OE_INFO << LC << "Map: " << filename << " to " << _aliasMap.resolve(filename,_context) << std::endl;
    if (osgDB::Registry::instance()->getReadFileCallback()) return osgDB::Registry::instance()->getReadFileCallback()->readImage(_aliasMap.resolve(filename,_context),options);
    else return osgDB::Registry::instance()->readImageImplementation(_aliasMap.resolve(filename,_context),options);
}

osgDB::ReaderWriter::ReadResult 
URIAliasMapReadCallback::readHeightField(const std::string& filename, const osgDB::Options* options)
{
    if (osgDB::Registry::instance()->getReadFileCallback()) return osgDB::Registry::instance()->getReadFileCallback()->readHeightField(_aliasMap.resolve(filename,_context),options);
    else return osgDB::Registry::instance()->readHeightFieldImplementation(_aliasMap.resolve(filename,_context),options);
}

osgDB::ReaderWriter::ReadResult 
URIAliasMapReadCallback::readNode(const std::string& filename, const osgDB::Options* options)
{
    if (osgDB::Registry::instance()->getReadFileCallback()) return osgDB::Registry::instance()->getReadFileCallback()->readNode(_aliasMap.resolve(filename,_context),options);
    else return osgDB::Registry::instance()->readNodeImplementation(_aliasMap.resolve(filename,_context),options);
}

osgDB::ReaderWriter::ReadResult 
URIAliasMapReadCallback::readShader(const std::string& filename, const osgDB::Options* options)
{
    if (osgDB::Registry::instance()->getReadFileCallback()) return osgDB::Registry::instance()->getReadFileCallback()->readShader(_aliasMap.resolve(filename,_context),options);
    else return osgDB::Registry::instance()->readShaderImplementation(_aliasMap.resolve(filename,_context),options);
}
