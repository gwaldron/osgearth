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
#include <osgEarth/URI>
#include <osgEarth/Cache>
#include <osgEarth/CacheBin>
#include <osgEarth/HTTPClient>
#include <osgEarth/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>
#include <osgDB/ReaderWriter>
#include <fstream>
#include <sstream>

#define LC "[URI] "

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
URIContext::store( osgDB::Options* options )
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

    //--------------------------------------------------------------------
    // Read functors (used by the doRead method)

    struct ReadObject
    {
        bool callbackRequestsCaching( URIReadCallback* cb ) const { return !cb || ((cb->cachingSupport() & URIReadCallback::CACHE_OBJECTS) != 0); }
        osgDB::ReaderWriter::ReadResult fromCallback( URIReadCallback* cb, const std::string& uri, const osgDB::Options* opt ) { return cb->readObject(uri, opt); }
        ReadResult fromCache( CacheBin* bin, const std::string& uri, double maxAge ) { return bin->readObject(uri, maxAge); }
        ReadResult fromHTTP( const std::string& uri, const osgDB::Options* opt, ProgressCallback* p ) { return HTTPClient::readObject(uri, opt, p); }
        ReadResult fromFile( const std::string& uri, const osgDB::Options* opt ) { return ReadResult(osgDB::readObjectFile(uri, opt)); }
    };

    struct ReadNode
    {
        bool callbackRequestsCaching( URIReadCallback* cb ) const { return !cb || ((cb->cachingSupport() & URIReadCallback::CACHE_NODES) != 0); }
        osgDB::ReaderWriter::ReadResult fromCallback( URIReadCallback* cb, const std::string& uri, const osgDB::Options* opt ) { return cb->readNode(uri, opt); }
        ReadResult fromCache( CacheBin* bin, const std::string& uri, double maxAge ) { return bin->readObject(uri, maxAge); }
        ReadResult fromHTTP( const std::string& uri, const osgDB::Options* opt, ProgressCallback* p ) { return HTTPClient::readNode(uri, opt, p); }
        ReadResult fromFile( const std::string& uri, const osgDB::Options* opt ) { return ReadResult(osgDB::readNodeFile(uri, opt)); }
    };

    struct ReadImage
    {
        bool callbackRequestsCaching( URIReadCallback* cb ) const { return !cb || ((cb->cachingSupport() & URIReadCallback::CACHE_IMAGES) != 0); }
        osgDB::ReaderWriter::ReadResult fromCallback( URIReadCallback* cb, const std::string& uri, const osgDB::Options* opt ) { return cb->readImage(uri, opt); }
        ReadResult fromCache( CacheBin* bin, const std::string& uri, double maxAge ) { return bin->readImage(uri, maxAge); }
        ReadResult fromHTTP( const std::string& uri, const osgDB::Options* opt, ProgressCallback* p ) { return HTTPClient::readImage(uri, opt, p); }
        ReadResult fromFile( const std::string& uri, const osgDB::Options* opt ) { return ReadResult(osgDB::readImageFile(uri, opt)); }
    };

    struct ReadString
    {
        bool callbackRequestsCaching( URIReadCallback* cb ) const { return !cb || ((cb->cachingSupport() & URIReadCallback::CACHE_STRINGS) != 0); }
        osgDB::ReaderWriter::ReadResult fromCallback( URIReadCallback* cb, const std::string& uri, const osgDB::Options* opt ) { return cb->readString(uri, opt); }
        ReadResult fromCache( CacheBin* bin, const std::string& uri, double maxAge ) { return bin->readString(uri, maxAge); }
        ReadResult fromHTTP( const std::string& uri, const osgDB::Options* opt, ProgressCallback* p ) { return HTTPClient::readString(uri, opt, p); }
        ReadResult fromFile( const std::string& uri, const osgDB::Options* opt ) 
        {
            std::ifstream input( uri.c_str() );
            if ( input.is_open() )
            {
                input >> std::noskipws;
                std::stringstream buf;
                buf << input.rdbuf();
			    std::string bufStr;
		        bufStr = buf.str();
                return ReadResult( new StringObject(bufStr) );
            }
            else return ReadResult();
        }
    };

    //--------------------------------------------------------------------
    // MASTER read template function. I templatized this so we wouldn't
    // have 4 95%-identical code paths to maintain...

    template<typename READ_FUNCTOR>
    ReadResult doRead(
        const URI&            uri,
        const osgDB::Options* dbOptions,
        const CachePolicy&    cachePolicy,
        ProgressCallback*     progress)
    {
        ReadResult result;

        if ( uri.empty() )
            return result;

        READ_FUNCTOR reader;

        // see if there's a read callback installed.
        URIReadCallback* cb = Registry::instance()->getURIReadCallback();

        // for a local URI, bypass all the caching logic
        if ( !uri.isRemote() )
        {
            // try to use the callback if it's set. Callback ignores the caching policy.
            if ( cb )
            {                
                osgDB::ReaderWriter::ReadResult rr = reader.fromCallback( cb, uri.full(), dbOptions );
                if ( rr.validObject() )
                {
                    result = ReadResult( rr.getObject() );
                }
                else if ( rr.status() != osgDB::ReaderWriter::ReadResult::NOT_IMPLEMENTED )
                {
                    // only "NOT_IMPLEMENTED" is a reason to fallback. Anything else if a FAIL
                    return ReadResult( ReadResult::RESULT_NOT_FOUND );
                }
            }

            if ( result.empty() )
            {
                result = reader.fromFile( uri.full(), dbOptions );
            }
        }

        // remote URI, consider caching:
        else
        {
            bool callbackCachingOK = !cb || reader.callbackRequestsCaching(cb);

            // establish our caching policy:
            const CachePolicy& cp = !cachePolicy.empty() ? cachePolicy : Registry::instance()->defaultCachePolicy();

            // get a cache bin if we need it:
            CacheBin* bin = 0L;
            if ( (cp.usage() != CachePolicy::USAGE_NO_CACHE) && callbackCachingOK )
            {
                bin = s_getCacheBin( dbOptions );
            }

            // first try to go to the cache if there is one:
            if ( bin && cp.isCacheReadable() )
            {
                result = reader.fromCache( bin, uri.full(), *cp.maxAge() );
            }

            // not in the cache, so proceed to read it from the network.
            if ( result.empty() )
            {
                // try to use the callback if it's set. Callback ignores the caching policy.
                if ( cb )
                {                
                    osgDB::ReaderWriter::ReadResult rr = reader.fromCallback( cb, uri.full(), dbOptions );
                    if ( rr.validObject() )
                    {
                        result = ReadResult( rr.getObject() );
                    }
                    else if ( rr.status() != osgDB::ReaderWriter::ReadResult::NOT_IMPLEMENTED )
                    {
                        // only "NOT_IMPLEMENTED" is a reason to fallback. Anything else if a FAIL
                        return ReadResult( ReadResult::RESULT_NOT_FOUND );
                    }
                }

                // still no data, go to the source:
                if ( result.empty() && cp.usage() != CachePolicy::USAGE_CACHE_ONLY )
                {
                    result = reader.fromHTTP( 
                        uri.full(),
                        dbOptions ? dbOptions : Registry::instance()->getDefaultOptions(),
                        progress );
                }

                // write the result to the cache if possible:
                if ( result.succeeded() && bin && cp.isCacheWriteable() )
                {
                    bin->write( uri.full(), result.getObject(), result.metadata() );
                }
            }
        }

        return result;
    }
}

ReadResult
URI::readObject(const osgDB::Options* dbOptions,
                const CachePolicy&    cachePolicy,
                ProgressCallback*     progress ) const
{
    return doRead<ReadObject>( *this, dbOptions, cachePolicy, progress );
}

ReadResult
URI::readNode(const osgDB::Options* dbOptions,
              const CachePolicy&    cachePolicy,
              ProgressCallback*     progress ) const
{
    return doRead<ReadNode>( *this, dbOptions, cachePolicy, progress );
}

ReadResult
URI::readImage(const osgDB::Options* dbOptions,
               const CachePolicy&    cachePolicy,
               ProgressCallback*     progress ) const
{
    return doRead<ReadImage>( *this, dbOptions, cachePolicy, progress );
}

ReadResult
URI::readString(const osgDB::Options* dbOptions,
                const CachePolicy&    cachePolicy,
                ProgressCallback*     progress ) const
{
    return doRead<ReadString>( *this, dbOptions, cachePolicy, progress );
}



#if 0
ReadResult
URI::readObject(const osgDB::Options* dbOptions,
                const CachePolicy&    cachePolicy,
                ProgressCallback*     progress ) const
{
    ReadResult result;

    if ( empty() )
        return result;

    // see if there's a read callback installed.
    URIReadCallback* cb = Registry::instance()->getURIReadCallback();
    bool cbSupportsCache = !cb || ((cb->cachingSupport() & URIReadCallback::CACHE_OBJECTS) != 0);

    // establish our caching policy:
    const CachePolicy& cp = !cachePolicy.empty() ? cachePolicy : Registry::instance()->defaultCachePolicy();

    // get a cache bin if we need it:
    CacheBin* bin = 0L;
    if (isRemote() && (cp.usage() != CachePolicy::USAGE_NO_CACHE) && cbSupportsCache )
    {
        bin = s_getCacheBin( dbOptions );
    }

    // first try to go to the cache if there is one:
    if ( bin && cp.isCacheReadable() )
    {
        result = bin->readObject( full(), *cp.maxAge() );
    }

    // not in the cache, so move on:
    if ( result.empty() )
    {
        // try to use the callback if it's set. Callback ignores the caching policy.
        if ( cb )
        {
            osgDB::ReaderWriter::ReadResult rr = cb->readObject( full(), dbOptions );
            if ( rr.validObject() )
            {
                result = ReadResult( rr.getObject() );
            }
            else if ( rr.status() != osgDB::ReaderWriter::ReadResult::NOT_IMPLEMENTED )
            {
                // only "NOT_IMPLEMENTED" is a reason to fallback. Anything else if a FAIL
                return ReadResult( ReadResult::RESULT_NOT_FOUND );
            }
        }

        // still no data, go to the source:
        if ( result.empty() )
        {
            if ( isRemote() )
            {
                result = HTTPClient::readObject(
                    full(),
                    dbOptions ? dbOptions : Registry::instance()->getDefaultOptions(),
                    progress );
            }
            else
            {
                result = ReadResult( osgDB::readObjectFile(full(), dbOptions) );
            }
        }

        // write the result to the cache if possible:
        if ( result.succeeded() && bin && cp.isCacheWriteable() )
        {
            bin->write( full(), result.getObject(), result.metadata() );
        }
    }

    return result;
}



ReadResult
URI::readImage(const osgDB::Options* dbOptions,
               const CachePolicy&    cachePolicy,
               ProgressCallback*     progress ) const
{
    ReadResult result;

    if ( empty() )
        return result;

    // see if there's a read callback installed.
    URIReadCallback* cb = Registry::instance()->getURIReadCallback();
    bool cbSupportsCache = !cb || ((cb->cachingSupport() & URIReadCallback::CACHE_OBJECTS) != 0);

    // establish our caching policy:
    const CachePolicy& cp = !cachePolicy.empty() ? cachePolicy : Registry::instance()->defaultCachePolicy();

    // get a cache bin if we need it:
    CacheBin* bin = 0L;
    if (isRemote() && (cp.usage() != CachePolicy::USAGE_NO_CACHE) && cbSupportsCache )
    {
        bin = s_getCacheBin( dbOptions );
    }

    // first, try to read from the cache.
    if ( bin && cp.isCacheReadable() )
    {
        result = bin->readImage( full(), *cp.maxAge() );
    }

    // second, try to read from the source.
    if ( result.empty() && cp.usage() != CachePolicy::USAGE_CACHE_ONLY )
    {
        if ( isRemote() )
        {
            result = HTTPClient::readImage(
                _fullURI,
                dbOptions ? dbOptions : Registry::instance()->getDefaultOptions(),
                progress );
        }
        else
        {
            result = ReadResult( osgDB::readImageFile( full(), dbOptions ) );
        }

        if ( result.succeeded() && bin && cp.isCacheWriteable() )
        {
            bin->write( full(), result.get<osg::Image>(), result.metadata() );
        }
    }

    return result;
}

ReadResult
URI::readNode(const osgDB::Options* dbOptions,
              const CachePolicy&    cachePolicy,
              ProgressCallback*     progress ) const
{
    ReadResult result;

    if ( empty() )
        return result;

    // see if there's a read callback installed.
    URIReadCallback* cb = Registry::instance()->getURIReadCallback();
    bool cbSupportsCache = !cb || ((cb->cachingSupport() & URIReadCallback::CACHE_OBJECTS) != 0);

    // establish our caching policy:
    const CachePolicy& cp = !cachePolicy.empty() ? cachePolicy : Registry::instance()->defaultCachePolicy();

    // get a cache bin if we need it:
    CacheBin* bin = 0L;
    if (isRemote() && (cp.usage() != CachePolicy::USAGE_NO_CACHE) && cbSupportsCache )
    {
        bin = s_getCacheBin( dbOptions );
    }

    if ( bin && cp.isCacheReadable() )
    {
        result = bin->readObject( full(), *cp.maxAge() );
    }

    if ( result.empty() && cp.usage() != CachePolicy::USAGE_CACHE_ONLY )
    {
        if ( isRemote() )
        {
            result = HTTPClient::readNode(
                _fullURI,
                dbOptions ? dbOptions : Registry::instance()->getDefaultOptions(),
                progress );
        }
        else
        {
            result = ReadResult( osgDB::readNodeFile(full(), dbOptions) );
        }

        if ( result.succeeded() && bin && cp.isCacheWriteable() )
        {
            bin->write( full(), result.getObject(), result.metadata() );
        }
    }

    return result;
}

ReadResult
URI::readString(const osgDB::Options* dbOptions,
                const CachePolicy&    cachePolicy,
                ProgressCallback*     progress ) const
{
    ReadResult result;

    CacheBin* bin = 0L;

    const CachePolicy& cp = !cachePolicy.empty() ? cachePolicy : Registry::instance()->defaultCachePolicy();

    if ( isRemote() && cp.usage() != CachePolicy::USAGE_NO_CACHE )
    {
        bin = s_getCacheBin( dbOptions );
    }

    if ( bin && cp.isCacheReadable() )
    {
        result = bin->readString( full(), *cp.maxAge() );
    }

    if ( result.empty() && cp.usage() != CachePolicy::USAGE_CACHE_ONLY )
    {
        if ( isRemote() )
        {
            result = HTTPClient::readString(
                _fullURI,
                dbOptions ? dbOptions : Registry::instance()->getDefaultOptions(),
                progress );
        }
        else
        {            
            std::ifstream input( full().c_str() );
            if ( input.is_open() )
            {
                input >> std::noskipws;
                std::stringstream buf;
                buf << input.rdbuf();
			    std::string bufStr;
		        bufStr = buf.str();
                result = ReadResult( new StringObject(bufStr) );
            }
        }

        if ( result.succeeded() && bin && cp.isCacheWriteable() )
        {
            bin->write( full(), result.getObject(), result.metadata() );
        }
    }

    return result;
}
#endif
