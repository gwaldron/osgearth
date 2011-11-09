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
        if ( rr.getObject() )
            return ReadResult( rr.getObject() );
        else
            return ReadResult( ReadResult::RESULT_NOT_FOUND ); // TODO: translate codes better
    }
}

ReadResult
URI::readObject(const osgDB::Options* dbOptions,
                const CachePolicy&    cachePolicy,
                ProgressCallback*     progress ) const
{
    ReadResult result;

    if ( empty() )
        return result;

    CacheBin* bin = 0L;

    const CachePolicy& cp = !cachePolicy.empty() ? cachePolicy : Registry::instance()->defaultCachePolicy();

    if ( cp.usage() != CachePolicy::USAGE_NO_CACHE )
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
            result = HTTPClient::readObject(
                _fullURI,
                dbOptions ? dbOptions : Registry::instance()->getDefaultOptions(),
                progress );
        }
        else
        {
            result = ReadResult( osgDB::readObjectFile(full(), dbOptions) );
        }

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

    CacheBin* bin = 0L;

    const CachePolicy& cp = !cachePolicy.empty() ? cachePolicy : Registry::instance()->defaultCachePolicy();

    if ( cp.usage() != CachePolicy::USAGE_NO_CACHE )
    {
        bin = s_getCacheBin( dbOptions );
    }

    if ( bin && cp.isCacheReadable() )
    {
        result = bin->readImage( full(), *cp.maxAge() );
    }

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

    CacheBin* bin = 0L;

    const CachePolicy& cp = !cachePolicy.empty() ? cachePolicy : Registry::instance()->defaultCachePolicy();

    if ( cp.usage() != CachePolicy::USAGE_NO_CACHE )
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

    if ( cp.usage() != CachePolicy::USAGE_NO_CACHE )
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
