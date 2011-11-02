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

#if 0
URI::ResultCode
URI::readObject(osg::ref_ptr<osg::Object>& output,
                const osgDB::Options*      dbOptions,
                const CachePolicy&         cachePolicy ) const
{
    if ( empty() )
        return RESULT_NOT_FOUND;

    osg::ref_ptr<const osgDB::Options> myDBOptions = fixOptions(dbOptions);

    osg::ref_ptr<osg::Object> object;
    ResultCode result = (ResultCode)HTTPClient::readObjectFile( _fullURI, object, myDBOptions.get() );
    if ( out_code ) *out_code = result;

    return object.release();
}
#endif

namespace
{
    // extracts a CacheBin from the dboptions; if one cannot be found, fall back on the
    // default CacheBin of a Cache found in the dboptions; failing that, call back on
    // the default CacheBin of the registry-wide cache.
    CacheBin* s_getCacheBin( const osgDB::Options* dbOptions )
    {
        CacheBin* bin = CacheBin::get( dbOptions );
        if ( !bin )
        {
            Cache* cache = Cache::get( dbOptions );
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
}

URI::ResultCode
URI::readObject(osg::ref_ptr<osg::Object>& output,
                const osgDB::Options*      dbOptions,
                const CachePolicy&         cachePolicy ) const
{
    output = 0L;

    if ( empty() )
        return RESULT_NOT_FOUND;

    ResultCode code   = RESULT_OK;
    CacheBin*  bin    = 0L;

    if ( cachePolicy.usage() != CachePolicy::USAGE_NO_CACHE )
    {
        bin = s_getCacheBin( dbOptions );
    }

    if ( bin && cachePolicy.isCacheReadable() )
    {
        bin->readObject( output, full(), *cachePolicy.maxAge() );
    }

    if ( !output.valid() )
    {
        code = (ResultCode)HTTPClient::readObjectFile( 
            _fullURI, 
            output, 
            dbOptions ? dbOptions : Registry::instance()->getDefaultOptions() );

        if ( code == RESULT_OK && output.valid() && bin && cachePolicy.isCacheWriteable() )
        {
            bin->write( full(), output.get() );
        }
    }

    return code;
}

URI::ResultCode
URI::readImage(osg::ref_ptr<osg::Image>& output,
               const osgDB::Options*     dbOptions,
               const CachePolicy&        cachePolicy ) const
{
    output = 0L;

    if ( empty() )
        return RESULT_NOT_FOUND;

    ResultCode code   = RESULT_OK;
    CacheBin*  bin    = 0L;

    if ( cachePolicy.usage() != CachePolicy::USAGE_NO_CACHE )
    {
        bin = s_getCacheBin( dbOptions );
    }

    if ( bin && cachePolicy.isCacheReadable() )
    {
        bin->readImage( output, full(), *cachePolicy.maxAge() );
    }

    if ( !output.valid() )
    {
        code = (ResultCode)HTTPClient::readImageFile( 
            _fullURI, 
            output, 
            dbOptions ? dbOptions : Registry::instance()->getDefaultOptions() );

        if ( code == RESULT_OK && output.valid() && bin && cachePolicy.isCacheWriteable() )
        {
            bin->write( full(), output.get() );
        }
    }

    return code;
}

URI::ResultCode
URI::readNode(osg::ref_ptr<osg::Node>&  output,
              const osgDB::Options*     dbOptions,
              const CachePolicy&        cachePolicy ) const
{
    output = 0L;

    if ( empty() )
        return RESULT_NOT_FOUND;

    ResultCode code   = RESULT_OK;
    CacheBin*  bin    = 0L;

    if ( cachePolicy.usage() != CachePolicy::USAGE_NO_CACHE )
    {
        bin = s_getCacheBin( dbOptions );
    }

    if ( bin && cachePolicy.isCacheReadable() )
    {
        bin->readObject( output, full(), *cachePolicy.maxAge() );
    }

    if ( !output.valid() )
    {
        code = (ResultCode)HTTPClient::readNodeFile(
            _fullURI, 
            output, 
            dbOptions ? dbOptions : Registry::instance()->getDefaultOptions() );

        if ( code == RESULT_OK && output.valid() && bin && cachePolicy.isCacheWriteable() )
        {
            bin->write( full(), output.get() );
        }
    }

    return code;
}

URI::ResultCode
URI::readString(std::string&              output,
                const osgDB::Options*     dbOptions,
                const CachePolicy&        cachePolicy ) const
{
    output.clear();

    if ( empty() )
        return RESULT_NOT_FOUND;

    ResultCode code   = RESULT_OK;
    CacheBin*  bin    = 0L;

    if ( cachePolicy.usage() != CachePolicy::USAGE_NO_CACHE )
    {
        bin = s_getCacheBin( dbOptions );
    }

    bool cacheReadOK = false;
    if ( bin && cachePolicy.isCacheReadable() )
    {
        cacheReadOK = bin->readString( output, full(), *cachePolicy.maxAge() );
    }

    if ( !cacheReadOK )
    {
        code = (ResultCode)HTTPClient::readString( _fullURI, output, 0L );

        if ( code == RESULT_OK && output.size()>0 && bin && cachePolicy.isCacheWriteable() )
        {
            bin->write( full(), output );
        }
    }

    return code;
}
