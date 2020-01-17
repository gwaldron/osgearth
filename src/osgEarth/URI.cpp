/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarth/Registry>
#include <osgEarth/FileUtils>
#include <osgEarth/Progress>
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>
#include <osgDB/Archive>

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

URIStream::URIStream(const URI& uri, std::ios_base::openmode mode) :
_instream( 0L )
{
    if ( osgDB::containsServerAddress(uri.full()) )
    {
        HTTPResponse res = HTTPClient::get( uri.full() );
        if ( res.isOK() )
        {
            std::string buf = res.getPartAsString(0);
            _instream = new std::istringstream(buf);
        }
    }
    else
    {
        _instream = new std::ifstream(uri.full().c_str(), mode);
    }
}

URIStream::~URIStream()
{
    if (_instream)
        delete _instream;
}

URIStream::operator std::istream& ()
{
    static std::istringstream s_nullStream;

    if ( _instream )
        return *_instream;
    else
        return s_nullStream;
}

//------------------------------------------------------------------------

URIContext::URIContext()
{
}

URIContext::URIContext(const std::string& referrer) :
    _referrer(referrer)
{
}

URIContext::URIContext(const URIContext& rhs) :
    _referrer(rhs._referrer),
    _headers(rhs._headers)
{
}

std::string
URIContext::getOSGPath( const std::string& target ) const
{
    return osgEarth::getFullPath( _referrer, target );
}

void
URIContext::addHeader(const std::string& name, const std::string& value)
{
    _headers[name] = value;
}

const Headers&
URIContext::getHeaders() const
{
    return _headers;
}

Headers&
URIContext::getHeaders()
{
    return _headers;
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
        if (_referrer.empty() == false)
        {
            options->setDatabasePath( _referrer );
            options->setPluginStringData( "osgEarth::URIContext::referrer", _referrer );
        }
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

URI::URI(const URI& rhs) :
_baseURI(rhs._baseURI),
_fullURI(rhs._fullURI),
_context(rhs._context),
_cacheKey(rhs._cacheKey)
{
    //nop
}

URI::URI( const std::string& location )
{
    _baseURI = location;
    _fullURI = location;
    ctorCacheKey();
}

URI::URI( const std::string& location, const URIContext& context )
{
    _context = context;
    _baseURI = location;
    _fullURI = context.getOSGPath( _baseURI );
    ctorCacheKey();
}

URI::URI( const char* location )
{
    _baseURI = std::string(location);
    _fullURI = _baseURI;
    ctorCacheKey();
}

URI
URI::append( const std::string& suffix ) const
{
    URI result;
    result._baseURI = _baseURI + suffix;
    result._fullURI = _fullURI + suffix;
    result._context = _context;
    result.ctorCacheKey();
    return result;
}

void
URI::ctorCacheKey()
{
    _cacheKey = Cache::makeCacheKey(_fullURI, "uri");
}

bool
URI::isRemote() const
{
    return osgDB::containsServerAddress( _fullURI );
}

Config
URI::getConfig() const
{
    Config conf("uri", base());
    conf.set("option_string", _optionString);
    conf.setReferrer(context().referrer());
    conf.setIsLocation(true);

    const Headers& headers = context().getHeaders();
    if (!headers.empty())
    {
        Config headersconf("headers");
        for(Headers::const_iterator i = headers.begin(); i != headers.end(); ++i)
        {
            if (!i->first.empty() && !i->second.empty())
            {
                headersconf.add(Config(i->first, i->second));
            }
        }
        conf.add(headersconf);
    }

    return conf;
}

void
URI::mergeConfig(const Config& conf)
{    
    conf.get("option_string", _optionString);

    const ConfigSet headers = conf.child("headers").children();
    for (ConfigSet::const_iterator i = headers.begin(); i != headers.end(); ++i)
    {
        const Config& header = *i;
        if (!header.key().empty() && !header.value().empty())
        {
            _context.addHeader(header.key(), header.value());
        }
    }
}

namespace
{
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
        std::ifstream input( uri.c_str(), std::ios::binary );
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
        ReadResult fromCache( CacheBin* bin, const std::string& key) { return bin->readObject(key, 0L); }
        ReadResult fromHTTP( const URI& uri, const osgDB::Options* opt, ProgressCallback* p, TimeStamp lastModified )
        {
            HTTPRequest req(uri.full());
            req.getHeaders() = uri.context().getHeaders();
            if (lastModified > 0)
            {
                req.setLastModified(lastModified);
            }
            return HTTPClient::readObject(req, opt, p);
        }
        ReadResult fromFile( const std::string& uri, const osgDB::Options* opt ) {
            return ReadResult(osgDB::readRefObjectFile(uri, opt).get());
        }
    };

    struct ReadNode
    {
        bool callbackRequestsCaching( URIReadCallback* cb ) const { return !cb || ((cb->cachingSupport() & URIReadCallback::CACHE_NODES) != 0); }
        ReadResult fromCallback( URIReadCallback* cb, const std::string& uri, const osgDB::Options* opt ) { return cb->readNode(uri, opt); }
        ReadResult fromCache( CacheBin* bin, const std::string& key ) { return bin->readObject(key, 0L); }
        ReadResult fromHTTP(const URI& uri, const osgDB::Options* opt, ProgressCallback* p, TimeStamp lastModified )
        {
            HTTPRequest req(uri.full());
            req.getHeaders() = uri.context().getHeaders();
            if (lastModified > 0)
            {
                req.setLastModified(lastModified);
            }
            return HTTPClient::readNode(req, opt, p);
        }
        ReadResult fromFile( const std::string& uri, const osgDB::Options* opt ) {
            return ReadResult(osgDB::readRefNodeFile(uri, opt));
        }
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
        ReadResult fromCache( CacheBin* bin, const std::string& key) {
            ReadResult r = bin->readImage(key, 0L);
            if ( r.getImage() ) r.getImage()->setFileName( key );
            return r;
        }
        ReadResult fromHTTP(const URI& uri, const osgDB::Options* opt, ProgressCallback* p, TimeStamp lastModified ) {
            HTTPRequest req(uri.full());
            req.getHeaders() = uri.context().getHeaders();

            if (lastModified > 0)
            {
                req.setLastModified(lastModified);
            }
            ReadResult r = HTTPClient::readImage(req, opt, p);
            if ( r.getImage() ) r.getImage()->setFileName( uri.full() );
            return r;
        }
        ReadResult fromFile( const std::string& uri, const osgDB::Options* opt ) {
            ReadResult r = ReadResult(osgDB::readRefImageFile(uri, opt));
            if ( r.getImage() ) r.getImage()->setFileName( uri );
            return r;
        }
    };

    struct ReadString
    {
        bool callbackRequestsCaching( URIReadCallback* cb ) const {
            return !cb || ((cb->cachingSupport() & URIReadCallback::CACHE_STRINGS) != 0);
        }
        ReadResult fromCallback( URIReadCallback* cb, const std::string& uri, const osgDB::Options* opt ) { 
            return cb->readString(uri, opt);
        }
        ReadResult fromCache( CacheBin* bin, const std::string& key) { 
            return bin->readString(key, 0L);
        }
        ReadResult fromHTTP(const URI& uri, const osgDB::Options* opt, ProgressCallback* p, TimeStamp lastModified )
        {
            HTTPRequest req(uri.full());
            req.getHeaders() = uri.context().getHeaders();
            if (lastModified > 0)
            {
                req.setLastModified(lastModified);
            }
            return HTTPClient::readString(req, opt, p);
        }
        ReadResult fromFile( const std::string& uri, const osgDB::Options* opt ) {
            return readStringFile(uri, opt);
        }
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

        if (osgEarth::Registry::instance()->isBlacklisted(inputURI.full()))
        {
            return result;
        }

        if ( !inputURI.empty() )
        {
            // establish our IO options:
            osg::ref_ptr<const osgDB::Options> localOptions = dbOptions ? dbOptions : Registry::instance()->getDefaultOptions();

            // if we have an option string, incorporate it.
            if ( inputURI.optionString().isSet() )
            {
                osgDB::Options* newLocalOptions = Registry::cloneOrCreateOptions(localOptions.get());
                newLocalOptions->setOptionString(
                    inputURI.optionString().get() + " " + localOptions->getOptionString());
                localOptions = newLocalOptions;
            }

            READ_FUNCTOR reader;

            URI uri = inputURI;

            bool gotResultFromCallback = false;

            // check if there's an alias map, and if so, attempt to resolve the alias:
            URIAliasMap* aliasMap = URIAliasMap::from( localOptions.get() );
            if ( aliasMap )
            {
                uri = aliasMap->resolve(inputURI.full(), inputURI.context());
            }

            // check if there's a URI cache in the options.
            URIResultCache* memCache = URIResultCache::from( localOptions.get() );
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
                        result = reader.fromCallback( cb, uri.full(), localOptions.get() );

                        if ( result.code() != ReadResult::RESULT_NOT_IMPLEMENTED )
                        {
                            // "not implemented" is the only excuse to fall back.
                            gotResultFromCallback = true;
                        }
                    }

                    if ( !gotResultFromCallback )
                    {
                        // no callback, just read from a local file.
                        result = reader.fromFile( uri.full(), localOptions.get() );
                    }
                }

                // remote URI, consider caching:
                else
                {
                    bool callbackCachingOK = !cb || reader.callbackRequestsCaching(cb);

                    optional<CachePolicy> cp;
                    osg::ref_ptr<CacheBin> bin;

                    CacheSettings* cacheSettings = CacheSettings::get(localOptions.get());
                    if (cacheSettings)
                    {
                        cp = cacheSettings->cachePolicy();
                        if (cp->isCacheEnabled() && callbackCachingOK)
                        {
                            bin = cacheSettings->getCacheBin();
                        }
                    }

                    bool expired = false;
                    // first try to go to the cache if there is one:
                    if ( bin && cp->isCacheReadable() )
                    {
                        result = reader.fromCache( bin.get(), uri.cacheKey() );
                        if ( result.succeeded() )
                        {
                            expired = cp->isExpired(result.lastModifiedTime());
                            result.setIsFromCache(true);
                        }
                    }

                    // If it's not cached, or it is cached but is expired then try to hit the server.
                    if ( result.empty() || expired )
                    {
                        // Need to do this to support nested PLODs and Proxynodes.
                        osg::ref_ptr<osgDB::Options> remoteOptions =
                            Registry::instance()->cloneOrCreateOptions( localOptions.get() );
                        remoteOptions->getDatabasePathList().push_front( osgDB::getFilePath(uri.full()) );

                        // Store the existing object from the cache if there is one.
                        osg::ref_ptr< osg::Object > object = result.getObject();

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
                            if ( (result.empty() || expired) && cp->usage() != CachePolicy::USAGE_CACHE_ONLY )
                            {
                                ReadResult remoteResult = reader.fromHTTP( uri, remoteOptions.get(), progress, result.lastModifiedTime() );
                                if (remoteResult.code() == ReadResult::RESULT_NOT_MODIFIED)
                                {
                                    OE_DEBUG << LC << uri.full() << " not modified, using cached result" << std::endl;
                                    // Touch the cached item to update it's last modified timestamp so it doesn't expire again immediately.
                                    if (bin)
                                        bin->touch( uri.cacheKey() );
                                }
                                else
                                {
                                    OE_DEBUG << LC << "Got remote result for " << uri.full() << std::endl;
                                    result = remoteResult;
                                }
                            }

                            // Check for cancelation before a cache write
                            if (progress && progress->isCanceled())
                            {
                                return 0L;
                            }

                            // write the result to the cache if possible:
                            if ( result.succeeded() && !result.isFromCache() && bin && cp->isCacheWriteable() && bin )
                            {
                                OE_DEBUG << LC << "Writing " << uri.cacheKey() << " to cache" << std::endl;
                                bin->write( uri.cacheKey(), result.getObject(), result.metadata(), remoteOptions.get() );
                            }
                        }
                    }
                }

                // Check for cancelation before a potential cache write
                if (progress && progress->isCanceled())
                {
                    return 0L;
                }

                if (result.getObject() && !gotResultFromCallback)
                {
                    result.getObject()->setName( uri.base() );

                    if ( memCache )
                    {
                        memCache->insert( uri, result );
                    }
                }

                // If the request failed with an unrecoverable error,
                // blacklist so we don't waste time on it again
                if (result.failed())
                {
                    osgEarth::Registry::instance()->blacklist(inputURI.full());
                }
            }

            OE_TEST << LC
                << uri.base() << ": "
                << (result.succeeded() ? "OK" : "FAILED")
                //<< "; policy=" << cp->usageString()
                << (result.isFromCache() && result.succeeded() ? "; (from cache)" : "")
                << std::endl;
        }

        // post-process if there's a post-URI callback.
        URIPostReadCallback* post = URIPostReadCallback::from(dbOptions);
        if ( post )
        {
            (*post)(result);
        }

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
