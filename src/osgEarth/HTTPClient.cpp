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

#include <curl/curl.h>
#include <osgEarth/HTTPClient>
#include <osgEarth/Registry>
#include <osgEarth/Version>
#include <osgDB/ReadFile>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>
#include <osg/Notify>
#include <string.h>
#include <sstream>
#include <fstream>
#include <iterator>
#include <iostream>
#include <algorithm>

#define LC "[HTTPClient] "

//#undef  OE_DEBUG
//#define OE_DEBUG OE_INFO

using namespace osgEarth;

//----------------------------------------------------------------------------

ProxySettings::ProxySettings( const Config& conf )
{
    mergeConfig( conf );
}

ProxySettings::ProxySettings( const std::string& host, int port ) :
_hostName(host),
_port(port)
{
    //nop
}

void
ProxySettings::mergeConfig( const Config& conf )
{
    _hostName = conf.value<std::string>( "host", "" );
    _port = conf.value<int>( "port", 8080 );
	_userName = conf.value<std::string>( "username", "" );
	_password = conf.value<std::string>( "password", "" );
}

Config
ProxySettings::getConfig() const
{
    Config conf( "proxy" );
    conf.add( "host", _hostName );
    conf.add( "port", toString(_port) );
	conf.add( "username", _userName);
	conf.add( "password", _password);

    return conf;
}

/****************************************************************************/
   
namespace osgEarth
{
    struct StreamObject
    {
        StreamObject(std::ostream* stream) : _stream(stream) { }

        void write(const char* ptr, size_t realsize)
        {
            if (_stream) _stream->write(ptr, realsize);
        }

        std::ostream* _stream;
        std::string     _resultMimeType;
    };

    static size_t
    StreamObjectReadCallback(void* ptr, size_t size, size_t nmemb, void* data)
    {
        size_t realsize = size* nmemb;
        StreamObject* sp = (StreamObject*)data;
        sp->write((const char*)ptr, realsize);
        return realsize;
    }
}

static int CurlProgressCallback(void *clientp,double dltotal,double dlnow,double ultotal,double ulnow)
{
    ProgressCallback* callback = (ProgressCallback*)clientp;
    bool cancelled = false;
    if (callback)
    {
        cancelled = callback->isCanceled() || callback->reportProgress(dlnow, dltotal);
    }
    return cancelled;
}

/****************************************************************************/

HTTPRequest::HTTPRequest( const std::string& url )
: _url( url )
{
    //NOP
}

HTTPRequest::HTTPRequest( const HTTPRequest& rhs ) :
_parameters( rhs._parameters ),
_url( rhs._url )
{
    //nop
}

void
HTTPRequest::addParameter( const std::string& name, const std::string& value )
{
    _parameters[name] = value;
}

void
HTTPRequest::addParameter( const std::string& name, int value )
{
    std::stringstream buf;
    buf << value;
	std::string bufStr;
    bufStr = buf.str();
    _parameters[name] = bufStr;
}

void
HTTPRequest::addParameter( const std::string& name, double value )
{
    std::stringstream buf;
    buf << value;
	std::string bufStr;
    bufStr = buf.str();
    _parameters[name] = bufStr;
}

const HTTPRequest::Parameters&
HTTPRequest::getParameters() const
{
    return _parameters; 
}

std::string
HTTPRequest::getURL() const
{
    if ( _parameters.size() == 0 )
    {
        return _url;
    }
    else
    {
        std::stringstream buf;
        buf << _url;
        for( Parameters::const_iterator i = _parameters.begin(); i != _parameters.end(); i++ )
        {
            buf << ( i == _parameters.begin() && _url.find( "?" ) == std::string::npos? "?" : "&" );
            buf << i->first << "=" << i->second;
        }
		std::string bufStr;
		bufStr = buf.str();
        return bufStr;
    }
}

/****************************************************************************/

HTTPResponse::HTTPResponse( long _code )
: _response_code( _code ),
  _cancelled(false)
{
    _parts.reserve(1);
}

HTTPResponse::HTTPResponse( const HTTPResponse& rhs ) :
_parts( rhs._parts ),
_response_code( rhs._response_code ),
_mimeType( rhs._mimeType ),
_cancelled( rhs._cancelled )
{
    //nop
}

unsigned
HTTPResponse::getCode() const {
    return _response_code;
}

bool
HTTPResponse::isOK() const {
    return _response_code == 200L && !isCancelled();
}

bool
HTTPResponse::isCancelled() const {
    return _cancelled;
}

unsigned int
HTTPResponse::getNumParts() const {
    return _parts.size();
}

unsigned int
HTTPResponse::getPartSize( unsigned int n ) const {
    return _parts[n]->_size;
}

const std::string&
HTTPResponse::getPartHeader( unsigned int n, const std::string& name ) const {
    return _parts[n]->_headers[name];
}

std::istream&
HTTPResponse::getPartStream( unsigned int n ) const {
    return _parts[n]->_stream;
}

std::string
HTTPResponse::getPartAsString( unsigned int n ) const {
	std::string streamStr;
	streamStr = _parts[n]->_stream.str();
    return streamStr;
}

const std::string&
HTTPResponse::getMimeType() const {
    return _mimeType;
}

Config
HTTPResponse::getHeadersAsConfig() const
{
    Config conf;
    if ( _parts.size() > 0 )
    {
        for( Part::Headers::const_iterator i = _parts[0]->_headers.begin(); i != _parts[0]->_headers.end(); ++i )
        {
            conf.set(i->first, i->second);
        }
    }
    return conf;
}

/****************************************************************************/

#define QUOTE_(X) #X
#define QUOTE(X) QUOTE_(X)
#define USER_AGENT "osgearth" QUOTE(OSGEARTH_MAJOR_VERSION) "." QUOTE(OSGEARTH_MINOR_VERSION)


static optional<ProxySettings>     _proxySettings;
static std::string                 _userAgent = USER_AGENT;


HTTPClient&
HTTPClient::getClient()
{
#if 1
    static Threading::PerThread< osg::ref_ptr<HTTPClient> > s_clientPerThread;

    osg::ref_ptr<HTTPClient>& client = s_clientPerThread.get();
    if ( !client.valid() )
        client = new HTTPClient();

    return *client.get();
#else
    typedef std::map< OpenThreads::Thread*, osg::ref_ptr<HTTPClient> > ThreadClientMap;        
    static Threading::ReadWriteMutex   _threadClientMapMutex;
    static ThreadClientMap             _threadClientMap;

    OpenThreads::Thread* current = OpenThreads::Thread::CurrentThread();

    // first try the map:
    {
        Threading::ScopedReadLock sharedLock(_threadClientMapMutex);
        ThreadClientMap::iterator i = _threadClientMap.find(current);
        if ( i != _threadClientMap.end() )
            return *i->second.get();
    }

    // not there; add it.
    {
        Threading::ScopedWriteLock exclusiveLock(_threadClientMapMutex);

        // normally, we'd double check b/c of the race condition, but since the map is being 
        // indexed by the actual thread pointer, there's no chance of a race.
        HTTPClient* client = new HTTPClient();
        _threadClientMap[current] = client;
        return *client;
    }
#endif
}

HTTPClient::HTTPClient()
{
    _previousHttpAuthentication = 0;
    _curl_handle = curl_easy_init();


	//Get the user agent
	std::string userAgent = _userAgent;
	const char* userAgentEnv = getenv("OSGEARTH_USERAGENT");
    if (userAgentEnv)
    {
		userAgent = std::string(userAgentEnv);        
    }

	OE_DEBUG << LC << "HTTPClient setting userAgent=" << userAgent << std::endl;

    curl_easy_setopt( _curl_handle, CURLOPT_USERAGENT, userAgent.c_str() );
    curl_easy_setopt( _curl_handle, CURLOPT_WRITEFUNCTION, osgEarth::StreamObjectReadCallback );
    curl_easy_setopt( _curl_handle, CURLOPT_FOLLOWLOCATION, (void*)1 );
    curl_easy_setopt( _curl_handle, CURLOPT_MAXREDIRS, (void*)5 );
    curl_easy_setopt( _curl_handle, CURLOPT_PROGRESSFUNCTION, &CurlProgressCallback);
    curl_easy_setopt( _curl_handle, CURLOPT_NOPROGRESS, (void*)0 ); //FALSE);
    //curl_easy_setopt( _curl_handle, CURLOPT_TIMEOUT, 1L );
}

HTTPClient::~HTTPClient()
{
    if (_curl_handle) curl_easy_cleanup( _curl_handle );
    _curl_handle = 0;
}

void
HTTPClient::setProxySettings( const ProxySettings &proxySettings )
{
	_proxySettings = proxySettings;
}

const std::string& HTTPClient::getUserAgent()
{
	return _userAgent;
}

void  HTTPClient::setUserAgent(const std::string& userAgent)
{
	_userAgent = userAgent;
}

void
HTTPClient::readOptions(const osgDB::Options* options, std::string& proxy_host, std::string& proxy_port) const
{
    // try to set proxy host/port by reading the CURL proxy options
    if ( options )
    {
        std::istringstream iss( options->getOptionString() );
        std::string opt;
        while( iss >> opt )
        {
            int index = opt.find( "=" );
            if( opt.substr( 0, index ) == "OSG_CURL_PROXY" )
            {
                proxy_host = opt.substr( index+1 );
            }
            else if ( opt.substr( 0, index ) == "OSG_CURL_PROXYPORT" )
            {
                proxy_port = opt.substr( index+1 );
            }
        }
    }
}

// from: http://www.rosettacode.org/wiki/Tokenizing_A_String#C.2B.2B
static std::vector<std::string> 
tokenize_str(const std::string & str, const std::string & delims=", \t")
{
  using namespace std;
  // Skip delims at beginning, find start of first token
  string::size_type lastPos = str.find_first_not_of(delims, 0);
  // Find next delimiter @ end of token
  string::size_type pos     = str.find_first_of(delims, lastPos);

  // output vector
  vector<string> tokens;

  while (string::npos != pos || string::npos != lastPos)
    {
      // Found a token, add it to the vector.
      tokens.push_back(str.substr(lastPos, pos - lastPos));
      // Skip delims.  Note the "not_of". this is beginning of token
      lastPos = str.find_first_not_of(delims, pos);
      // Find next delimiter at end of token.
      pos     = str.find_first_of(delims, lastPos);
    }

  return tokens;
}


void
HTTPClient::decodeMultipartStream(const std::string&   boundary,
                                  HTTPResponse::Part*  input,
                                  HTTPResponse::Parts& output) const
{
    std::string bstr = std::string("--") + boundary;
    std::string line;
    char tempbuf[256];

    // first thing in the stream should be the boundary.
    input->_stream.read( tempbuf, bstr.length() );
    tempbuf[bstr.length()] = 0;
    line = tempbuf;
    if ( line != bstr )
    {
        OE_WARN << LC 
            << "decodeMultipartStream: protocol violation; "
            << "expecting boundary; instead got: \"" 
            << line
            << "\"" << std::endl;
        return;
    }

    for( bool done=false; !done; )
    {
        osg::ref_ptr<HTTPResponse::Part> next_part = new HTTPResponse::Part();

        // first finish off the boundary.
        std::getline( input->_stream, line );
        if ( line == "--" )
        {
            done = true;
        }
        else
        {
            // read all headers. this ends with a blank line.
            line = " ";
            while( line.length() > 0 && !done )
            {
                std::getline( input->_stream, line );

                // check for EOS:
                if ( line == "--" )
                {
                    done = true;
                }
                else
                {
                    std::vector<std::string> tized = tokenize_str( line, ":" );
                    if ( tized.size() >= 2 )
                        next_part->_headers[tized[0]] = tized[1];
                }
            }
        }

        if ( !done )
        {
            // read data until we reach the boundary
            unsigned int bstr_ptr = 0;
            std::string temp;
            //unsigned int c = 0;
            while( bstr_ptr < bstr.length() )
            {
                char b;
                input->_stream.read( &b, 1 );
                if ( b == bstr[bstr_ptr] )
                {
                    bstr_ptr++;
                }
                else
                {
                    for( unsigned int i=0; i<bstr_ptr; i++ )
                    {
                        next_part->_stream << bstr[i];
                    }
                    next_part->_stream << b;
                    next_part->_size += bstr_ptr + 1;
                    bstr_ptr = 0;
                }
            }
            output.push_back( next_part.get() );
        }
    }
}

HTTPResponse
HTTPClient::get( const HTTPRequest&    request,
                 const osgDB::Options* options,
                 ProgressCallback*     callback)
{
    return getClient().doGet( request, options, callback );
}

HTTPResponse 
HTTPClient::get( const std::string&    url,
                 const osgDB::Options* options,
                 ProgressCallback*     callback)
{
    return getClient().doGet( url, options, callback);
}

ReadResult
HTTPClient::readImage(const std::string&    location,
                      const osgDB::Options* options,
                      ProgressCallback*     callback)
{
    return getClient().doReadImage( location, options, callback );
}

ReadResult
HTTPClient::readNode(const std::string&    location,
                     const osgDB::Options* options,
                     ProgressCallback*     callback)
{
    return getClient().doReadNode( location, options, callback );
}

ReadResult
HTTPClient::readObject(const std::string&    location,
                       const osgDB::Options* options,
                       ProgressCallback*     callback)
{
    return getClient().doReadObject( location, options, callback );
}

ReadResult
HTTPClient::readString(const std::string&    location,
                       const osgDB::Options* options,
                       ProgressCallback*     callback)
{
    return getClient().doReadString( location, options, callback );
}

bool
HTTPClient::download(const std::string& uri,
                     const std::string& localPath)
{
    return getClient().doDownload( uri, localPath );
}

HTTPResponse
HTTPClient::doGet( const HTTPRequest& request, const osgDB::Options* options, ProgressCallback* callback) const
{
    OE_DEBUG << LC << "doGet " << request.getURL() << std::endl;

    const osgDB::AuthenticationMap* authenticationMap = (options && options->getAuthenticationMap()) ? 
            options->getAuthenticationMap() :
            osgDB::Registry::instance()->getAuthenticationMap();

    std::string proxy_host;
    std::string proxy_port = "8080";

	std::string proxy_auth;

    //TODO: don't do all this proxy setup on every GET. Just do it once per client, or only when 
    // the proxy information changes.

	//Try to get the proxy settings from the global settings
	if (_proxySettings.isSet())
	{
		proxy_host = _proxySettings.get().hostName();
		std::stringstream buf;
		buf << _proxySettings.get().port();
		proxy_port = buf.str();

		std::string proxy_username = _proxySettings.get().userName();
		std::string proxy_password = _proxySettings.get().password();
		if (!proxy_username.empty() && !proxy_password.empty())
		{
			proxy_auth = proxy_username + ":" + proxy_password;
		}
	}

	//Try to get the proxy settings from the local options that are passed in.
    readOptions( options, proxy_host, proxy_port );

	//Try to get the proxy settings from the environment variable
    const char* proxyEnvAddress = getenv("OSG_CURL_PROXY");
    if (proxyEnvAddress) //Env Proxy Settings
    {
		proxy_host = std::string(proxyEnvAddress);

        const char* proxyEnvPort = getenv("OSG_CURL_PROXYPORT"); //Searching Proxy Port on Env
		if (proxyEnvPort)
		{
			proxy_port = std::string( proxyEnvPort );
		}
    }

	const char* proxyEnvAuth = getenv("OSGEARTH_CURL_PROXYAUTH");	
	if (proxyEnvAuth)
	{
		proxy_auth = std::string(proxyEnvAuth);
	}

    // Set up proxy server:
    std::string proxy_addr;
    if ( !proxy_host.empty() )
    {
        std::stringstream buf;
        buf << proxy_host << ":" << proxy_port;
		std::string bufStr;
		bufStr = buf.str();
        proxy_addr = bufStr;
    
        OE_DEBUG << LC << "setting proxy: " << proxy_addr << std::endl;
		//curl_easy_setopt( _curl_handle, CURLOPT_HTTPPROXYTUNNEL, 1 ); 
        curl_easy_setopt( _curl_handle, CURLOPT_PROXY, proxy_addr.c_str() );

		//Setup the proxy authentication if setup
		if (!proxy_auth.empty())
		{
			OE_DEBUG << LC << "Setting up proxy authentication " << proxy_auth << std::endl;
			curl_easy_setopt( _curl_handle, CURLOPT_PROXYUSERPWD, proxy_auth.c_str());
		}
    }

    const osgDB::AuthenticationDetails* details = authenticationMap ?
        authenticationMap->getAuthenticationDetails(request.getURL()) :
        0;

        if (details)
        {
            const std::string colon(":");
            std::string password(details->username + colon + details->password);
            curl_easy_setopt(_curl_handle, CURLOPT_USERPWD, password.c_str());
            const_cast<HTTPClient*>(this)->_previousPassword = password;

            // use for https.
            // curl_easy_setopt(_curl, CURLOPT_KEYPASSWD, password.c_str());

#if LIBCURL_VERSION_NUM >= 0x070a07
            if (details->httpAuthentication != _previousHttpAuthentication)
            { 
                curl_easy_setopt(_curl_handle, CURLOPT_HTTPAUTH, details->httpAuthentication); 
                const_cast<HTTPClient*>(this)->_previousHttpAuthentication = details->httpAuthentication;
            }
#endif
    }
    else
    {
        if (!_previousPassword.empty())
        {
            curl_easy_setopt(_curl_handle, CURLOPT_USERPWD, 0);
            const_cast<HTTPClient*>(this)->_previousPassword.clear();
        }

#if LIBCURL_VERSION_NUM >= 0x070a07
        // need to reset if previously set.
        if (_previousHttpAuthentication!=0)
        {
            curl_easy_setopt(_curl_handle, CURLOPT_HTTPAUTH, 0); 
            const_cast<HTTPClient*>(this)->_previousHttpAuthentication = 0;
        }
#endif
    }

    osg::ref_ptr<HTTPResponse::Part> part = new HTTPResponse::Part();
    StreamObject sp( &part->_stream );

    //Take a temporary ref to the callback
    osg::ref_ptr<ProgressCallback> progressCallback = callback;
    curl_easy_setopt( _curl_handle, CURLOPT_URL, request.getURL().c_str() );
    if (callback)
    {
        curl_easy_setopt(_curl_handle, CURLOPT_PROGRESSDATA, progressCallback.get());
    }

    char errorBuf[CURL_ERROR_SIZE];
    errorBuf[0] = 0;
    curl_easy_setopt( _curl_handle, CURLOPT_ERRORBUFFER, (void*)errorBuf );

    curl_easy_setopt( _curl_handle, CURLOPT_WRITEDATA, (void*)&sp);
    CURLcode res = curl_easy_perform( _curl_handle );
    curl_easy_setopt( _curl_handle, CURLOPT_WRITEDATA, (void*)0 );
    curl_easy_setopt( _curl_handle, CURLOPT_PROGRESSDATA, (void*)0);

    long response_code = 0L;
	if (!proxy_addr.empty())
	{
		long connect_code = 0L;
        curl_easy_getinfo( _curl_handle, CURLINFO_HTTP_CONNECTCODE, &connect_code );
		OE_DEBUG << LC << "proxy connect code " << connect_code << std::endl;
	}
	
    curl_easy_getinfo( _curl_handle, CURLINFO_RESPONSE_CODE, &response_code );     

	//OE_DEBUG << LC << "got response, code = " << response_code << std::endl;

    HTTPResponse response( response_code );
   
    if ( response_code == 200L && res != CURLE_ABORTED_BY_CALLBACK && res != CURLE_OPERATION_TIMEDOUT ) //res == 0 )
    {
        // check for multipart content:
        char* content_type_cp;
        curl_easy_getinfo( _curl_handle, CURLINFO_CONTENT_TYPE, &content_type_cp );
        if ( content_type_cp == NULL )
        {
            OE_NOTICE << LC
                << "NULL Content-Type (protocol violation) " 
                << "URL=" << request.getURL() << std::endl;
            return HTTPResponse();
        }

        // NOTE:
        //   WCS 1.1 specified a "multipart/mixed" response, but ArcGIS Server gives a "multipart/related"
        //   content type ...

        std::string content_type( content_type_cp );

        //OE_DEBUG << LC << "content-type = \"" << content_type << "\"" << std::endl;

        if ( content_type.length() > 9 && ::strstr( content_type.c_str(), "multipart" ) == content_type.c_str() )
        //if ( content_type == "multipart/mixed; boundary=wcs" ) //todo: parse this.
        {
            OE_DEBUG << LC << "detected multipart data; decoding..." << std::endl;

            //TODO: parse out the "wcs" -- this is WCS-specific
            decodeMultipartStream( "wcs", part.get(), response._parts );
        }
        else
        {
            // store headers that we care about
            part->_headers[IOMetadata::CONTENT_TYPE] = content_type;

            response._parts.push_back( part.get() );
        }
    }
    else if (res == CURLE_ABORTED_BY_CALLBACK || res == CURLE_OPERATION_TIMEDOUT)
    {
        //If we were aborted by a callback, then it was cancelled by a user
        response._cancelled = true;
    }

    // Store the mime-type, if any. (Note: CURL manages the buffer returned by
    // this call.)
    char* ctbuf = NULL;
    if ( curl_easy_getinfo(_curl_handle, CURLINFO_CONTENT_TYPE, &ctbuf) == 0 && ctbuf )
    {
        response._mimeType = ctbuf;
    }

    return response;
}


HTTPResponse
HTTPClient::doGet( const std::string& url, const osgDB::Options* options, ProgressCallback* callback) const
{
    return doGet( HTTPRequest(url), options, callback );
}

bool
HTTPClient::doDownload(const std::string& url, const std::string& filename)
{
    // download the data
    HTTPResponse response = this->doGet( HTTPRequest(url) );

    if ( response.isOK() )
    {
        unsigned int part_num = response.getNumParts() > 1? 1 : 0;
        std::istream& input_stream = response.getPartStream( part_num );

        std::ofstream fout;
        fout.open(filename.c_str(), std::ios::out | std::ios::binary);

        input_stream.seekg (0, std::ios::end);
        int length = input_stream.tellg();
        input_stream.seekg (0, std::ios::beg);

        char *buffer = new char[length];
        input_stream.read(buffer, length);
        fout.write(buffer, length);
        delete[] buffer;
        fout.close();
        return true;
    }
    else
    {
        OE_WARN << LC << "Error downloading file " << filename << std::endl;
        return false;
    } 
}

namespace
{
    osgDB::ReaderWriter*
    getReader( const std::string& url, const HTTPResponse& response )
    {
        osgDB::ReaderWriter* reader = 0L;

        // try to look up a reader by mime-type first:
        std::string mimeType = response.getMimeType();
        if ( !mimeType.empty() )
        {
            reader = osgEarth::Registry::instance()->getReaderWriterForMimeType(mimeType);
        }

        if ( !reader )
        {
            // Try to find a reader by file extension.
            std::string ext = osgDB::getFileExtension( url );
            reader = osgDB::Registry::instance()->getReaderWriterForExtension( ext );
        }

        return reader;
    }
}

ReadResult
HTTPClient::doReadImage(const std::string&    location,
                        const osgDB::Options* options,
                        ProgressCallback*     callback)
{
    ReadResult result;

    HTTPResponse response = this->doGet(location, options, callback);

    if (response.isOK())
    {
        osgDB::ReaderWriter* reader = getReader(location, response);
        if (!reader)
        {
            OE_WARN << LC << "Can't find an OSG plugin to read "<<location<<std::endl;
            result = ReadResult(ReadResult::RESULT_NO_READER);
        }

        else 
        {
            osgDB::ReaderWriter::ReadResult rr = reader->readImage(response.getPartStream(0), options);
            if ( rr.validImage() )
            {
                result = ReadResult(rr.takeImage(), response.getHeadersAsConfig() );
            }
            else 
            {
                if ( !rr.message().empty() )
                {
                    OE_WARN << LC << "HTTP error: " << rr.message() << std::endl;
                }
                OE_WARN << LC << reader->className() << " failed to read image from " << location << std::endl;
                result = ReadResult(ReadResult::RESULT_READER_ERROR);
            }
        }
    }
    else
    {
        result = ReadResult(
            response.isCancelled() ? ReadResult::RESULT_CANCELED :
            response.getCode() == HTTPResponse::NOT_FOUND ? ReadResult::RESULT_NOT_FOUND :
            response.getCode() == HTTPResponse::SERVER_ERROR ? ReadResult::RESULT_SERVER_ERROR :
            ReadResult::RESULT_UNKNOWN_ERROR );

        //If we have an error but it's recoverable, like a server error or timeout then set the callback to retry.
        if (HTTPClient::isRecoverable( result.code() ) )
        {
            if (callback)
            {
                OE_DEBUG << "Error in HTTPClient for " << location << " but it's recoverable" << std::endl;
                callback->setNeedsRetry( true );
            }
        }
    }

    return result;
}

ReadResult
HTTPClient::doReadNode(const std::string&    location,
                       const osgDB::Options* options,
                       ProgressCallback*     callback)
{
    ReadResult result;

    HTTPResponse response = this->doGet(location, options, callback);

    if (response.isOK())
    {
        osgDB::ReaderWriter* reader = getReader(location, response);
        if (!reader)
        {
            OE_WARN << LC << "Can't find an OSG plugin to read "<<location<<std::endl;
            result = ReadResult(ReadResult::RESULT_NO_READER);
        }

        else 
        {
            osgDB::ReaderWriter::ReadResult rr = reader->readNode(response.getPartStream(0), options);
            if ( rr.validNode() )
            {
                result = ReadResult(rr.takeNode(), response.getHeadersAsConfig());
            }
            else 
            {
                if ( !rr.message().empty() )
                {
                    OE_WARN << LC << "HTTP error: " << rr.message() << std::endl;
                }
                OE_WARN << LC << reader->className() << " failed to read node from " << location << std::endl;
                result = ReadResult(ReadResult::RESULT_READER_ERROR);
            }
        }
    }
    else
    {
        result = ReadResult(
            response.isCancelled() ? ReadResult::RESULT_CANCELED :
            response.getCode() == HTTPResponse::NOT_FOUND ? ReadResult::RESULT_NOT_FOUND :
            response.getCode() == HTTPResponse::SERVER_ERROR ? ReadResult::RESULT_SERVER_ERROR :
            ReadResult::RESULT_UNKNOWN_ERROR );

        //If we have an error but it's recoverable, like a server error or timeout then set the callback to retry.
        if (HTTPClient::isRecoverable( result.code() ) )
        {
            if (callback)
            {
                OE_DEBUG << "Error in HTTPClient for " << location << " but it's recoverable" << std::endl;
                callback->setNeedsRetry( true );
            }
        }
    }

    return result;
}

ReadResult
HTTPClient::doReadObject(const std::string&    location,
                         const osgDB::Options* options,
                         ProgressCallback*     callback)
{
    ReadResult result;

    HTTPResponse response = this->doGet(location, options, callback);

    if (response.isOK())
    {
        osgDB::ReaderWriter* reader = getReader(location, response);
        if (!reader)
        {
            OE_WARN << LC << "Can't find an OSG plugin to read "<<location<<std::endl;
            result = ReadResult(ReadResult::RESULT_NO_READER);
        }

        else 
        {
            osgDB::ReaderWriter::ReadResult rr = reader->readObject(response.getPartStream(0), options);
            if ( rr.validObject() )
            {
                result = ReadResult(rr.takeObject(), response.getHeadersAsConfig());
            }
            else 
            {
                if ( !rr.message().empty() )
                {
                    OE_WARN << LC << "HTTP error: " << rr.message() << std::endl;
                }
                OE_WARN << LC << reader->className() << " failed to read object from " << location << std::endl;
                result = ReadResult(ReadResult::RESULT_READER_ERROR);
            }
        }
    }
    else
    {
        result = ReadResult(
            response.isCancelled() ? ReadResult::RESULT_CANCELED :
            response.getCode() == HTTPResponse::NOT_FOUND ? ReadResult::RESULT_NOT_FOUND :
            response.getCode() == HTTPResponse::SERVER_ERROR ? ReadResult::RESULT_SERVER_ERROR :
            ReadResult::RESULT_UNKNOWN_ERROR );

        //If we have an error but it's recoverable, like a server error or timeout then set the callback to retry.
        if (HTTPClient::isRecoverable( result.code() ) )
        {
            if (callback)
            {
                OE_DEBUG << "Error in HTTPClient for " << location << " but it's recoverable" << std::endl;
                callback->setNeedsRetry( true );
            }
        }
    }

    return result;
}


ReadResult
HTTPClient::doReadString(const std::string&    location,
                         const osgDB::Options* options,
                         ProgressCallback*     callback )
{
    ReadResult result;

    HTTPResponse response = this->doGet( location, options, callback );
    if ( response.isOK() )
    {
        result = ReadResult( new StringObject(response.getPartAsString(0)), response.getHeadersAsConfig());
    }
    else
    {
        result = ReadResult(
            response.isCancelled() ? ReadResult::RESULT_CANCELED :
            response.getCode() == HTTPResponse::NOT_FOUND ? ReadResult::RESULT_NOT_FOUND :
            response.getCode() == HTTPResponse::SERVER_ERROR ? ReadResult::RESULT_SERVER_ERROR :
            ReadResult::RESULT_UNKNOWN_ERROR );

        //If we have an error but it's recoverable, like a server error or timeout then set the callback to retry.
        if (HTTPClient::isRecoverable( result.code() ) )
        {
            if (callback)
            {
                OE_DEBUG << "Error in HTTPClient for " << location << " but it's recoverable" << std::endl;
                callback->setNeedsRetry( true );
            }
        }
    }

    return result;
}
