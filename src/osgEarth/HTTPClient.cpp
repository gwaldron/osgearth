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
#include <osgEarth/HTTPClient>
#include <osgEarth/Registry>
#include <osgEarth/Version>
#include <osgEarth/Progress>
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
#include <curl/curl.h>

#define LC "[HTTPClient] "

//#define OE_TEST OE_NOTICE
#define OE_TEST OE_NULL

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

bool
ProxySettings::fromOptions( const osgDB::Options* dbOptions, optional<ProxySettings>& out )
{
    if ( dbOptions )
    {
        std::string jsonString = dbOptions->getPluginStringData( "osgEarth::ProxySettings" );
        if ( !jsonString.empty() )
        {
            Config conf;
            conf.fromJSON( jsonString );
            out = ProxySettings( conf );
            return true;
        }
    }
    return false;
}

void
ProxySettings::apply( osgDB::Options* dbOptions ) const
{
    if ( dbOptions )
    {
        Config conf = getConfig();
        dbOptions->setPluginStringData( "osgEarth::ProxySettings", conf.toJSON() );
    }
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
_response_code( rhs._response_code ),
_parts( rhs._parts ),
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


namespace
{
    // TODO: consider moving this stuff into the osgEarth::Registry;
    // don't like it here in the global scope
    // per-thread client map (must be global scope)
    static Threading::PerThread<HTTPClient> s_clientPerThread;

    static optional<ProxySettings>     s_proxySettings;

    static std::string                 s_userAgent = USER_AGENT;

    static long                        s_timeout = 0;
    static long                        s_connectTimeout = 0;

    // HTTP debugging.
    static bool                        s_HTTP_DEBUG = false;

    static osg::ref_ptr< URLRewriter > s_rewriter;
}

HTTPClient&
HTTPClient::getClient()
{
    return s_clientPerThread.get();
}

HTTPClient::HTTPClient() :
_initialized    ( false ),
_curl_handle    ( 0L ),
_simResponseCode( -1L )
{
    //nop
    //do no CURL calls here.
}

void
HTTPClient::initialize() const
{
    if ( !_initialized )
    {
        const_cast<HTTPClient*>(this)->initializeImpl();
    }
}

void
HTTPClient::initializeImpl()
{
    _previousHttpAuthentication = 0;
    _curl_handle = curl_easy_init();

    //Get the user agent
    std::string userAgent = s_userAgent;
    const char* userAgentEnv = getenv("OSGEARTH_USERAGENT");
    if (userAgentEnv)
    {
        userAgent = std::string(userAgentEnv);
    }

    //Check for a response-code simulation (for testing)
    const char* simCode = getenv("OSGEARTH_SIMULATE_HTTP_RESPONSE_CODE");
    if ( simCode )
    {
        _simResponseCode = osgEarth::as<long>(std::string(simCode), 404L);
        OE_WARN << LC << "Simulating a network error with Response Code = " << _simResponseCode << std::endl;
    }

    // Dumps out HTTP request/response info
    if ( ::getenv("OSGEARTH_HTTP_DEBUG") )
    {
        s_HTTP_DEBUG = true;
        OE_WARN << LC << "HTTP debugging enabled" << std::endl;
    }

    OE_DEBUG << LC << "HTTPClient setting userAgent=" << userAgent << std::endl;

    curl_easy_setopt( _curl_handle, CURLOPT_USERAGENT, userAgent.c_str() );
    curl_easy_setopt( _curl_handle, CURLOPT_WRITEFUNCTION, osgEarth::StreamObjectReadCallback );
    curl_easy_setopt( _curl_handle, CURLOPT_FOLLOWLOCATION, (void*)1 );
    curl_easy_setopt( _curl_handle, CURLOPT_MAXREDIRS, (void*)5 );
    curl_easy_setopt( _curl_handle, CURLOPT_PROGRESSFUNCTION, &CurlProgressCallback);
    curl_easy_setopt( _curl_handle, CURLOPT_NOPROGRESS, (void*)0 ); //FALSE);
    curl_easy_setopt( _curl_handle, CURLOPT_FILETIME, true );

    long timeout = s_timeout;
    const char* timeoutEnv = getenv("OSGEARTH_HTTP_TIMEOUT");
    if (timeoutEnv)
    {
        timeout = osgEarth::as<long>(std::string(timeoutEnv), 0);
    }
    OE_DEBUG << LC << "Setting timeout to " << timeout << std::endl;
    curl_easy_setopt( _curl_handle, CURLOPT_TIMEOUT, timeout );
    long connectTimeout = s_connectTimeout;
    const char* connectTimeoutEnv = getenv("OSGEARTH_HTTP_CONNECTTIMEOUT");
    if (connectTimeoutEnv)
    {
        connectTimeout = osgEarth::as<long>(std::string(connectTimeoutEnv), 0);
    }
    OE_DEBUG << LC << "Setting connect timeout to " << connectTimeout << std::endl;
    curl_easy_setopt( _curl_handle, CURLOPT_CONNECTTIMEOUT, connectTimeout );

    _initialized = true;
}

HTTPClient::~HTTPClient()
{
    if (_curl_handle) curl_easy_cleanup( _curl_handle );
    _curl_handle = 0;
}

void
HTTPClient::setProxySettings( const ProxySettings& proxySettings )
{
    s_proxySettings = proxySettings;
}

const std::string& HTTPClient::getUserAgent()
{
    return s_userAgent;
}

void  HTTPClient::setUserAgent(const std::string& userAgent)
{
    s_userAgent = userAgent;
}

long HTTPClient::getTimeout()
{
    return s_timeout;
}

void HTTPClient::setTimeout( long timeout )
{
    s_timeout = timeout;
}

long HTTPClient::getConnectTimeout()
{
    return s_connectTimeout;
}

void HTTPClient::setConnectTimeout( long timeout )
{
    s_connectTimeout = timeout;
}
URLRewriter* HTTPClient::getURLRewriter()
{
    return s_rewriter.get();
}

void HTTPClient::setURLRewriter( URLRewriter* rewriter )
{
    s_rewriter = rewriter;
}

void
HTTPClient::globalInit()
{
    curl_global_init(CURL_GLOBAL_ALL);
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

namespace
{
    // from: http://www.rosettacode.org/wiki/Tokenizing_A_String#C.2B.2B
    std::vector<std::string> 
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
    initialize();

    const osgDB::AuthenticationMap* authenticationMap = (options && options->getAuthenticationMap()) ? 
            options->getAuthenticationMap() :
            osgDB::Registry::instance()->getAuthenticationMap();

    std::string proxy_host;
    std::string proxy_port = "8080";

    std::string proxy_auth;

    //TODO: don't do all this proxy setup on every GET. Just do it once per client, or only when 
    // the proxy information changes.

    //Try to get the proxy settings from the global settings
    if (s_proxySettings.isSet())
    {
        proxy_host = s_proxySettings.get().hostName();
        std::stringstream buf;
        buf << s_proxySettings.get().port();
        proxy_port = buf.str();

        std::string proxy_username = s_proxySettings.get().userName();
        std::string proxy_password = s_proxySettings.get().password();
        if (!proxy_username.empty() && !proxy_password.empty())
        {
            proxy_auth = proxy_username + std::string(":") + proxy_password;
        }
    }

    //Try to get the proxy settings from the local options that are passed in.
    readOptions( options, proxy_host, proxy_port );

    optional< ProxySettings > proxySettings;
    ProxySettings::fromOptions( options, proxySettings );
    if (proxySettings.isSet())
    {       
        proxy_host = proxySettings.get().hostName();
        proxy_port = toString<int>(proxySettings.get().port());
        OE_DEBUG << "Read proxy settings from options " << proxy_host << " " << proxy_port << std::endl;
    }

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
    
        if ( s_HTTP_DEBUG )
            OE_NOTICE << LC << "Using proxy: " << proxy_addr << std::endl;

        //curl_easy_setopt( _curl_handle, CURLOPT_HTTPPROXYTUNNEL, 1 ); 
        curl_easy_setopt( _curl_handle, CURLOPT_PROXY, proxy_addr.c_str() );

        //Setup the proxy authentication if setup
        if (!proxy_auth.empty())
        {
            if ( s_HTTP_DEBUG )
                OE_NOTICE << LC << "Using proxy authentication " << proxy_auth << std::endl;

            curl_easy_setopt( _curl_handle, CURLOPT_PROXYUSERPWD, proxy_auth.c_str());
        }
    }
    else
    {
        OE_DEBUG << "Removing proxy settings" << std::endl;
        curl_easy_setopt( _curl_handle, CURLOPT_PROXY, 0 );
    }

    std::string url = request.getURL();
    // Rewrite the url if the url rewriter is available  
    osg::ref_ptr< URLRewriter > rewriter = getURLRewriter();
    if ( rewriter.valid() )
    {
        std::string oldURL = url;
        url = rewriter->rewrite( oldURL );
        OE_INFO << "Rewrote URL " << oldURL << " to " << url << std::endl;
    }

    const osgDB::AuthenticationDetails* details = authenticationMap ?
        authenticationMap->getAuthenticationDetails( url ) :
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
    curl_easy_setopt( _curl_handle, CURLOPT_URL, url.c_str() );
    if (callback)
    {
        curl_easy_setopt(_curl_handle, CURLOPT_PROGRESSDATA, progressCallback.get());
    }

    CURLcode res;
    long response_code = 0L;

    if ( _simResponseCode < 0 )
    {
        char errorBuf[CURL_ERROR_SIZE];
        errorBuf[0] = 0;
        curl_easy_setopt( _curl_handle, CURLOPT_ERRORBUFFER, (void*)errorBuf );

        curl_easy_setopt( _curl_handle, CURLOPT_WRITEDATA, (void*)&sp);
        res = curl_easy_perform( _curl_handle );
        curl_easy_setopt( _curl_handle, CURLOPT_WRITEDATA, (void*)0 );
        curl_easy_setopt( _curl_handle, CURLOPT_PROGRESSDATA, (void*)0);

        //Disable peer certificate verification to allow us to access in https servers where the peer certificate cannot be verified.
        curl_easy_setopt( _curl_handle, CURLOPT_SSL_VERIFYPEER, (void*)0 );

        if (!proxy_addr.empty())
        {
            long connect_code = 0L;
            CURLcode r = curl_easy_getinfo(_curl_handle, CURLINFO_HTTP_CONNECTCODE, &connect_code);
            if ( r != CURLE_OK )
            {
                OE_WARN << LC << "Proxy connect error: " << curl_easy_strerror(r) << std::endl;
                return HTTPResponse(0);
            }
        }

        curl_easy_getinfo( _curl_handle, CURLINFO_RESPONSE_CODE, &response_code );        
    }
    else
    {
        // simulate failure with a custom response code
        response_code = _simResponseCode;
        res = response_code == 408 ? CURLE_OPERATION_TIMEDOUT : CURLE_COULDNT_CONNECT;
    }    

    HTTPResponse response( response_code );
    
    // read the response content type:
    char* content_type_cp;

    curl_easy_getinfo( _curl_handle, CURLINFO_CONTENT_TYPE, &content_type_cp );    

    if ( content_type_cp != NULL )
    {
        response._mimeType = content_type_cp;    
    }            

    if ( s_HTTP_DEBUG )
    {
        TimeStamp filetime = 0;
        if (CURLE_OK != curl_easy_getinfo(_curl_handle, CURLINFO_FILETIME, &filetime))
            filetime = 0;

        OE_NOTICE << LC 
            << "GET(" << response_code << ", " << response._mimeType << ") : \"" 
            << url << "\" (" << DateTime(filetime).asRFC1123() << ")"<< std::endl;
    }

    // upon success, parse the data:
    if ( res != CURLE_ABORTED_BY_CALLBACK && res != CURLE_OPERATION_TIMEDOUT )
    {        
        // check for multipart content
        if (response._mimeType.length() > 9 && 
            ::strstr( response._mimeType.c_str(), "multipart" ) == response._mimeType.c_str() )
        {
            OE_DEBUG << LC << "detected multipart data; decoding..." << std::endl;

            //TODO: parse out the "wcs" -- this is WCS-specific
            decodeMultipartStream( "wcs", part.get(), response._parts );
        }
        else
        {
            // store headers that we care about
            part->_headers[IOMetadata::CONTENT_TYPE] = response._mimeType;
            response._parts.push_back( part.get() );
        }
    }
    else  /*if (res == CURLE_ABORTED_BY_CALLBACK || res == CURLE_OPERATION_TIMEDOUT) */
    {        
        //If we were aborted by a callback, then it was cancelled by a user
        response._cancelled = true;
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
    initialize();

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
        OE_WARN << LC << "Error downloading file " << filename
            << " (" << response.getCode() << ")" << std::endl;
        return false;
    } 
}

namespace
{
    osgDB::ReaderWriter*
    getReader( const std::string& url, const HTTPResponse& response )
    {
        osgDB::ReaderWriter* reader = 0L;

        // try extension first:
        std::string ext = osgDB::getFileExtension( url );
        if ( !ext.empty() )
        {
            reader = osgDB::Registry::instance()->getReaderWriterForExtension( ext );
        }

        if ( !reader )
        {
            // try to look up a reader by mime-type first:
            std::string mimeType = response.getMimeType();
            if ( !mimeType.empty() )
            {
                reader = osgDB::Registry::instance()->getReaderWriterForMimeType(mimeType);
            }
        }

        if ( !reader )
        {
            OE_WARN << LC << "Cannot find an OSG plugin to read response data (ext="
                << ext << "; mime-type=" << response.getMimeType()
                << ")" << std::endl;
        }

        return reader;
    }
}

ReadResult
HTTPClient::doReadImage(const std::string&    location,
                        const osgDB::Options* options,
                        ProgressCallback*     callback)
{
    initialize();

    ReadResult result;

    HTTPResponse response = this->doGet(location, options, callback);

    if (response.isOK())
    {
        osgDB::ReaderWriter* reader = getReader(location, response);
        if (!reader)
        {            
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
        
        // last-modified (file time)
        TimeStamp filetime = 0;
        if ( CURLE_OK == curl_easy_getinfo(_curl_handle, CURLINFO_FILETIME, &filetime) )
        {
            result.setLastModifiedTime( filetime );
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

    // set the source name
    if ( result.getImage() )
        result.getImage()->setName( location );

    return result;
}

ReadResult
HTTPClient::doReadNode(const std::string&    location,
                       const osgDB::Options* options,
                       ProgressCallback*     callback)
{
    initialize();

    ReadResult result;

    HTTPResponse response = this->doGet(location, options, callback);

    if (response.isOK())
    {
        osgDB::ReaderWriter* reader = getReader(location, response);
        if (!reader)
        {
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
        
        // last-modified (file time)
        TimeStamp filetime = 0;
        if ( CURLE_OK == curl_easy_getinfo(_curl_handle, CURLINFO_FILETIME, &filetime) )
        {
            result.setLastModifiedTime( filetime );
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
    initialize();

    ReadResult result;

    HTTPResponse response = this->doGet(location, options, callback);

    if (response.isOK())
    {
        osgDB::ReaderWriter* reader = getReader(location, response);
        if (!reader)
        {
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
        
        // last-modified (file time)
        TimeStamp filetime = 0;
        if ( CURLE_OK == curl_easy_getinfo(_curl_handle, CURLINFO_FILETIME, &filetime) )
        {
            result.setLastModifiedTime( filetime );
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
    initialize();

    ReadResult result;

    HTTPResponse response = this->doGet( location, options, callback );
    if ( response.isOK() )
    {
        result = ReadResult( new StringObject(response.getPartAsString(0)), response.getHeadersAsConfig());
    }

    else if ( response.getCode() >= 400 && response.getCode() < 500 && response.getCode() != 404 )
    {
        // for request errors, return an error result with the part data intact
        // so the user can parse it as needed. We only do this for readString.
        result = ReadResult( 
            ReadResult::RESULT_SERVER_ERROR,
            new StringObject(response.getPartAsString(0)), 
            response.getHeadersAsConfig() );
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

    // last-modified (file time)
    TimeStamp filetime = 0;
    if ( CURLE_OK == curl_easy_getinfo(_curl_handle, CURLINFO_FILETIME, &filetime) )
    {
        result.setLastModifiedTime( filetime );
    }

    return result;
}
