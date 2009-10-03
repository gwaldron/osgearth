/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <curl/types.h>
#include <osgEarth/HTTPClient>
#include <osgEarth/Registry>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>
#include <osg/Notify>
#include <string.h>
#include <sstream>

using namespace osgEarth;

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
_url( rhs._url ),
_parameters( rhs._parameters )
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
    _parameters[name] = buf.str();
}

void
HTTPRequest::addParameter( const std::string& name, double value )
{
    std::stringstream buf;
    buf << value;
    _parameters[name] = buf.str();
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
        return buf.str();
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

long
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
    return _parts[n]->_stream.str();
}

const std::string&
HTTPResponse::getMimeType() const {
    return _mimeType;
}

/****************************************************************************/

typedef std::map< OpenThreads::Thread*, osg::ref_ptr<HTTPClient> >    ThreadClientMap;        
static OpenThreads::Mutex          _threadClientMapMutex;
static ThreadClientMap             _threadClientMap;

HTTPClient& HTTPClient::getClient()
{
    OpenThreads::ScopedLock<OpenThreads::Mutex>  lock(_threadClientMapMutex);
    static unsigned int numClients = 0;
    osg::ref_ptr<HTTPClient>& client = _threadClientMap[OpenThreads::Thread::CurrentThread()];
    if (!client) 
    {
        client = new HTTPClient();
        numClients++;
    }

    return *client;
}

HTTPClient::HTTPClient( const osgDB::ReaderWriter::Options* options )
{
    _curl_handle = curl_easy_init();
    curl_easy_setopt( _curl_handle, CURLOPT_USERAGENT, "libcurl-agent/1.0" );
    curl_easy_setopt( _curl_handle, CURLOPT_WRITEFUNCTION, osgEarth::StreamObjectReadCallback );
    curl_easy_setopt( _curl_handle, CURLOPT_FOLLOWLOCATION, (void*)1 );
    curl_easy_setopt( _curl_handle, CURLOPT_MAXREDIRS, (void*)5 );
    curl_easy_setopt(_curl_handle, CURLOPT_PROGRESSFUNCTION,&CurlProgressCallback);
    curl_easy_setopt(_curl_handle, CURLOPT_NOPROGRESS,FALSE);

    _proxy_port = "8080";

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
                setProxyHost( opt.substr( index+1 ) );
                osg::notify(osg::INFO) << "[osgEarth::HTTPClient] set proxy host = " << _proxy_host << std::endl;
            }
            else if ( opt.substr( 0, index ) == "OSG_CURL_PROXYPORT" )
            {
                setProxyPort( opt.substr( index+1 ) );
                //setProxyPort( (unsigned short)::atoi( opt.substr( index+1 ).c_str() ) );
                osg::notify(osg::INFO) << "[osgEarth::HTTPClient] set proxy port = " << _proxy_port << std::endl;
            }
        }
    }
}

HTTPClient::~HTTPClient()
{
    if (_curl_handle) curl_easy_cleanup( _curl_handle );
    _curl_handle = 0;
}

void
HTTPClient::setProxyHost( const std::string& host )
{
    _proxy_host = host;
}

void
HTTPClient::setProxyPort( const std::string& port )
{
    _proxy_port = port;
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
    std::string bstr = "--" + boundary;
    std::string line;
    char tempbuf[256];

    // first thing in the stream should be the boundary.
    input->_stream.read( tempbuf, bstr.length() );
    tempbuf[bstr.length()] = 0;
    line = tempbuf;
    if ( line != bstr )
    {
        osg::notify(osg::WARN)
            << "[osgEarth::HTTPClient] HTTPClient.decodeMultipartStream: protocol violation; "
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
HTTPClient::get( const HTTPRequest& request,
                 ProgressCallback* callback)
{
    return getClient().doGet( request, callback );
}

HTTPResponse 
HTTPClient::get( const std::string &url,
                 ProgressCallback* callback)
{
    return getClient().doGet( url, callback);
}

osg::Image*
HTTPClient::readImageFile(const std::string &filename,
                          const osgDB::ReaderWriter::Options *options,
                          osgEarth::ProgressCallback *callback)
{
    return getClient().doReadImageFile( filename, options, callback );
}

HTTPResponse
HTTPClient::doGet( const HTTPRequest& request, ProgressCallback* callback) const
{
    HTTPResponse response(0);

    // Set up proxy server:
    std::string proxy_addr;
    if ( !_proxy_host.empty() )
    {
        std::stringstream buf;
        buf << _proxy_host << ":" << _proxy_port;
        proxy_addr = buf.str();
    
        //osg::notify(osg::INFO) << "[osgEarth::HTTPClient] setting proxy: " << proxy_addr << std::endl;
        curl_easy_setopt( _curl_handle, CURLOPT_PROXY, proxy_addr.c_str() );
    }

    osg::notify(osg::INFO) << "[osgEarth::HTTPClient] GET " << request.getURL() << std::endl;

    osg::ref_ptr<HTTPResponse::Part> part = new HTTPResponse::Part();
    StreamObject sp( &part->_stream );

    //Take a temporary ref to the callback
    osg::ref_ptr<ProgressCallback> progressCallback = callback;
    curl_easy_setopt( _curl_handle, CURLOPT_URL, request.getURL().c_str() );
    if (callback)
    {
        curl_easy_setopt(_curl_handle, CURLOPT_PROGRESSDATA,progressCallback.get());
    }
    curl_easy_setopt( _curl_handle, CURLOPT_WRITEDATA, (void*)&sp);
    CURLcode res = curl_easy_perform( _curl_handle );
    curl_easy_setopt( _curl_handle, CURLOPT_WRITEDATA, (void*)0 );
    curl_easy_setopt( _curl_handle, CURLOPT_PROGRESSDATA, (void*)0);
   
    if ( res == 0 )
    {
        long code;
        if ( !proxy_addr.empty() )
            curl_easy_getinfo( _curl_handle, CURLINFO_HTTP_CONNECTCODE, &code );
        else
            curl_easy_getinfo( _curl_handle, CURLINFO_RESPONSE_CODE, &code );     

        osg::notify(osg::INFO) << "[HTTPClient] got response, code = " << code << std::endl;

        response = HTTPResponse( code );

        // check for multipart content:
        char* content_type_cp;
        curl_easy_getinfo( _curl_handle, CURLINFO_CONTENT_TYPE, &content_type_cp );
        if ( content_type_cp == NULL )
        {
            osg::notify(osg::NOTICE) << "[osgEarth::HTTPClient] NULL Content-Type (protocol violation)" << std::endl;
            return NULL;
        }

        // NOTE:
        //   WCS 1.1 specified a "multipart/mixed" response, but ArcGIS Server gives a "multipart/related"
        //   content type ...

        std::string content_type( content_type_cp );
        //osg::notify(osg::NOTICE) << "[osgEarth.HTTPClient] content-type = \"" << content_type << "\"" << std::endl;
        if ( content_type.length() > 9 && ::strstr( content_type.c_str(), "multipart" ) == content_type.c_str() )
        //if ( content_type == "multipart/mixed; boundary=wcs" ) //todo: parse this.
        {
            //osg::notify(osg::NOTICE) << "[osgEarth.HTTPClient] detected multipart data; decoding..." << std::endl;
            //TODO: parse out the "wcs" -- this is WCS-specific
            decodeMultipartStream( "wcs", part.get(), response._parts );
        }
        else
        {
            //osg::notify(osg::NOTICE) << "[osgEarth.HTTPClient] detected single part data" << std::endl;
            response._parts.push_back( part.get() );
        }
    }
    else if (res == CURLE_ABORTED_BY_CALLBACK)
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
HTTPClient::doGet( const std::string& url, ProgressCallback* callback) const
{
    return doGet( HTTPRequest( url ), callback );
}

bool
HTTPClient::downloadFile(const std::string &url, const std::string &filename)
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
        osg::notify(osg::WARN) << "[osgEarth::HTTPClient] Error downloading file " << filename << std::endl;
        return false;
    } 
}

osg::Image*
HTTPClient::doReadImageFile(const std::string &filename,
                            const osgDB::ReaderWriter::Options *options,
                            osgEarth::ProgressCallback *callback)
{
    HTTPResponse response = this->doGet(filename, callback);

    if (response.isOK())
    {
        // Try to find a reader by file extension. If this fails, we will fetch the file
        // anyway and try to get a reader via mime-type.
        std::string ext = osgDB::getFileExtension( filename );
        osgDB::ReaderWriter *reader =
            osgDB::Registry::instance()->getReaderWriterForExtension( ext );
        //osg::notify(osg::NOTICE) << "Reading " << filename << " with mime " << response.getMimeType() << std::endl;

        //If we didn't get a reader by extension, try to get it via mime type
        if (!reader)
        {
            std::string mimeType = response.getMimeType();
            osg::notify(osg::INFO) << "HTTPClient: Looking up extension for mime-type " << mimeType << std::endl;
            if ( mimeType.length() > 0 )
            {
                reader = osgEarth::Registry::instance()->getReaderWriterForMimeType(mimeType);
            }
        }

        if (!reader)
        {
            osg::notify(osg::NOTICE)<<"Error: No ReaderWriter for file "<<filename<<std::endl;
            return NULL;
        }

        if (reader)
        {
            osgDB::ReaderWriter::ReadResult rr = reader->readImage(response.getPartStream(0), options);
            if (rr.validImage()) return rr.takeImage();
            if (rr.error()) osg::notify(osg::WARN) << rr.message() << std::endl;
            return NULL;
        }
    }
    else
    {
        /*if (response.isCancelled())
            osg::notify(osg::NOTICE) << "Request for " << filename << " was cancelled " << std::endl;*/
    }
    return 0;
}
