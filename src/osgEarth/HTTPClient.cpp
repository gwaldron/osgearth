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
#include <fstream>
#include <iterator>
#include <iostream>
#include <algorithm>

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
	std::string streamStr;
	streamStr = _parts[n]->_stream.str();
    return streamStr;
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

HTTPClient::HTTPClient()
{
    _curl_handle = curl_easy_init();
    curl_easy_setopt( _curl_handle, CURLOPT_USERAGENT, "libcurl-agent/1.0" );
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
HTTPClient::readOptions( const osgDB::ReaderWriter::Options* options, std::string& proxy_host, std::string& proxy_port) const
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
                 const osgDB::ReaderWriter::Options* options,
                 ProgressCallback* callback)
{
    return getClient().doGet( request, options, callback );
}

HTTPResponse 
HTTPClient::get( const std::string &url,
                 const osgDB::ReaderWriter::Options* options,
                 ProgressCallback* callback)
{
    return getClient().doGet( url, options, callback);
}

HTTPClient::ResultCode
HTTPClient::readImageFile(const std::string &filename,
                          osg::ref_ptr<osg::Image>& output,
                          const osgDB::ReaderWriter::Options *options,
                          osgEarth::ProgressCallback *callback)
{
    return getClient().doReadImageFile( filename, output, options, callback );
}

HTTPClient::ResultCode
HTTPClient::readNodeFile(const std::string& filename,
                         osg::ref_ptr<osg::Node>& output,
                         const osgDB::ReaderWriter::Options *options,
                         osgEarth::ProgressCallback *callback)
{
    return getClient().doReadNodeFile( filename, output, options, callback );
}

HTTPClient::ResultCode
HTTPClient::readString(const std::string& filename,
                       std::string& output,
                       osgEarth::ProgressCallback* callback)
{
    return getClient().doReadString( filename, output, callback );
}

HTTPResponse
HTTPClient::doGet( const HTTPRequest& request, const osgDB::ReaderWriter::Options* options, ProgressCallback* callback) const
{
    std::string proxy_host;
    std::string proxy_port = "8080";
    readOptions( options, proxy_host, proxy_port );

    // Set up proxy server:
    std::string proxy_addr;
    if ( !proxy_host.empty() )
    {
        std::stringstream buf;
        buf << proxy_host << ":" << proxy_port;
		std::string bufStr;
		bufStr = buf.str();
        proxy_addr = bufStr;
    
        osg::notify(osg::INFO) << "[osgEarth::HTTPClient] setting proxy: " << proxy_addr << std::endl;
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
        curl_easy_setopt(_curl_handle, CURLOPT_PROGRESSDATA, progressCallback.get());
    }

    char errorBuf[CURL_ERROR_SIZE];
    errorBuf[0] = 0;
    curl_easy_setopt( _curl_handle, CURLOPT_ERRORBUFFER, (void*)errorBuf );

    curl_easy_setopt( _curl_handle, CURLOPT_WRITEDATA, (void*)&sp);
    CURLcode res = curl_easy_perform( _curl_handle );
    curl_easy_setopt( _curl_handle, CURLOPT_WRITEDATA, (void*)0 );
    curl_easy_setopt( _curl_handle, CURLOPT_PROGRESSDATA, (void*)0);

    long code = 0L;
    if ( !proxy_addr.empty() )
        curl_easy_getinfo( _curl_handle, CURLINFO_HTTP_CONNECTCODE, &code );
    else
        curl_easy_getinfo( _curl_handle, CURLINFO_RESPONSE_CODE, &code );     

    osg::notify(osg::INFO) << "[HTTPClient] got response, code = " << code << std::endl;

    HTTPResponse response( code );
   
    if ( code == 200L && res != CURLE_ABORTED_BY_CALLBACK && res != CURLE_OPERATION_TIMEDOUT ) //res == 0 )
    {
        // check for multipart content:
        char* content_type_cp;
        curl_easy_getinfo( _curl_handle, CURLINFO_CONTENT_TYPE, &content_type_cp );
        if ( content_type_cp == NULL )
        {
            osg::notify(osg::NOTICE) 
                << "[osgEarth::HTTPClient] NULL Content-Type (protocol violation) " 
                << "URL=" << request.getURL() << std::endl;
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
    else if (res == CURLE_ABORTED_BY_CALLBACK || res == CURLE_OPERATION_TIMEDOUT)
    {
        //If we were aborted by a callback, then it was cancelled by a user
        response._cancelled = true;
    }
    else
    {        
        //if ( callback )
        //{
        //    if ( errorBuf[0] ) {
        //        callback->message() = errorBuf;
        //    }
        //    else {
        //        std::stringstream buf;
        //        buf << "HTTP Code " << response.getCode();
        //        callback->message() = buf.str();
        //    }
        //}
        //else {
        //    osg::notify(osg::NOTICE) << "[osgEarth] [HTTP] error, code = " << code << std::endl;
        //}
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
HTTPClient::doGet( const std::string& url, const osgDB::ReaderWriter::Options* options, ProgressCallback* callback) const
{
    return doGet( HTTPRequest( url ), options, callback );
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

HTTPClient::ResultCode
HTTPClient::doReadImageFile(const std::string& filename, 
                            osg::ref_ptr<osg::Image>& output,
                            const osgDB::ReaderWriter::Options *options,
                            osgEarth::ProgressCallback *callback)
{
    ResultCode result = RESULT_OK;

    if ( osgDB::containsServerAddress( filename ) )
    {
        HTTPResponse response = this->doGet(filename, options, callback);

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
                result = RESULT_NO_READER;
            }

            else 
            {
                osgDB::ReaderWriter::ReadResult rr = reader->readImage(response.getPartStream(0), options);
                if ( rr.validImage() )
                {
                    output = rr.takeImage();
                }
                else 
                {
                    if (rr.error()) 
                    {
                        osg::notify(osg::WARN) << "[osgEarth] HTTP Reader Error: " << rr.message() << std::endl;
                    }
                    result = RESULT_READER_ERROR;
                }
            }
        }
        else
        {
            result =
                response.isCancelled() ? RESULT_CANCELED :
                response.getCode() == HTTPResponse::NOT_FOUND ? RESULT_NOT_FOUND :
                response.getCode() == HTTPResponse::SERVER_ERROR ? RESULT_SERVER_ERROR :
                RESULT_UNKNOWN_ERROR;

            //if ( response.isCancelled() )
            //    osg::notify(osg::NOTICE) << "[osgEarth] HTTP cancel: " << filename << std::endl;
            //else
            //    osg::notify(osg::NOTICE) << "[osgEarth] HTTP ERROR " << response.getCode() << ": " << filename << std::endl;

            /*if (response.isCancelled())
                osg::notify(osg::NOTICE) << "Request for " << filename << " was cancelled " << std::endl;*/
        }
    }
    else
    {
        output = osgDB::readImageFile( filename, options );
        if ( !output.valid() )
            result = RESULT_NOT_FOUND;
    }

    return result;
}

HTTPClient::ResultCode
HTTPClient::doReadNodeFile(const std::string& filename,
                           osg::ref_ptr<osg::Node>& output,
                           const osgDB::ReaderWriter::Options *options,
                           osgEarth::ProgressCallback *callback)
{
    ResultCode result = RESULT_OK;

    if ( osgDB::containsServerAddress( filename ) )
    {
        HTTPResponse response = this->doGet(filename, options, callback);
        if (response.isOK())
        {
            // Try to find a reader by file extension. If this fails, we will fetch the file
            // anyway and try to get a reader via mime-type.
            std::string ext = osgDB::getFileExtension( filename );
            osgDB::ReaderWriter *reader =
                osgDB::Registry::instance()->getReaderWriterForExtension( ext );

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

            // if we still didn't get it, bad news
            if (!reader)
            {
                osg::notify(osg::NOTICE)<<"Error: No ReaderWriter for file "<<filename<<std::endl;
                result = RESULT_NO_READER;
            }

            else
            {
                osgDB::ReaderWriter::ReadResult rr = reader->readNode(response.getPartStream(0), options);
                if ( rr.validNode() )
                {
                    output = rr.takeNode();
                }
                else
                {
                    if ( rr.error() )
                    {
                        osg::notify(osg::WARN) << "[osgEarth] HTTP Reader Error: " << rr.message() << std::endl;
                    }
                    result = RESULT_READER_ERROR;
                }
            }
        }
        else
        {
            result =
                response.isCancelled() ? RESULT_CANCELED :
                response.getCode() == HTTPResponse::NOT_FOUND ? RESULT_NOT_FOUND :
                response.getCode() == HTTPResponse::SERVER_ERROR ? RESULT_SERVER_ERROR :
                RESULT_UNKNOWN_ERROR;
               
            /*if (response.isCancelled())
                osg::notify(osg::NOTICE) << "Request for " << filename << " was cancelled " << std::endl;*/
        }
    }
    else
    {
        output = osgDB::readNodeFile( filename, options );
        if ( !output.valid() )
            result = RESULT_NOT_FOUND;
    }

    return result;
}


HTTPClient::ResultCode 
HTTPClient::doReadString(const std::string& filename,
                         std::string& output,
                         osgEarth::ProgressCallback* callback )
{
    ResultCode result = RESULT_OK;

    if ( osgDB::containsServerAddress( filename ) )
    {
        HTTPResponse response = this->doGet( filename, NULL, callback );
        if ( response.isOK() )
        {
            output = response.getPartAsString( 0 );
        }
        else
        {
            result =
                response.isCancelled() ? RESULT_CANCELED :
                response.getCode() == HTTPResponse::NOT_FOUND ? RESULT_NOT_FOUND :
                response.getCode() == HTTPResponse::SERVER_ERROR ? RESULT_SERVER_ERROR :
                RESULT_UNKNOWN_ERROR;
        }
    }
    else
    {
        std::ifstream input( filename.c_str() );
        if ( input.is_open() )
        {
            input >> std::noskipws;
            std::stringstream buf;
            buf << input.rdbuf();
			std::string bufStr;
		    bufStr = buf.str();
            output = bufStr;
        }
        else
        {
            result = RESULT_NOT_FOUND;
        }
    }

    return result;
}
