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
#include <osgEarth/HTTPClient>
#include <osgDB/FileNameUtils>
#include <fstream>
#include <sstream>

using namespace osgEarth;

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

URI::URI()
{
    //nop
}

URI::URI( const std::string& location, const URIContext& context )
{
    _baseURI = location;
    _fullURI = osgEarth::getFullPath( context, location );
    _context = context;
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

osg::Image*
URI::readImage( ResultCode* code, const osgDB::Options* options ) const
{
    if ( empty() ) {
        if ( code ) *code = RESULT_NOT_FOUND;
        return 0L;
    }
    osg::ref_ptr<osg::Image> image;
    ResultCode result = (ResultCode)HTTPClient::readImageFile( _fullURI, image, options );
    if ( code ) *code = result;
    return image.release();
}

osg::Node*
URI::readNode( ResultCode* code, const osgDB::Options* options ) const
{
    if ( empty() ) {
        if ( code ) *code = RESULT_NOT_FOUND;
        return 0L;
    }
    osg::ref_ptr<osg::Node> node;
    ResultCode result = (ResultCode)HTTPClient::readNodeFile( _fullURI, node, options );
    if ( code ) *code = result;
    return node.release();
}

std::string
URI::readString( ResultCode* code ) const
{
    if ( empty() ) {
        if ( code ) *code = RESULT_NOT_FOUND;
        return 0L;
    }
    std::string str;
    ResultCode result = (ResultCode)HTTPClient::readString( _fullURI, str );
    if ( code ) *code = result;
    return str;
}

