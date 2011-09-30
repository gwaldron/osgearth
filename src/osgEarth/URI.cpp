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
#include <osgDB/ReadFile>
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

void
URIContext::toOsgPath( const std::string& relative, std::string& out ) const
{

    out = osgEarth::getFullPath( _referrer, relative );
    if ( !_archive.empty() )
        out = out + "|" + _archive;
}

URIContext
URIContext::parent( const URIContext& sub ) const
{
    std::string newArchive;
    if ( sub._archive.empty() && !_archive.empty() )
        newArchive = _archive;
    else if ( _archive.empty() && !sub._archive.empty() )
        newArchive = sub._archive;
    else if ( !_archive.empty() && !sub._archive.empty() )
        newArchive = sub._archive + "|" + _archive;

    return URIContext(
        osgEarth::getFullPath( _referrer, sub._referrer ),
        newArchive );
}

void
URIContext::store( osgDB::Options* options )
{
    if ( options )
    {
        std::string encodedString = _archive.empty() ? _referrer : _referrer + "|" + _archive;
        options->setPluginStringData( "osgEarth::URIContext", encodedString );
    }
}

URIContext::URIContext( const osgDB::Options* options )
{
    if ( options )
    {
        std::string str = options->getPluginStringData( "osgEarth::URIContext" );
        if ( !str.empty() )
        {
            int pos = str.find('|');
            if ( pos == std::string::npos ) {
                _referrer = str;
            }
            else {
                _referrer = str.substr( 0, pos );
                _archive = str.substr( pos + 1 );
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
    context.toOsgPath( location, _fullURI );
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
