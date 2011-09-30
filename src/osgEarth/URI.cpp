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

osg::Object*
URI::readObject( const osgDB::Options* options, ResultCode* out_code ) const
{
    if ( empty() ) {
        if ( out_code ) *out_code = RESULT_NOT_FOUND;
        return 0L;
    }

    osg::ref_ptr<const osgDB::Options> myOptions = fixOptions(options);

    osg::ref_ptr<osg::Object> object;
    ResultCode result = (ResultCode)HTTPClient::readObjectFile( _fullURI, object, myOptions.get() );
    if ( out_code ) *out_code = result;

    return object.release();
}

osg::Image*
URI::readImage( const osgDB::Options* options, ResultCode* out_code ) const
{
    if ( empty() ) {
        if ( out_code ) *out_code = RESULT_NOT_FOUND;
        return 0L;
    }

    osg::ref_ptr<const osgDB::Options> myOptions = fixOptions(options);

    //OE_INFO << LC << "readImage: " << _fullURI << std::endl;

    osg::ref_ptr<osg::Image> image;
    ResultCode result = (ResultCode)HTTPClient::readImageFile( _fullURI, image, myOptions.get() );
    if ( out_code ) *out_code = result;

    return image.release();
}

osg::Node*
URI::readNode( const osgDB::Options* options, ResultCode* out_code ) const
{
    if ( empty() ) {
        if ( out_code ) *out_code = RESULT_NOT_FOUND;
        return 0L;
    }

    osg::ref_ptr<const osgDB::Options> myOptions = fixOptions(options);

    osg::ref_ptr<osg::Node> node;
    ResultCode result = (ResultCode)HTTPClient::readNodeFile( _fullURI, node, myOptions.get() );
    if ( out_code ) *out_code = result;
    return node.release();
}

std::string
URI::readString( ResultCode* out_code ) const
{
    if ( empty() ) {
        if ( out_code ) *out_code = RESULT_NOT_FOUND;
        return 0L;
    }

    std::string str;
    ResultCode result = (ResultCode)HTTPClient::readString( _fullURI, str );
    if ( out_code ) *out_code = result;
    return str;
}
