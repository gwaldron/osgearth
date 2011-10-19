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
#include <osgEarth/Config>
#include <osgEarth/XmlUtils>
#include <sstream>
#include <iomanip>

using namespace osgEarth;

Config& emptyConfig()
{
    static Config _emptyConfig;
    return _emptyConfig;
}

void
Config::setURIContext( const URIContext& context )
{
    _uriContext = context;
    for( ConfigSet::iterator i = _children.begin(); i != _children.end(); i++ )
    { 
        i->setURIContext( context.add(i->_uriContext) );
        //URI newURI( i->uriContext(), context );
        //i->setURIContext( *newURI );
    }
}

bool
Config::loadXML( std::istream& in )
{
    osg::ref_ptr<XmlDocument> xml = XmlDocument::load( in );
    if ( xml.valid() )
        *this = xml->getConfig();
    return xml.valid();
}

const Config&
Config::child( const std::string& childName ) const
{
    for( ConfigSet::const_iterator i = _children.begin(); i != _children.end(); i++ ) {
        if ( i->key() == childName )
            return *i;
    }
    return emptyConfig();
}

void
Config::merge( const Config& rhs ) 
{
    for( Properties::const_iterator a = rhs._attrs.begin(); a != rhs._attrs.end(); ++a )
        _attrs[ a->first ] = a->second;

    for( ConfigSet::const_iterator c = rhs._children.begin(); c != rhs._children.end(); ++c )
        addChild( *c );
}

std::string
Config::toString( int indent ) const
{
    std::stringstream buf;
    buf << std::fixed;
    for( int i=0; i<indent; i++ ) buf << "  ";
    buf << "{ " << (_key.empty()? "anonymous" : _key) << ": ";
    if ( !_defaultValue.empty() ) buf << _defaultValue;
    if ( !_attrs.empty() ) {
        buf << std::endl;
        for( int i=0; i<indent+1; i++ ) buf << "  ";
        buf << "attrs: [ ";
        for( Properties::const_iterator a = _attrs.begin(); a != _attrs.end(); a++ )
            buf << a->first << "=" << a->second << ", ";
        buf << " ]";
    }
    if ( !_children.empty() ) {
        for( ConfigSet::const_iterator c = _children.begin(); c != _children.end(); c++ )
            buf << std::endl << (*c).toString( indent+1 );
    }

    buf << " }";

	std::string bufStr;
	bufStr = buf.str();
    return bufStr;
}

std::string
Config::toHashString() const
{
    std::stringstream buf;
    buf << std::fixed;
    buf << "{" << (_key.empty()? "anonymous" : _key) << ":";
    if ( !_defaultValue.empty() ) buf << _defaultValue;
    if ( !_attrs.empty() ) {
        buf << "[";
        for( Properties::const_iterator a = _attrs.begin(); a != _attrs.end(); a++ )
            buf << a->first << "=" << a->second << ",";
        buf << "]";
    }
    if ( !_children.empty() ) {
        for( ConfigSet::const_iterator c = _children.begin(); c != _children.end(); c++ )
            buf << (*c).toHashString();
    }

    buf << "}";

	std::string bufStr;
	bufStr = buf.str();
    return bufStr;
}
