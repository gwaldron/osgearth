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
#include <osgEarth/Config>
#include <osgEarth/XmlUtils>
#include <sstream>

using namespace osgEarth;

static Config emptyConfig;

bool
Config::loadXML( std::istream& in )
{
    osg::ref_ptr<XmlDocument> xml = XmlDocument::load( in );
    if ( xml.valid() )
        *this = xml->toConfig();
    return xml.valid();
}

const Config&
Config::child( const std::string& childName ) const
{
    for( ConfigSet::const_iterator i = _children.begin(); i != _children.end(); i++ ) {
        if ( i->key() == childName )
            return *i;
    }
    return emptyConfig;
}

std::string
Config::toString( int indent ) const
{
    std::stringstream buf;
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
