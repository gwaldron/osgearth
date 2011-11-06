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
#include <osgEarth/JsonUtils>
#include <sstream>
#include <iomanip>

using namespace osgEarth;

void
Config::setReferrer( const std::string& referrer )
{
    _referrer = referrer;
    for( ConfigSet::iterator i = _children.begin(); i != _children.end(); i++ )
    { 
        i->setReferrer( osgEarth::getFullPath(_referrer, i->_referrer) );
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

Config
Config::child( const std::string& childName ) const
{
    for( ConfigSet::const_iterator i = _children.begin(); i != _children.end(); i++ ) {
        if ( i->key() == childName )
            return *i;
    }

    Config emptyConf;
    emptyConf.setReferrer( _referrer );
    return emptyConf;
}

Config*
Config::mutable_child( const std::string& childName )
{
    for( ConfigSet::iterator i = _children.begin(); i != _children.end(); i++ ) {
        if ( i->key() == childName )
            return &(*i);
    }

    return 0L;
}

void
Config::merge( const Config& rhs ) 
{
    for( ConfigSet::const_iterator c = rhs._children.begin(); c != rhs._children.end(); ++c )
        addChild( *c );
}

namespace
{
    Json::Value conf2json( const Config& conf )
    {
        Json::Value value( Json::objectValue );

        if ( conf.isSimple() )
        {
            value[ conf.key() ] = conf.value();
        }
        else
        {
            if ( !conf.key().empty() )
                value["$key"] = conf.key();

            if ( !conf.value().empty() )
                value["$value"] = conf.value();

            if ( conf.children().size() > 0 )
            {
                Json::Value children( Json::arrayValue );
                unsigned i = 0;
                for( ConfigSet::const_iterator c = conf.children().begin(); c != conf.children().end(); ++c )
                {
                    if ( c->isSimple() )
                        value[c->key()] = c->value();
                    else
                        children[i++] = conf2json( *c );
                }

                if ( !children.empty() )
                    value["$children"] = children;
            }
        }

        return value;
    }

    void json2conf( const Json::Value& json, Config& conf )
    {
        if ( json.type() == Json::objectValue )
        {
            Json::Value::Members members = json.getMemberNames();

            if ( members.size() == 1 )
            {
                const Json::Value& value = json[members[0]];
                if ( value.type() != Json::nullValue && value.type() != Json::objectValue && value.type() != Json::arrayValue )
                {
                    conf.key() = members[0];
                    conf.value() = value.asString();
                    return;
                }
            }

            for( Json::Value::Members::const_iterator i = members.begin(); i != members.end(); ++i )
            {
                const Json::Value& value = json[*i];
                if ( (*i) == "$key" )
                {
                    conf.key() = value.asString();
                }
                else if ( (*i) == "$value" )
                {
                    conf.value() = value.asString();
                }
                else if ( (*i) == "$children" && value.isArray() )
                {
                    json2conf( value, conf );
                }
                else
                {
                    conf.add( *i, value.asString() );
                }
            }
        }
        else if ( json.type() == Json::arrayValue )
        {          
            for( Json::Value::const_iterator j = json.begin(); j != json.end(); ++j )
            {
                Config child;
                json2conf( *j, child );
                if ( !child.empty() )
                    conf.add( child );
            }
        }
        else if ( json.type() != Json::nullValue )
        {
            //conf.value() = json.asString();
        }
    }
}

std::string
Config::toJSON( bool pretty ) const
{
    Json::Value root = conf2json( *this );
    if ( pretty )
        return Json::StyledWriter().write( root );
    else
        return Json::FastWriter().write( root );
}

bool
Config::fromJSON( const std::string& input )
{
    Json::Reader reader;
    Json::Value root( Json::objectValue );
    if ( reader.parse( input, root ) )
    {
        json2conf( root, *this );
        return true;
    }
    return false;
}
