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
#include <osgEarth/Config>
#include <osgEarth/XmlUtils>
#include <osgEarth/JsonUtils>
#include <osgEarth/FileUtils>
#include <osgDB/ReaderWriter>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <sstream>
#include <fstream>
#include <iomanip>

using namespace osgEarth;

Config::~Config()
{
}

void
Config::setReferrer( const std::string& referrer )
{
    _referrer = referrer;
    for( ConfigSet::iterator i = _children.begin(); i != _children.end(); i++ )
    { 
        i->setReferrer( osgEarth::getFullPath(_referrer, i->_referrer) );
    }
}

void
Config::inheritReferrer( const std::string& referrer )
{
    if ( _referrer.empty() || !osgEarth::isRelativePath(referrer) )
    {
        setReferrer( referrer );
    }
    else if ( !referrer.empty() )
    {
        setReferrer( osgDB::concatPaths(_referrer, referrer) );
    }
}

bool
Config::fromXML( std::istream& in )
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

const Config*
Config::child_ptr( const std::string& childName ) const
{
    for( ConfigSet::const_iterator i = _children.begin(); i != _children.end(); i++ ) {
        if ( i->key() == childName )
            return &(*i);
    }
    return 0L;
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
    // remove any matching keys first; this will allow the addition of multi-key values
    for( ConfigSet::const_iterator c = rhs._children.begin(); c != rhs._children.end(); ++c )
        remove( c->key() );

    // add in the new values.
    for( ConfigSet::const_iterator c = rhs._children.begin(); c != rhs._children.end(); ++c )
        add( *c );
}

const Config*
Config::find( const std::string& key, bool checkMe ) const
{
    if ( checkMe && key == this->key() )
        return this;

    for( ConfigSet::const_iterator c = _children.begin(); c != _children.end(); ++c )
        if ( key == c->key() )
            return &(*c);

    for( ConfigSet::const_iterator c = _children.begin(); c != _children.end(); ++c )
    {
        const Config* r = c->find(key, false);
        if ( r ) return r;
    }

    return 0L;
}

Config*
Config::find( const std::string& key, bool checkMe )
{
    if ( checkMe && key == this->key() )
        return this;

    for( ConfigSet::iterator c = _children.begin(); c != _children.end(); ++c )
        if ( key == c->key() )
            return &(*c);

    for( ConfigSet::iterator c = _children.begin(); c != _children.end(); ++c )
    {
        Config* r = c->find(key, false);
        if ( r ) return r;
    }

    return 0L;
}

/****************************************************************/
ConfigOptions::~ConfigOptions()
{
}

/****************************************************************/
DriverConfigOptions::~DriverConfigOptions()
{
}

namespace
{
    // Converts a Config to JSON. The "nicer" flag formats the data in a more 
    // readable way than nicer=false. Nicer=true attempts to create JSON "objects",
    // whereas nicer=false makes "$key" and "$children" members.
    Json::Value conf2json( const Config& conf, bool nicer )
    {
        Json::Value value( Json::objectValue );

        if ( conf.isSimple() )
        {
            value[ conf.key() ] = conf.value();
        }
        else
        {
            if ( !nicer )
            {
                if ( !conf.key().empty() )
                {
                    value["$key"] = conf.key();
                }
            }

            if ( !conf.value().empty() )
            {
                value["$value"] = conf.value();
            }

            if ( conf.children().size() > 0 )
            {
                if ( nicer )
                {
                    std::map< std::string, std::vector<Config> > sets;

                    // sort into bins by name:
                    for( ConfigSet::const_iterator c = conf.children().begin(); c != conf.children().end(); ++c )
                    {
                        sets[c->key()].push_back( *c );
                    }

                    for( std::map<std::string,std::vector<Config> >::iterator i = sets.begin(); i != sets.end(); ++i )
                    {
                        if ( i->second.size() == 1 )
                        {
                            Config& c = i->second[0];
                            if ( c.isSimple() )
                                value[i->first] = c.value();
                            else
                                value[i->first] = conf2json(c, nicer);
                        }
                        else
                        {
                            std::string array_key = Stringify() << i->first << "_$set";
                            Json::Value array_value( Json::arrayValue );
                            for( std::vector<Config>::iterator j = i->second.begin(); j != i->second.end(); ++j )
                            {
                                array_value.append( conf2json(*j, nicer) );
                            }
                            value[array_key] = array_value;
                        }
                    }

#if 0
                    bool hasdupes = false;
                    std::set<std::string> dupes;
                    for( ConfigSet::const_iterator c = conf.children().begin(); c != conf.children().end(); ++c ) {
                        if ( dupes.find( c->key() ) != dupes.end() ) {
                            hasdupes = true;
                            break;
                        }
                        else {
                            dupes.insert(c->key());
                        }
                    }

                    for( ConfigSet::const_iterator c = conf.children().begin(); c != conf.children().end(); ++c )
                    {
                        if ( hasdupes )
                            children[i++] = conf2json(*c, nicer);
                        else if ( c->isSimple() )
                            value[c->key()] = c->value();
                        else
                            value[c->key()] = conf2json(*c, nicer);
                    }
#endif
                }
                else
                {
                    Json::Value children( Json::arrayValue );
                    unsigned i = 0;

                    for( ConfigSet::const_iterator c = conf.children().begin(); c != conf.children().end(); ++c )
                    {
                        if ( c->isSimple() )
                            value[c->key()] = c->value();
                        else
                            children[i++] = conf2json(*c, nicer);
                    }

                    if ( !children.empty() )
                    {
                        value["$children"] = children;
                    }
                }
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

                if ( value.isObject() )
                {
                    Config element( *i );
                    json2conf( value, element );
                    conf.add( element );
                }
                else if ( value.isArray() && endsWith(*i, "_$set") )
                {
                    std::string key = i->substr(0, i->length()-5);
                    for( Json::Value::const_iterator j = value.begin(); j != value.end(); ++j )
                    {
                        Config child( key );
                        json2conf( *j, child );
                        conf.add( child );
                    }
                }
                else if ( (*i) == "$key" )
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
                else if ( value.isArray() )
                {
                    Config element( *i );
                    json2conf( value, element );
                    conf.add( element );
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
            conf.value() = json.asString();
        }
    }
}

std::string
Config::toJSON( bool pretty ) const
{
    Json::Value root = conf2json( *this, pretty );
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

Config
Config::operator - ( const Config& rhs ) const
{
    Config result( *this );

    for( ConfigSet::const_iterator i = rhs.children().begin(); i != rhs.children().end(); ++i )
    {
        result.remove( i->key() );
    }

    return result;
}
