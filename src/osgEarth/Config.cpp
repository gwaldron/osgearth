/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
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

#define LC "[Config] "

Config::~Config()
{
}

void
Config::setReferrer( const std::string& referrer )
{
    if ( referrer.empty() )
        return;

    std::string absReferrer;
    if( !osgDB::containsServerAddress( referrer ) ) {
        absReferrer = osgEarth::getAbsolutePath( referrer );

        if( osgEarth::isRelativePath( absReferrer ) )
        {
            OE_WARN << LC << "ILLEGAL: call to setReferrer with relative path:  "
                "key=" << key() << "; referrer=" << referrer << "\n";
            return;
        }
    }
    else {
        absReferrer = referrer;
    }

    // Don't overwrite an existing referrer:
    if ( _referrer.empty() )
    {
        _referrer = absReferrer;
    }

    for( ConfigSet::iterator i = _children.begin(); i != _children.end(); i++ )
    { 
        i->setReferrer( absReferrer );
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
    Json::Value conf2json(const Config& conf, bool nicer, int depth)
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
                            {
                                value[i->first] = c.value();
                            }
                            else
                            {
                                Json::Value child = conf2json(c, nicer, depth+1);
                                if (child.isObject())
                                {
                                    value[i->first] = child;
                                }   
                            }
                        }
                        else
                        {
                            std::string array_key = Stringify() << i->first << "__array__";
                            Json::Value array_value( Json::arrayValue );
                            for( std::vector<Config>::iterator j = i->second.begin(); j != i->second.end(); ++j )
                            {
                                array_value.append( conf2json(*j, nicer, depth+1) );
                            }
                            value[array_key] = array_value;
                            //value = array_value;
                        }
                    }
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
                            children[i++] = conf2json(*c, nicer, depth+1);
                    }

                    if ( !children.empty() )
                    {
                        value["$children"] = children;
                    }
                }
            }

            // At the root, embed the Config in a single JSON object.
            if ( depth == 0 )
            {
                Json::Value root;
                root[conf.key()] = value;
                value = root;
            }
        }

        return value;
    }

    void json2conf(const Json::Value& json, Config& conf, int depth)
    {
        if ( json.type() == Json::objectValue )
        {
            Json::Value::Members members = json.getMemberNames();

            for( Json::Value::Members::const_iterator i = members.begin(); i != members.end(); ++i )
            {
                const Json::Value& value = json[*i];

                if ( value.isObject() )
                {
                    if (depth == 0 && members.size() == 1)
                    {
                        conf.key() = *i;
                        json2conf(value, conf, depth+1);
                    }
                    else
                    {
                        Config element( *i );
                        json2conf( value, element, depth+1 );
                        conf.add( element );
                    }
                }
                else if ( value.isArray() )
                {
                    if ( endsWith(*i, "__array__") )
                    {
                        std::string key = i->substr(0, i->length()-9);
                        for( Json::Value::const_iterator j = value.begin(); j != value.end(); ++j )
                        {
                            Config child;
                            json2conf( *j, child, depth+1 );
                            conf.add( key, child );
                        }
                    }
                    else if ( endsWith(*i, "_$set") ) // backwards compatibility
                    {
                        std::string key = i->substr(0, i->length()-5);
                        for( Json::Value::const_iterator j = value.begin(); j != value.end(); ++j )
                        {
                            Config child;
                            json2conf( *j, child, depth+1 );
                            conf.add( key, child );
                        }
                    }
                    else
                    {
                        Config element( *i );
                        json2conf( value, element, depth+1 );
                        conf.add( element );
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
                    json2conf( value, conf, depth+1 );
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
                json2conf( *j, child, depth+1 );
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
    Json::Value root = conf2json( *this, true, 0 );
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
        json2conf( root, *this, 0 );
        return true;
    }
    else
    {
        OE_WARN 
            << "JSON decoding error: "
            << reader.getFormatedErrorMessages() 
            << std::endl;
    }
    return false;
}

Config
Config::readJSON(const std::string& json)
{
    Config conf;
    conf.fromJSON(json);
    return conf;
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
