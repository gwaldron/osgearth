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

#include <osgEarth/MapConfig>
#include <osgEarth/XmlUtils>
#include <osgEarth/HTTPClient>

#include <osg/Notify>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <fstream>

using namespace osgEarth;

MapConfig::MapConfig()
{
    model_cstype = MapConfig::CSTYPE_GEOCENTRIC;
    vertical_scale = 1.0f;
    skirt_ratio = 0.02;
    proxy_port = 8080;
    min_tile_range_factor = 8.0;
    north_cap_color = osg::Vec4ub(2,5,20,255);
    south_cap_color = osg::Vec4ub(255,255,255,255);
    cache_only = false;
    normalize_edges = true;
    filename = "";
    profile = "";
}

void
MapConfig::setName( const std::string& _name )
{
    name = _name;
}

const std::string&
MapConfig::getName() const
{
    return name;
}

void
MapConfig::setFilename(const std::string &_filename)
{
    filename = _filename;
}

const std::string&
MapConfig::getFilename() const
{
    return filename;
}

const MapConfig::CoordinateSystemType&
MapConfig::getCoordinateSystemType() const
{
    return model_cstype;
}

void
MapConfig::setCoordinateSystemType( const CoordinateSystemType& type )
{
    model_cstype = type;
}

void
MapConfig::setVerticalScale( float value )
{
    vertical_scale = value;
}

float
MapConfig::getVerticalScale() const
{
    return vertical_scale;
}

SourceConfigList&
MapConfig::getImageSources()
{
    return image_sources;
}

const SourceConfigList&
MapConfig::getImageSources() const 
{
    return image_sources;
}

SourceConfigList&
MapConfig::getHeightFieldSources()
{
    return heightfield_sources;
}

const SourceConfigList&
MapConfig::getHeightFieldSources() const
{
    return heightfield_sources;
}

void
MapConfig::setProxyHost( const std::string& value )
{
    proxy_host = value;
}

const std::string&
MapConfig::getProxyHost() const 
{
    return proxy_host;
}

void
MapConfig::setProxyPort( unsigned short value )
{
    proxy_port = value;
}

unsigned short
MapConfig::getProxyPort() const
{
    return proxy_port;
}

void
MapConfig::setMinTileRangeFactor( float value )
{
    min_tile_range_factor = value;
}

float
MapConfig::getMinTileRangeFactor() const
{
    return min_tile_range_factor;
}

void
MapConfig::setNorthCapColor(const osg::Vec4ub &color)
{
    north_cap_color = color;
}

const osg::Vec4ub&
MapConfig::getNorthCapColor() const
{
    return north_cap_color;
}


void 
MapConfig::setSouthCapColor(const osg::Vec4ub &color)
{
    south_cap_color = color;
}

const osg::Vec4ub&
MapConfig::getSouthCapColor() const
{
    return south_cap_color;
}

void
MapConfig::setCacheOnly(bool cacheOnly)
{
    cache_only = cacheOnly;
}

bool
MapConfig::getCacheOnly() const

{
    return cache_only;
}

const std::string& 
MapConfig::getProfile() const
{
    return profile;
}

void MapConfig::setProfile(const std::string& profile)
{
    this->profile = profile;
}

const CacheConfig*
MapConfig::getCacheConfig() const
{
    return cache_config.get();
}

void
MapConfig::setCacheConfig(CacheConfig* cacheConfig)
{
    cache_config = cacheConfig;
}


bool
MapConfig::getNormalizeEdges() const
{
    return normalize_edges;
}

void
MapConfig::setNormalizeEdges(bool normalizeEdges)
{
    normalize_edges = normalizeEdges;
}



/************************************************************************/


SourceConfig::SourceConfig()
{
    //NOP
}

void
SourceConfig::setName( const std::string& _name )
{
    name = _name;
}

const std::string&
SourceConfig::getName() const
{
    return name;
}

void
SourceConfig::setDriver( const std::string& _driver )
{
    driver = _driver;
}

const std::string&
SourceConfig::getDriver() const
{
    return driver;
}

SourceProperties&
SourceConfig::getProperties()
{
    return properties;
}

const SourceProperties&
SourceConfig::getProperties() const
{
    return properties;
}

const CacheConfig*
SourceConfig::getCacheConfig() const
{
    return cache_config.get();
}

void
SourceConfig::setCacheConfig(CacheConfig* cacheConfig)
{
    cache_config = cacheConfig;
}


/************************************************************************/


CacheConfig::CacheConfig()
{
}
const std::string& 
CacheConfig::getType() const
{
    return _type;
}

void
CacheConfig::setType(const std::string &type)
{
    _type = type;
}

/**
* Gets the collection of name/value pairs for the cache.
*/
CacheProperties&
CacheConfig::getProperties()
{
    return _properties;
}

const CacheProperties& CacheConfig::getProperties() const
{
    return _properties;
}

/***********************************************************************/

#define ELEM_MAP               "map"
#define ATTR_NAME              "name"
#define ATTR_CSTYPE            "type"
#define ELEM_IMAGE             "image"
#define ELEM_HEIGHTFIELD       "heightfield"
#define ELEM_VERTICAL_SCALE    "vertical_scale"
#define ELEM_MIN_TILE_RANGE    "min_tile_range_factor"
#define ATTR_DRIVER            "driver"
#define ELEM_SKIRT_RATIO       "skirt_ratio"
#define ELEM_PROXY_HOST        "proxy_host"
#define ELEM_PROXY_PORT        "proxy_port"
#define ELEM_NORTH_CAP_COLOR   "north_cap_color"
#define ELEM_SOUTH_CAP_COLOR   "south_cap_color"
#define ELEM_CACHE_ONLY        "cache_only"
#define ELEM_PROFILE           "profile"
#define ELEM_NORMALIZE_EDGES   "normalize_edges"

#define ELEM_CACHE             "cache"
#define ATTR_TYPE              "type"

#define VALUE_TRUE             "true"
#define VALUE_FALSE            "false"

static CacheConfig*
readCache( XmlElement* e_cache )
{
    CacheConfig* cache = new CacheConfig();
    cache->setType( e_cache->getAttr( ATTR_TYPE ) );

    const XmlNodeList& e_props = e_cache->getChildren();
    for( XmlNodeList::const_iterator i = e_props.begin(); i != e_props.end(); i++ )
    {
        XmlElement* e_prop = dynamic_cast<XmlElement*>( i->get() );
        if ( e_prop )
        {
            std::string name = e_prop->getName();
            std::string value = e_prop->getText();
            if ( !name.empty() && !value.empty() )
            {
                cache->getProperties()[name] = value;
            }
        }
    }
    return cache;
}

static void writeCache( const CacheConfig* cache, XmlElement* e_cache )
{
    e_cache->getAttrs()[ATTR_TYPE] = cache->getType();

    //Add all the properties
    for (CacheProperties::const_iterator i = cache->getProperties().begin(); i != cache->getProperties().end(); i++ )
    {
        e_cache->addSubElement(i->first, i->second);
    }
}

static SourceConfig*
readSource( XmlElement* e_source )
{
    SourceConfig* source = new SourceConfig();

    source->setName( e_source->getAttr( ATTR_NAME ) );
    source->setDriver( e_source->getAttr( ATTR_DRIVER ) );

    //Try to read the cache for the source if one exists
    XmlElement* e_cache = static_cast<XmlElement*>(e_source->getSubElement( ELEM_CACHE ));
    if (e_cache)
    {
        source->setCacheConfig( readCache( e_cache) );
    }

    const XmlNodeList& e_props = e_source->getChildren();
    for( XmlNodeList::const_iterator i = e_props.begin(); i != e_props.end(); i++ )
    {
        XmlElement* e_prop = dynamic_cast<XmlElement*>( i->get() );
        if ( e_prop )
        {
            std::string name = e_prop->getName();
            std::string value = e_prop->getText();
            if ( !name.empty() && !value.empty() )
            {
                source->getProperties()[name] = value;
            }
        }
    }

    return source;
}

static void writeSource( const SourceConfig* source, XmlElement* e_source )
{
    e_source->getAttrs()[ATTR_NAME] = source->getName();
    e_source->getAttrs()[ATTR_DRIVER] = source->getDriver();

    //Add all the properties
    for (SourceProperties::const_iterator i = source->getProperties().begin(); i != source->getProperties().end(); i++ )
    {
        e_source->addSubElement(i->first, i->second);
    }

    if (source->getCacheConfig())
    {
       XmlElement* e_cache = new XmlElement(ELEM_CACHE);
       writeCache(source->getCacheConfig(), e_cache);
       e_source->getChildren().push_back(e_cache);
    }
}



osg::Vec4ub getColor(const std::string& str, osg::Vec4ub default_value)
{
    osg::Vec4ub color = default_value;
    std::istringstream strin(str);
    int r, g, b, a;
    if (strin >> r && strin >> g && strin >> b && strin >> a)
    {
        color.r() = (unsigned char)r;
        color.g() = (unsigned char)g;
        color.b() = (unsigned char)b;
        color.a() = (unsigned char)a;
    }
    return color;
}



static MapConfig*
readMap( XmlElement* e_map )
{
    MapConfig* map = new MapConfig();
    
    map->setName( e_map->getAttr( ATTR_NAME ) );
    
    std::string a_cstype = e_map->getAttr( ATTR_CSTYPE );
    if ( a_cstype == "geocentric" || a_cstype == "round" || a_cstype == "globe" || a_cstype == "earth" )
        map->setCoordinateSystemType( MapConfig::CSTYPE_GEOCENTRIC );
    else if ( a_cstype == "geographic" || a_cstype == "flat" || a_cstype == "plate carre" || a_cstype == "projected")
        map->setCoordinateSystemType( MapConfig::CSTYPE_PROJECTED );

    std::string cache_only = e_map->getSubElementText(ELEM_CACHE_ONLY);
    if (cache_only == VALUE_TRUE )
        map->setCacheOnly(true);
    else if (cache_only == VALUE_FALSE)
        map->setCacheOnly(false);

    std::string normalizeEdges = e_map->getSubElementText(ELEM_NORMALIZE_EDGES);
    if (normalizeEdges == VALUE_TRUE)
        map->setNormalizeEdges(true);
    else if (normalizeEdges == VALUE_FALSE)
        map->setNormalizeEdges(false);




    map->setVerticalScale( as<float>( e_map->getSubElementText( ELEM_VERTICAL_SCALE ), map->getVerticalScale() ) );
    map->setMinTileRangeFactor( as<float>( e_map->getSubElementText( ELEM_MIN_TILE_RANGE ), map->getMinTileRangeFactor() ) );
    map->setSkirtRatio(as<float>(e_map->getSubElementText( ELEM_SKIRT_RATIO ), map->getSkirtRatio()));

    map->setProxyHost( as<std::string>( e_map->getSubElementText( ELEM_PROXY_HOST ), map->getProxyHost() ) );
    map->setProxyPort( as<unsigned short>( e_map->getSubElementText( ELEM_PROXY_PORT ), map->getProxyPort() ) );

    map->setNorthCapColor(getColor(e_map->getSubElementText(ELEM_NORTH_CAP_COLOR ), map->getNorthCapColor()));
    map->setSouthCapColor(getColor(e_map->getSubElementText(ELEM_SOUTH_CAP_COLOR ), map->getSouthCapColor()));
    map->setProfile( e_map->getSubElementText( ELEM_PROFILE ) );



    XmlNodeList e_images = e_map->getSubElements( ELEM_IMAGE );
    for( XmlNodeList::const_iterator i = e_images.begin(); i != e_images.end(); i++ )
    {
        SourceConfig* image_source = readSource( static_cast<XmlElement*>( i->get() ) );
        if ( image_source )
            map->getImageSources().push_back( image_source );
    }

    XmlNodeList e_heightfields = e_map->getSubElements( ELEM_HEIGHTFIELD );
    for( XmlNodeList::const_iterator i = e_heightfields.begin(); i != e_heightfields.end(); i++ )
    {
        SourceConfig* heightfield_source = readSource( static_cast<XmlElement*>( i->get() ) );
        if ( heightfield_source )
            map->getHeightFieldSources().push_back( heightfield_source );
    }

    //Try to read the global map cache if one is specifiec
    XmlElement* e_cache = static_cast<XmlElement*>(e_map->getSubElement( ELEM_CACHE ));
    if (e_cache)
    {
        map->setCacheConfig( readCache( e_cache) );
    }

    //If the OSGEARTH_CACHE_ONLY environment variable is set, override whateve is in the map config
    if (getenv("OSGEARTH_CACHE_ONLY") != 0)
    {
        osg::notify(osg::NOTICE) << "Setting osgEarth to cache only mode due to OSGEARTH_CACHE_ONLY environment variable " << std::endl;
        map->setCacheOnly(true);
    }

    return map;
}

XmlDocument*
mapToXmlDocument( const MapConfig *map)
{
    //Create the root XML document
    osg::ref_ptr<XmlDocument> doc = new XmlDocument();
    
    //Create the root "map" node
    osg::ref_ptr<XmlElement> e_map = new XmlElement( "map" );
    doc->getChildren().push_back( e_map.get() );

    //Write the map's name
    e_map->getAttrs()[ATTR_NAME] = map->getName();

    //Write the coordinate system
    std::string cs;
    if (map->getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC) cs = "geocentric";
    else if (map->getCoordinateSystemType() == MapConfig::CSTYPE_PROJECTED) cs = "projected";
    else
    {
        osg::notify(osg::NOTICE) << "Unhandled CoordinateSystemType " << std::endl;
        return NULL;
    }
    e_map->getAttrs()[ATTR_CSTYPE] = cs;

    e_map->addSubElement( ELEM_CACHE_ONLY, toString<bool>(map->getCacheOnly()));
    e_map->addSubElement( ELEM_NORMALIZE_EDGES, toString<bool>(map->getNormalizeEdges()));

    e_map->addSubElement( ELEM_VERTICAL_SCALE, toString<float>( map->getVerticalScale() ) );
    e_map->addSubElement( ELEM_MIN_TILE_RANGE, toString<float>( map->getMinTileRangeFactor() ) );
    e_map->addSubElement( ELEM_SKIRT_RATIO, toString<float>( map->getSkirtRatio() ) );

    e_map->addSubElement( ELEM_PROXY_HOST, map->getProxyHost() );
    e_map->addSubElement( ELEM_PROXY_PORT, toString<unsigned short>(map->getProxyPort() ) );
    e_map->addSubElement( ELEM_NORTH_CAP_COLOR , toString<osg::Vec4ub>( map->getNorthCapColor() ) );
    e_map->addSubElement( ELEM_SOUTH_CAP_COLOR , toString<osg::Vec4ub>( map->getSouthCapColor() ) );
    e_map->addSubElement( ELEM_PROFILE, map->getProfile() );

    //Write all the image sources
    for (SourceConfigList::const_iterator i = map->getImageSources().begin(); i != map->getImageSources().end(); i++)
    {
        osg::ref_ptr<XmlElement> e_source = new XmlElement( ELEM_IMAGE );
        writeSource(i->get(), e_source.get());
        e_map->getChildren().push_back( e_source.get() );
    }

    //Write all the heightfield sources
    for (SourceConfigList::const_iterator i = map->getHeightFieldSources().begin(); i != map->getHeightFieldSources().end(); i++)
    {
        osg::ref_ptr<XmlElement> e_source = new XmlElement( ELEM_HEIGHTFIELD );
        writeSource(i->get(), e_source.get());
        e_map->getChildren().push_back( e_source.get() );
    }

    if (map->getCacheConfig())
    {
        XmlElement* e_cache = new XmlElement(ELEM_CACHE);
        writeCache(map->getCacheConfig(), e_cache);
        e_map->getChildren().push_back(e_cache);
    }

    return doc.release();
}

MapConfig*
MapConfigReaderWriter::readXml( std::istream& input )
{
    MapConfig* map = NULL;
    osg::ref_ptr<XmlDocument> doc = XmlDocument::load( input );
    if ( doc.valid() )
    {
        map = readMap( doc->getSubElement( ELEM_MAP ) );
    }
    return map;
}

MapConfig*
MapConfigReaderWriter::readXml( const std::string& location )
{
    MapConfig* map = NULL;
    if ( osgDB::containsServerAddress( location ) )
    {
        HTTPClient client;
        osg::ref_ptr<HTTPResponse> response = client.get( location );
        if ( response->isOK() && response->getNumParts() > 0 )
        {
            map = readXml( response->getPartStream( 0 ) );
        }
    }
    else
    {
        if (!osgDB::fileExists(location) && (osgDB::fileType(location) == osgDB::REGULAR_FILE))
        {
            osg::notify(osg::NOTICE) << location << " does not exists " << std::endl;
            return 0;
        }

        std::ifstream in( location.c_str() );
        map = readXml( in );
    }

    if (map)
    {
        std::string filename = location;
        if (!osgDB::containsServerAddress(filename))
        {
            filename = osgDB::getRealPath( location );
        }
        map->setFilename( location );
    }
    return map;
}

void
MapConfigReaderWriter::writeXml(const MapConfig* map, const std::string &location)
{
    std::ofstream out(location.c_str());
    writeXml(map, out);
}

void
MapConfigReaderWriter::writeXml(const MapConfig* map, std::ostream &output)
{
    osg::ref_ptr<XmlDocument> doc = mapToXmlDocument(map);    
    doc->store(output);
}