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
#include <osgEarth/EarthFile>
#include <osgEarth/XmlUtils>
#include <osgEarth/HTTPClient>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <OpenThreads/ScopedLock>

using namespace osgEarth;
using namespace OpenThreads;

EarthFile::EarthFile()
{
    //nop
}

EarthFile::EarthFile( Map* map, const MapEngineProperties& props ) :
_map( map ),
_engineProps( props )
{
    //nop
}

void
EarthFile::setMap( Map* map ) {
    _map = map;
}

void
EarthFile::setMapEngineProperties( const MapEngineProperties& props ) {
    _engineProps = props;
}

Map*
EarthFile::getMap() {
    return _map.get();
}

MapEngineProperties& 
EarthFile::getMapEngineProperties() {
    return _engineProps;
}

#define ELEM_MAP                      "map"
#define ATTR_NAME                     "name"
#define ATTR_CSTYPE                   "type"
#define ELEM_IMAGE                    "image"
#define ELEM_HEIGHTFIELD              "heightfield"
#define ELEM_VERTICAL_SCALE           "vertical_scale"
#define ELEM_MIN_TILE_RANGE           "min_tile_range_factor"
#define ELEM_USE_MERCATOR_LOCATOR     "use_mercator_locator"
#define ATTR_DRIVER                   "driver"
#define ATTR_REPROJECT_BEFORE_CACHING "reproject_before_caching"
#define ELEM_SKIRT_RATIO              "skirt_ratio"
#define ELEM_SAMPLE_RATIO             "sample_ratio"
#define ELEM_PROXY_HOST               "proxy_host"
#define ELEM_PROXY_PORT               "proxy_port"
#define ATTR_CACHE_ONLY               "cache_only"
#define ELEM_NORMALIZE_EDGES          "normalize_edges"
#define ELEM_COMBINE_LAYERS           "combine_layers"
#define ATTR_MIN_LEVEL                "min_level"
#define ATTR_MAX_LEVEL                "max_level"
#define ELEM_CACHE                    "cache"
#define ATTR_TYPE                     "type"

#define VALUE_TRUE                    "true"
#define VALUE_FALSE                   "false"

#define ELEM_PROFILE                  "profile"
#define ATTR_MINX                     "xmin"
#define ATTR_MINY                     "ymin"
#define ATTR_MAXX                     "xmax"
#define ATTR_MAXY                     "ymax"
#define ATTR_SRS                      "srs"
#define ATTR_USELAYER                 "use"

static CacheConfig
readCache( XmlElement* e_cache )
{
    CacheConfig cache;

    std::string type_token = e_cache->getAttr( ATTR_TYPE );
    if ( type_token == "tms" || type_token.empty() ) cache.setType( CacheConfig::TYPE_TMS );
    else if ( type_token == "tilecache" ) cache.setType( CacheConfig::TYPE_TILECACHE );
    else if ( type_token == "none" || type_token == "disabled") cache.setType( CacheConfig::TYPE_DISABLED );
    
    std::string cache_only = e_cache->getAttr( ATTR_CACHE_ONLY );
    if ( !cache_only.empty() )
        cache.runOffCacheOnly() = cache_only == VALUE_TRUE? true : false;
    
	std::string a_reproj = e_cache->getAttr( ATTR_REPROJECT_BEFORE_CACHING );
    if ( !a_reproj.empty() )
        cache.reprojectBeforeCaching() = a_reproj == VALUE_TRUE? true : false;

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
                cache.getProperties()[name] = value;
            }
        }
    }
    return cache;
}

static void
writeCache( const CacheConfig& cache, XmlElement* e_cache )
{
    e_cache->getAttrs()[ATTR_TYPE] = 
        cache.getType() == CacheConfig::TYPE_DISABLED?  "disabled" :
        cache.getType() == CacheConfig::TYPE_TILECACHE? "tilecache" :
        cache.getType() == CacheConfig::TYPE_TMS? "tms" :
        "tms";

    if ( cache.runOffCacheOnly().isSet() )
        e_cache->getAttrs()[ATTR_CACHE_ONLY] = cache.runOffCacheOnly().get();

    if ( cache.reprojectBeforeCaching().isSet() )
        e_cache->getAttrs()[ATTR_REPROJECT_BEFORE_CACHING] = cache.reprojectBeforeCaching().get();

    //Add all the properties
    for (Properties::const_iterator i = cache.getProperties().begin(); i != cache.getProperties().end(); i++ )
    {
        e_cache->addSubElement(i->first, i->second);
    }
}

static ProfileConfig
readProfileConfig( XmlElement* e_profile )
{
    ProfileConfig profile;

    profile.setNamedProfile( e_profile->getText() );
    profile.setRefLayer( e_profile->getAttr( ATTR_USELAYER ) );

    std::string srs_text = e_profile->getSubElementText( ATTR_SRS );
    std::string srs_attr = e_profile->getAttr( ATTR_SRS );
    profile.setSRS( !srs_attr.empty()? srs_attr : srs_text );

    double minx, miny, maxx, maxy;
    profile.getExtents(minx, miny, maxx, maxy);

    //Get the bounding box (sub element or attr is OK)
    minx = as<double>(e_profile->getSubElementText( ATTR_MINX ), as<double>(e_profile->getAttr( ATTR_MINX ), minx) );
    miny = as<double>(e_profile->getSubElementText( ATTR_MINY ), as<double>(e_profile->getAttr( ATTR_MINY ), miny) );
    maxx = as<double>(e_profile->getSubElementText( ATTR_MAXX ), as<double>(e_profile->getAttr( ATTR_MAXX ), maxx) );
    maxy = as<double>(e_profile->getSubElementText( ATTR_MAXY ), as<double>(e_profile->getAttr( ATTR_MAXY ), maxy) );

    profile.setExtents(minx, miny, maxx, maxy);

    return profile;
}

static void
writeProfileConfig(const ProfileConfig& profile, XmlElement* e_profile )
{
    e_profile->getChildren().push_back(new XmlText(profile.getNamedProfile()));
    e_profile->getAttrs()[ATTR_USELAYER] = profile.getRefLayer();
    e_profile->getAttrs()[ATTR_SRS] = profile.getSRS();

    if (profile.areExtentsValid())
    {
        double minx, miny, maxx, maxy;
        profile.getExtents(minx, miny, maxx, maxy);
        e_profile->getAttrs()[ATTR_MINX] = toString(minx);
        e_profile->getAttrs()[ATTR_MINY] = toString(miny);
        e_profile->getAttrs()[ATTR_MAXX] = toString(maxx);
        e_profile->getAttrs()[ATTR_MAXX] = toString(maxy);
    }
}


static MapLayer*
readLayer( XmlElement* e_source, MapLayer::Type layerType, const Properties& additionalDriverProps )
{
    std::string name = e_source->getAttr( ATTR_NAME );
    std::string driver = e_source->getAttr( ATTR_DRIVER );
    
    Properties driverProps;
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
                driverProps[name] = value;
            }
        }
    }

    // add in the additional properties:
    for(Properties::const_iterator i = additionalDriverProps.begin(); i != additionalDriverProps.end(); i++ )
        driverProps[i->first] = i->second;

    MapLayer* layer = new MapLayer(
        name,
        layerType,
        driver,
        driverProps );

    // Read the min and max LOD settings:
    int minLevel = as<double>(e_source->getSubElementText( ATTR_MIN_LEVEL ), as<double>(e_source->getAttr( ATTR_MIN_LEVEL ), -1) );
    if ( minLevel >= 0 )
        layer->minLevel() = minLevel;

    int maxLevel = as<double>(e_source->getSubElementText( ATTR_MAX_LEVEL ), as<double>(e_source->getAttr( ATTR_MAX_LEVEL ), -1) );
    if ( maxLevel >= 0 )
        layer->maxLevel() = maxLevel;

    // Try to read the cache for the source if one exists
    XmlElement* e_cache = static_cast<XmlElement*>(e_source->getSubElement( ELEM_CACHE ));
    if (e_cache)
    {
        layer->cacheConfig() = readCache( e_cache );
    }

    // Check for an explicit profile override:
    XmlElement* e_profile = static_cast<XmlElement*>( e_source->getSubElement( ELEM_PROFILE ) );
    if ( e_profile )
    {
        layer->profileConfig() = readProfileConfig( e_profile );
    }

    return layer;
}

static void
writeLayer( MapLayer* layer, XmlElement* e_source )
{
    e_source->getAttrs()[ATTR_NAME] = layer->getName();
    e_source->getAttrs()[ATTR_DRIVER] = layer->getDriver();
	//e_source->getAttrs()[ATTR_REPROJECT_BEFORE_CACHING] = toString<bool>(source.getReprojectBeforeCaching());

    //Add all the properties
    const Properties& driverProps = layer->getDriverProperties();
    for (Properties::const_iterator i = driverProps.begin(); i != driverProps.end(); i++ )
    {
        e_source->addSubElement(i->first, i->second);
    }

	//Write the profile config
	if ( layer->profileConfig().isSet() )
	{
		XmlElement* e_profile = new XmlElement(ELEM_PROFILE);
		writeProfileConfig( layer->profileConfig().get(), e_profile );
		e_source->getChildren().push_back(e_profile);
	}

	//Write the source config
    if ( layer->cacheConfig().isSet() )
    {
       XmlElement* e_cache = new XmlElement(ELEM_CACHE);
       writeCache(layer->cacheConfig().get(), e_cache);
       e_source->getChildren().push_back(e_cache);
    }
}


static osg::Vec4ub
getColor(const std::string& str, osg::Vec4ub default_value)
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



static bool
readMap( XmlElement* e_map, const std::string& referenceURI, EarthFile* earth )
{
    bool success = true;

    Map::CoordinateSystemType cstype = Map::CSTYPE_GEOCENTRIC;

    std::string a_cstype = e_map->getAttr( ATTR_CSTYPE );
    if ( a_cstype == "geocentric" || a_cstype == "round" || a_cstype == "globe" || a_cstype == "earth" )
        cstype = Map::CSTYPE_GEOCENTRIC;
    else if ( a_cstype == "geographic" || a_cstype == "flat" || a_cstype == "plate carre" || a_cstype == "projected")
        cstype = Map::CSTYPE_PROJECTED;
    else if ( a_cstype == "cube" )
        cstype = Map::CSTYPE_GEOCENTRIC_CUBE;

    osg::ref_ptr<Map> map = new Map( cstype );
    map->setReferenceURI( referenceURI );

    MapEngineProperties engineProps;

    map->setName( e_map->getAttr( ATTR_NAME ) );

    std::string use_merc_locator = e_map->getSubElementText(ELEM_USE_MERCATOR_LOCATOR);
    if (use_merc_locator == VALUE_TRUE )
        engineProps.setUseMercatorLocator( true );
    else if ( use_merc_locator == VALUE_FALSE )
        engineProps.setUseMercatorLocator( false );

    std::string combine_layers = e_map->getSubElementText(ELEM_COMBINE_LAYERS);
    if (combine_layers == VALUE_TRUE)
        engineProps.setCombineLayers(true);
    else if (combine_layers == VALUE_FALSE)
        engineProps.setCombineLayers(false);

    std::string normalizeEdges = e_map->getSubElementText(ELEM_NORMALIZE_EDGES);
    if (normalizeEdges == VALUE_TRUE)
        engineProps.setNormalizeEdges(true);
    else if (normalizeEdges == VALUE_FALSE)
        engineProps.setNormalizeEdges(false);

    engineProps.setVerticalScale( as<float>( e_map->getSubElementText( ELEM_VERTICAL_SCALE ), engineProps.getVerticalScale() ) );
    engineProps.setMinTileRangeFactor( as<float>( e_map->getSubElementText( ELEM_MIN_TILE_RANGE ), engineProps.getMinTileRangeFactor() ) );
    engineProps.setSkirtRatio(as<float>(e_map->getSubElementText( ELEM_SKIRT_RATIO ), engineProps.getSkirtRatio()));
    engineProps.setSampleRatio(as<float>(e_map->getSubElementText( ELEM_SAMPLE_RATIO ), engineProps.getSampleRatio()));

    engineProps.setProxyHost( as<std::string>( e_map->getSubElementText( ELEM_PROXY_HOST ), engineProps.getProxyHost() ) );
    engineProps.setProxyPort( as<unsigned short>( e_map->getSubElementText( ELEM_PROXY_PORT ), engineProps.getProxyPort() ) );

    //Read the profile definition
    XmlElement* e_profile = static_cast<XmlElement*>(e_map->getSubElement( ELEM_PROFILE ));
    if (e_profile)
    {
        map->profileConfig() = readProfileConfig( e_profile );
    }

    //Try to read the global map cache if one is specifiec
    XmlElement* e_cache = static_cast<XmlElement*>(e_map->getSubElement( ELEM_CACHE ));
    if (e_cache)
    {
        map->cacheConfig() = readCache(e_cache);
    }

    // Read the layers in LAST (otherwise they will not benefit from the cache/profile configuration)

    XmlNodeList e_images = e_map->getSubElements( ELEM_IMAGE );
    for( XmlNodeList::const_iterator i = e_images.begin(); i != e_images.end(); i++ )
    {
        Properties customProps;
        customProps["default_tile_size"] = "256";

        MapLayer* layer = readLayer( static_cast<XmlElement*>( i->get() ), MapLayer::TYPE_IMAGE, customProps );
        if ( layer )
        {
            map->addMapLayer( layer );
        }
    }

    XmlNodeList e_heightfields = e_map->getSubElements( ELEM_HEIGHTFIELD );
    for( XmlNodeList::const_iterator i = e_heightfields.begin(); i != e_heightfields.end(); i++ )
    {
        Properties customProps;
        customProps["default_tile_size"] = "32";

        MapLayer* layer = readLayer( static_cast<XmlElement*>( i->get() ), MapLayer::TYPE_HEIGHTFIELD, customProps );
        if ( layer )
        {
            map->addMapLayer( layer );
        }
    }


    earth->setMap( map.get() );
    earth->setMapEngineProperties( engineProps );

    return success;
}

XmlDocument*
mapToXmlDocument( Map* map, const MapEngineProperties& engineProps )
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
    if (map->getCoordinateSystemType() == Map::CSTYPE_GEOCENTRIC) cs = "geocentric";
    else if (map->getCoordinateSystemType() == Map::CSTYPE_PROJECTED) cs = "projected";
    else if ( map->getCoordinateSystemType() == Map::CSTYPE_GEOCENTRIC_CUBE) cs = "cube";
    else
    {
        osg::notify(osg::NOTICE) << "[osgEarth::EarthFile] Unhandled CoordinateSystemType " << std::endl;
        return NULL;
    }
    e_map->getAttrs()[ATTR_CSTYPE] = cs;

    //e_map->addSubElement( ELEM_CACHE_ONLY, toString<bool>(map.getCacheOnly()));
    e_map->addSubElement( ELEM_USE_MERCATOR_LOCATOR, toString<bool>(engineProps.getUseMercatorLocator()));
    e_map->addSubElement( ELEM_NORMALIZE_EDGES, toString<bool>(engineProps.getNormalizeEdges()));
    e_map->addSubElement( ELEM_COMBINE_LAYERS, toString<bool>(engineProps.getCombineLayers()));

    e_map->addSubElement( ELEM_VERTICAL_SCALE, toString<float>( engineProps.getVerticalScale() ) );
    e_map->addSubElement( ELEM_MIN_TILE_RANGE, toString<float>( engineProps.getMinTileRangeFactor() ) );
    e_map->addSubElement( ELEM_SKIRT_RATIO, toString<float>( engineProps.getSkirtRatio() ) );
    e_map->addSubElement( ELEM_SAMPLE_RATIO, toString<float>( engineProps.getSampleRatio() ) );

    e_map->addSubElement( ELEM_PROXY_HOST, engineProps.getProxyHost() );
    e_map->addSubElement( ELEM_PROXY_PORT, toString<unsigned short>( engineProps.getProxyPort() ) );

    //Write all the image sources
    for( MapLayerList::const_iterator i = map->getImageMapLayers().begin(); i != map->getImageMapLayers().end(); i++ )
    {
        osg::ref_ptr<XmlElement> e_source = new XmlElement( ELEM_IMAGE );
        writeLayer( i->get(), e_source.get());
        e_map->getChildren().push_back( e_source.get() );
    }

    //Write all the heightfield sources
    for (MapLayerList::const_iterator i = map->getHeightFieldMapLayers().begin(); i != map->getHeightFieldMapLayers().end(); i++ )
    {
        osg::ref_ptr<XmlElement> e_source = new XmlElement( ELEM_HEIGHTFIELD );
        writeLayer( i->get(), e_source.get());
        e_map->getChildren().push_back( e_source.get() );
    }

    if ( map->cacheConfig().isSet() )
    {
        XmlElement* e_cache = new XmlElement(ELEM_CACHE);
        writeCache(map->cacheConfig().get(), e_cache);
        e_map->getChildren().push_back(e_cache);
    }

    if ( map->profileConfig().isSet() )
    {
        XmlElement* e_profile = new XmlElement(ELEM_PROFILE);
        writeProfileConfig(map->profileConfig().get(), e_profile);
        e_map->getChildren().push_back(e_profile);
    }

    return doc.release();
}

bool
EarthFile::readXML( std::istream& input, const std::string& location )
{
    bool success = false;
    osg::ref_ptr<XmlDocument> doc = XmlDocument::load( input );
    if ( doc.valid() )
    {
        success = readMap( doc->getSubElement( ELEM_MAP ), location, this );
    }
    return success;
}

bool
EarthFile::readXML( const std::string& location )
{
    bool success = false;

    if ( osgDB::containsServerAddress( location ) )
    {
        HTTPResponse response = HTTPClient::get( location );
        if ( response.isOK() && response.getNumParts() > 0 )
        {
            success = readXML( response.getPartStream( 0 ), location );
        }
    }
    else
    {
        if (osgDB::fileExists(location) && (osgDB::fileType(location) == osgDB::REGULAR_FILE))
        {
            std::ifstream in( location.c_str() );
            success = readXML( in, location );
        }
    }

    if ( success )
    {
        std::string filename = location;
        if (!osgDB::containsServerAddress(filename))
        {
            filename = osgDB::getRealPath( location );
        }
        _map->setReferenceURI( filename );
    }

    return success;
}

bool
EarthFile::writeXML( const std::string& location )
{
    if ( !_map.valid() )
        return false;

    std::ofstream out( location.c_str() );
    return writeXML( out );
}

bool
EarthFile::writeXML( std::ostream& output )
{
    if ( !_map.valid() )
        return false;

    ScopedReadLock lock( _map->getMapDataMutex() );

    osg::ref_ptr<XmlDocument> doc = mapToXmlDocument( _map.get(), _engineProps );   
    doc->store( output );
    return true;
}
