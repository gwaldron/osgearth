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
#include <osgEarth/Registry>

#include <osg/Notify>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <fstream>

using namespace osgEarth;

MapConfig::MapConfig()
{
    _model_cstype = MapConfig::CSTYPE_GEOCENTRIC;
    _vertical_scale = 1.0f;
    _skirt_ratio = 0.02f;
    _sample_ratio = 1.0f;
    _proxy_port = 8080;
    _min_tile_range_factor = 5;
    _cache_only = false;
    _normalize_edges = false;
    _filename = "";
}

MapConfig::MapConfig( const MapConfig& rhs ) :
_model_cstype( rhs._model_cstype ),
_vertical_scale( rhs._vertical_scale ),
_skirt_ratio( rhs._skirt_ratio ),
_sample_ratio( rhs._sample_ratio ),
_proxy_host( rhs._proxy_host ),
_proxy_port( rhs._proxy_port ),
_min_tile_range_factor( rhs._min_tile_range_factor ),
_cache_only( rhs._cache_only ),
_normalize_edges( rhs._normalize_edges ),
_filename( rhs._filename ),
_image_sources( rhs._image_sources ),
_heightfield_sources( rhs._heightfield_sources ),
_cache_config( rhs._cache_config.get() ),
_profile_config( rhs._profile_config.get() ),
_global_options( rhs._global_options.get() )
{
    //NOP
}

void
MapConfig::setName( const std::string& name )
{
    _name = name;
}

const std::string&
MapConfig::getName() const
{
    return _name;
}

void
MapConfig::setFilename(const std::string& filename)
{
    _filename = filename;
}

const std::string&
MapConfig::getFilename() const
{
    return _filename;
}

const MapConfig::CoordinateSystemType&
MapConfig::getCoordinateSystemType() const
{
    return _model_cstype;
}

void
MapConfig::setCoordinateSystemType( const CoordinateSystemType& type )
{
    _model_cstype = type;
}

void
MapConfig::setVerticalScale( float value )
{
    _vertical_scale = value;
}

float
MapConfig::getVerticalScale() const
{
    return _vertical_scale;
}

void
MapConfig::setSampleRatio(float sample_ratio)
{
    _sample_ratio = sample_ratio;
}

float
MapConfig::getSampleRatio() const
{
    return _sample_ratio;
}

void
MapConfig::setSkirtRatio( float value )
{
    _skirt_ratio = value;
}

float
MapConfig::getSkirtRatio() const
{
    return _skirt_ratio;
}

SourceConfigList&
MapConfig::getImageSources()
{
    return _image_sources;
}

const SourceConfigList&
MapConfig::getImageSources() const 
{
    return _image_sources;
}

SourceConfigList&
MapConfig::getHeightFieldSources()
{
    return _heightfield_sources;
}

const SourceConfigList&
MapConfig::getHeightFieldSources() const
{
    return _heightfield_sources;
}

void
MapConfig::setProxyHost( const std::string& value )
{
    _proxy_host = value;
}

const std::string&
MapConfig::getProxyHost() const 
{
    return _proxy_host;
}

void
MapConfig::setProxyPort( unsigned short value )
{
    _proxy_port = value;
}

unsigned short
MapConfig::getProxyPort() const
{
    return _proxy_port;
}

void
MapConfig::setMinTileRangeFactor( float value )
{
    _min_tile_range_factor = value;
}

float
MapConfig::getMinTileRangeFactor() const
{
    return _min_tile_range_factor;
}

void
MapConfig::setCacheOnly(bool cacheOnly)
{
    _cache_only = cacheOnly;
}

bool
MapConfig::getCacheOnly() const

{
    return _cache_only;
}

CacheConfig*
MapConfig::getCacheConfig() const
{
    return _cache_config.get();
}

void
MapConfig::setCacheConfig(CacheConfig* cache_config)
{
    _cache_config = cache_config;
}


bool
MapConfig::getNormalizeEdges() const
{
    return _normalize_edges;
}

void
MapConfig::setNormalizeEdges(bool normalize_edges)
{
    _normalize_edges = normalize_edges;
}

const ProfileConfig*
MapConfig::getProfileConfig() const
{
    return _profile_config.get();
}

void
MapConfig::setProfileConfig(ProfileConfig* profile_config)
{
    _profile_config = profile_config;
}

void
MapConfig::setGlobalOptions( const osgDB::ReaderWriter::Options* options )
{
    _global_options = options;
}

const osgDB::ReaderWriter::Options*
MapConfig::getGlobalOptions() const
{
    return _global_options.get();
}




/************************************************************************/


SourceConfig::SourceConfig()
{
    //NOP
}

SourceConfig::SourceConfig( const SourceConfig& rhs ) :
_name( rhs._name ),
_driver( rhs._driver ),
_properties( rhs._properties ),
_profile_config( rhs._profile_config.get() ),
_cache_config( rhs._cache_config.get() )
{
    //NOP
}

SourceConfig::SourceConfig(const std::string& name,
                           const std::string& driver ) :
_name( name ),
_driver( driver )
{
    //NOP
}

SourceConfig::SourceConfig(const std::string& name,
                           const std::string& driver,
                           const SourceProperties& props ) :
_name( name ),
_driver( driver ),
_properties( props )
{
    //NOP
}

bool
SourceConfig::isValid() const
{
    return !_name.empty() && !_driver.empty();
}

void
SourceConfig::setName( const std::string& name )
{
    _name = name;
}

const std::string&
SourceConfig::getName() const
{
    return _name;
}

void
SourceConfig::setDriver( const std::string& driver )
{

    _driver = driver;
}

const std::string&
SourceConfig::getDriver() const
{
    return _driver;
}

SourceProperties&
SourceConfig::getProperties()
{
    return _properties;
}

const SourceProperties&
SourceConfig::getProperties() const
{
    return _properties;
}

void
SourceConfig::setProperty( const std::string& name, const std::string& value )
{
    _properties[name] = value;
}

CacheConfig*
SourceConfig::getCacheConfig() const
{
    return _cache_config.get();
}

void
SourceConfig::setCacheConfig(CacheConfig* cache_config)
{
    _cache_config = cache_config;
}

ProfileConfig*
SourceConfig::getProfileConfig() const
{
    return _profile_config.get();
}

void
SourceConfig::setProfileConfig( ProfileConfig* profile_config )
{
    _profile_config = profile_config;
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

void CacheConfig::inheritFrom(const osgEarth::CacheConfig *rhs)
{
  if (!rhs) return;

  //Inherit the type
  if (!rhs->getType().empty() ) setType( rhs->getType() );
  
  //Inherit the properites
  for (CacheProperties::const_iterator itr = rhs->getProperties().begin(); itr != rhs->getProperties().end(); ++itr)
  {
    getProperties()[itr->first] = itr->second;
  }
}

/***********************************************************************/

ProfileConfig::ProfileConfig():
_minX(DBL_MAX),
_minY(DBL_MAX),
_maxX(-DBL_MAX),
_maxY(-DBL_MAX)
{
}

const std::string& ProfileConfig::getNamedProfile() const
{
    return _namedProfile;
}

void
ProfileConfig::setNamedProfile( const std::string &namedProfile)
{
    _namedProfile = namedProfile;
}

const std::string&
ProfileConfig::getRefLayer() const
{
    return _refLayer;
}

void
ProfileConfig::setRefLayer(const std::string &refLayer)
{
    _refLayer = refLayer;
}

const std::string&
ProfileConfig::getSRS() const
{
    return _srs;
}

void
ProfileConfig::setSRS(const std::string& srs)
{
    _srs = srs;
}

bool ProfileConfig::areExtentsValid() const
{
    return _maxX >= _minX && _maxY >= _minY;
}

void
ProfileConfig::getExtents(double &minX, double &minY, double &maxX, double &maxY) const
{
    minX = _minX;
    minY = _minY;
    maxX = _maxX;
    maxY = _maxY;
}

void ProfileConfig::setExtents(double minX, double minY, double maxX, double maxY)
{
    _minX = minX;
    _minY = minY;
    _maxX = maxX;
    _maxY = maxY;
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
#define ELEM_SAMPLE_RATIO      "sample_ratio"
#define ELEM_PROXY_HOST        "proxy_host"
#define ELEM_PROXY_PORT        "proxy_port"
#define ELEM_CACHE_ONLY        "cache_only"
#define ELEM_NORMALIZE_EDGES   "normalize_edges"

#define ELEM_CACHE             "cache"
#define ATTR_TYPE              "type"

#define VALUE_TRUE             "true"
#define VALUE_FALSE            "false"

#define ELEM_PROFILE           "profile"
#define ATTR_MINX              "minx"
#define ATTR_MINY              "miny"
#define ATTR_MAXX              "maxx"
#define ATTR_MAXY              "maxy"
#define ATTR_SRS               "srs"
#define ATTR_USELAYER          "use"

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

static ProfileConfig* readProfileConfig( XmlElement* e_profile )
{
    ProfileConfig* profile = new ProfileConfig;
    profile->setNamedProfile( e_profile->getText() );
    profile->setRefLayer( e_profile->getAttr( ATTR_USELAYER ) );

    std::string srs_text = e_profile->getText();
    std::string srs_attr = e_profile->getAttr( ATTR_SRS );
    profile->setSRS( !srs_attr.empty()? srs_attr : srs_text );

    double minx, miny, maxx, maxy;
    profile->getExtents(minx, miny, maxx, maxy);

    //Get the bounding box
    minx = as<double>(e_profile->getAttr( ATTR_MINX ), minx);
    miny = as<double>(e_profile->getAttr( ATTR_MINY ), miny);
    maxx = as<double>(e_profile->getAttr( ATTR_MAXX ), maxx);
    maxy = as<double>(e_profile->getAttr( ATTR_MAXY ), maxy);

    profile->setExtents(minx, miny, maxx, maxy);

    return profile;
}

static void
writeProfileConfig(const ProfileConfig* profile, XmlElement* e_profile )
{
    e_profile->getChildren().push_back(new XmlText(profile->getNamedProfile()));
    e_profile->getAttrs()[ATTR_USELAYER] = profile->getRefLayer();
    e_profile->getAttrs()[ATTR_SRS] = profile->getSRS();

    if (profile->areExtentsValid())
    {
        double minx, miny, maxx, maxy;
        profile->getExtents(minx, miny, maxx, maxy);
        e_profile->getAttrs()[ATTR_MINX] = toString(minx);
        e_profile->getAttrs()[ATTR_MINY] = toString(miny);
        e_profile->getAttrs()[ATTR_MAXX] = toString(maxx);
        e_profile->getAttrs()[ATTR_MAXX] = toString(maxy);
    }
}


static SourceConfig
readSource( XmlElement* e_source )
{
    SourceConfig source;

    source.setName( e_source->getAttr( ATTR_NAME ) );
    source.setDriver( e_source->getAttr( ATTR_DRIVER ) );

    //Try to read the cache for the source if one exists
    XmlElement* e_cache = static_cast<XmlElement*>(e_source->getSubElement( ELEM_CACHE ));
    if (e_cache)
    {
        source.setCacheConfig( readCache( e_cache) );
    }

    // Check for an explicit profile override:
    XmlElement* e_profile = static_cast<XmlElement*>( e_source->getSubElement( ELEM_PROFILE ) );
    if ( e_profile )
    {
        source.setProfileConfig( readProfileConfig( e_profile ) );
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
                source.setProperty( name, value );
            }
        }
    }

    return source;
}

static void
writeSource( const SourceConfig& source, XmlElement* e_source )
{
    e_source->getAttrs()[ATTR_NAME] = source.getName();
    e_source->getAttrs()[ATTR_DRIVER] = source.getDriver();

    //Add all the properties
    for (SourceProperties::const_iterator i = source.getProperties().begin(); i != source.getProperties().end(); i++ )
    {
        e_source->addSubElement(i->first, i->second);
    }

    if (source.getCacheConfig())
    {
       XmlElement* e_cache = new XmlElement(ELEM_CACHE);
       writeCache(source.getCacheConfig(), e_cache);
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



static bool
readMap( XmlElement* e_map, MapConfig& out_map )
{
    bool success = true;

    out_map.setName( e_map->getAttr( ATTR_NAME ) );
    
    std::string a_cstype = e_map->getAttr( ATTR_CSTYPE );
    if ( a_cstype == "geocentric" || a_cstype == "round" || a_cstype == "globe" || a_cstype == "earth" )
        out_map.setCoordinateSystemType( MapConfig::CSTYPE_GEOCENTRIC );
    else if ( a_cstype == "geographic" || a_cstype == "flat" || a_cstype == "plate carre" || a_cstype == "projected")
        out_map.setCoordinateSystemType( MapConfig::CSTYPE_PROJECTED );
    else if ( a_cstype == "cube" )
        out_map.setCoordinateSystemType( MapConfig::CSTYPE_GEOCENTRIC_CUBE );

    std::string cache_only = e_map->getSubElementText(ELEM_CACHE_ONLY);
    if (cache_only == VALUE_TRUE )
        out_map.setCacheOnly(true);
    else if (cache_only == VALUE_FALSE)
        out_map.setCacheOnly(false);

    std::string normalizeEdges = e_map->getSubElementText(ELEM_NORMALIZE_EDGES);
    if (normalizeEdges == VALUE_TRUE)
        out_map.setNormalizeEdges(true);
    else if (normalizeEdges == VALUE_FALSE)
        out_map.setNormalizeEdges(false);

    out_map.setVerticalScale( as<float>( e_map->getSubElementText( ELEM_VERTICAL_SCALE ), out_map.getVerticalScale() ) );
    out_map.setMinTileRangeFactor( as<float>( e_map->getSubElementText( ELEM_MIN_TILE_RANGE ), out_map.getMinTileRangeFactor() ) );
    out_map.setSkirtRatio(as<float>(e_map->getSubElementText( ELEM_SKIRT_RATIO ), out_map.getSkirtRatio()));
    out_map.setSampleRatio(as<float>(e_map->getSubElementText( ELEM_SAMPLE_RATIO ), out_map.getSampleRatio()));


    out_map.setProxyHost( as<std::string>( e_map->getSubElementText( ELEM_PROXY_HOST ), out_map.getProxyHost() ) );
    out_map.setProxyPort( as<unsigned short>( e_map->getSubElementText( ELEM_PROXY_PORT ), out_map.getProxyPort() ) );

    //Read the profile definition
    XmlElement* e_profile = static_cast<XmlElement*>(e_map->getSubElement( ELEM_PROFILE ));
    if (e_profile)
    {
        out_map.setProfileConfig( readProfileConfig( e_profile ) );
    }



    XmlNodeList e_images = e_map->getSubElements( ELEM_IMAGE );
    for( XmlNodeList::const_iterator i = e_images.begin(); i != e_images.end(); i++ )
    {
        SourceConfig image_source = readSource( static_cast<XmlElement*>( i->get() ) );
        if ( image_source.isValid() )
        {
            image_source.setProperty( "default_tile_size", "256" ); //->getProperties()["default_tile_size"] = "256";
            out_map.getImageSources().push_back( image_source );
        }
    }

    XmlNodeList e_heightfields = e_map->getSubElements( ELEM_HEIGHTFIELD );
    for( XmlNodeList::const_iterator i = e_heightfields.begin(); i != e_heightfields.end(); i++ )
    {
        SourceConfig heightfield_source = readSource( static_cast<XmlElement*>( i->get() ) );
        if ( heightfield_source.isValid() )
        {
            heightfield_source.setProperty( "default_tile_size", "32" );
            //heightfield_source->getProperties()["default_tile_size"] = "32";
            out_map.getHeightFieldSources().push_back( heightfield_source );
        }
    }

    //Try to read the global map cache if one is specifiec
    XmlElement* e_cache = static_cast<XmlElement*>(e_map->getSubElement( ELEM_CACHE ));
    if (e_cache)
    {
        out_map.setCacheConfig( readCache( e_cache) );
    }

    //If the OSGEARTH_CACHE_ONLY environment variable is set, override whateve is in the map config
    if (getenv("OSGEARTH_CACHE_ONLY") != 0)
    {
        osg::notify(osg::NOTICE) << "Setting osgEarth to cache only mode due to OSGEARTH_CACHE_ONLY environment variable " << std::endl;
        out_map.setCacheOnly(true);
    }


    //Inherit the map CacheConfig with the override from the registry
    if (Registry::instance()->getCacheConfigOverride())
    {
        //If the map doesn't have a CacheConfig, create a new one
        if (!out_map.getCacheConfig()) out_map.setCacheConfig( new CacheConfig() );

        out_map.getCacheConfig()->inheritFrom( Registry::instance()->getCacheConfigOverride() );
        osg::notify(osg::NOTICE) << "Overriding Map Cache" << std::endl;
    }

    if (out_map.getCacheConfig())
    {
        //Inherit the Source CacheConfig's with the map's
        for (SourceConfigList::iterator itr = out_map.getImageSources().begin(); itr != out_map.getImageSources().end(); ++itr)
        {
            if (!itr->getCacheConfig()) itr->setCacheConfig( new CacheConfig() );
            itr->getCacheConfig()->inheritFrom( out_map.getCacheConfig() );
        }

        //Inherit the Source CacheConfig's with the map's
        for (SourceConfigList::iterator itr = out_map.getHeightFieldSources().begin(); itr != out_map.getHeightFieldSources().end(); ++itr)
        {
            if (!itr->getCacheConfig()) itr->setCacheConfig( new CacheConfig() );
            itr->getCacheConfig()->inheritFrom( out_map.getCacheConfig() );
        }
    }

    return success;
}

XmlDocument*
mapToXmlDocument( const MapConfig& map )
{
    //Create the root XML document
    osg::ref_ptr<XmlDocument> doc = new XmlDocument();
    
    //Create the root "map" node
    osg::ref_ptr<XmlElement> e_map = new XmlElement( "map" );
    doc->getChildren().push_back( e_map.get() );

    //Write the map's name
    e_map->getAttrs()[ATTR_NAME] = map.getName();

    //Write the coordinate system
    std::string cs;
    if (map.getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC) cs = "geocentric";
    else if (map.getCoordinateSystemType() == MapConfig::CSTYPE_PROJECTED) cs = "projected";
    else
    {
        osg::notify(osg::NOTICE) << "Unhandled CoordinateSystemType " << std::endl;
        return NULL;
    }
    e_map->getAttrs()[ATTR_CSTYPE] = cs;

    e_map->addSubElement( ELEM_CACHE_ONLY, toString<bool>(map.getCacheOnly()));
    e_map->addSubElement( ELEM_NORMALIZE_EDGES, toString<bool>(map.getNormalizeEdges()));

    e_map->addSubElement( ELEM_VERTICAL_SCALE, toString<float>( map.getVerticalScale() ) );
    e_map->addSubElement( ELEM_MIN_TILE_RANGE, toString<float>( map.getMinTileRangeFactor() ) );
    e_map->addSubElement( ELEM_SKIRT_RATIO, toString<float>( map.getSkirtRatio() ) );
    e_map->addSubElement( ELEM_SAMPLE_RATIO, toString<float>( map.getSampleRatio() ) );

    e_map->addSubElement( ELEM_PROXY_HOST, map.getProxyHost() );
    e_map->addSubElement( ELEM_PROXY_PORT, toString<unsigned short>( map.getProxyPort() ) );

    //Write all the image sources
    for (SourceConfigList::const_iterator i = map.getImageSources().begin(); i != map.getImageSources().end(); i++)
    {
        osg::ref_ptr<XmlElement> e_source = new XmlElement( ELEM_IMAGE );
        writeSource( *i, e_source.get());
        e_map->getChildren().push_back( e_source.get() );
    }

    //Write all the heightfield sources
    for (SourceConfigList::const_iterator i = map.getHeightFieldSources().begin(); i != map.getHeightFieldSources().end(); i++)
    {
        osg::ref_ptr<XmlElement> e_source = new XmlElement( ELEM_HEIGHTFIELD );
        writeSource( *i, e_source.get());
        e_map->getChildren().push_back( e_source.get() );
    }

    if (map.getCacheConfig())
    {
        XmlElement* e_cache = new XmlElement(ELEM_CACHE);
        writeCache(map.getCacheConfig(), e_cache);
        e_map->getChildren().push_back(e_cache);
    }

    if (map.getProfileConfig())
    {
        XmlElement* e_profile = new XmlElement(ELEM_PROFILE);
        writeProfileConfig(map.getProfileConfig(), e_profile);
        e_map->getChildren().push_back(e_profile);
    }

    return doc.release();
}

bool
MapConfigReaderWriter::readXml( std::istream& input, MapConfig& out_map )
{
    bool success = false;
    osg::ref_ptr<XmlDocument> doc = XmlDocument::load( input );
    if ( doc.valid() )
    {
        success = readMap( doc->getSubElement( ELEM_MAP ), out_map );
    }
    return success;
}

bool
MapConfigReaderWriter::readXml( const std::string& location, MapConfig& out_map )
{
    bool success = false;

    if ( osgDB::containsServerAddress( location ) )
    {
        HTTPClient client;
        osg::ref_ptr<HTTPResponse> response = client.get( location );
        if ( response->isOK() && response->getNumParts() > 0 )
        {
            success = readXml( response->getPartStream( 0 ), out_map );
        }
    }
    else
    {
        if (osgDB::fileExists(location) && (osgDB::fileType(location) == osgDB::REGULAR_FILE))
        {
            std::ifstream in( location.c_str() );
            success = readXml( in, out_map );
        }
    }

    if ( success )
    {
        std::string filename = location;
        if (!osgDB::containsServerAddress(filename))
        {
            filename = osgDB::getRealPath( location );
        }
        out_map.setFilename( filename );
    }

    return success;
}

bool
MapConfigReaderWriter::writeXml( const std::string& location, const MapConfig& map )
{
    std::ofstream out(location.c_str());
    return writeXml( out, map );
}

bool
MapConfigReaderWriter::writeXml( std::ostream& output, const MapConfig& map )
{
    osg::ref_ptr<XmlDocument> doc = mapToXmlDocument( map );    
    doc->store( output );
    return true;
}