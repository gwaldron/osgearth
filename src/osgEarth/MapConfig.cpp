#include <osgEarth/MapConfig>
#include <osgEarth/XmlUtils>
#include <osgEarth/HTTPClient>

#include <osg/Notify>

#include <osgDB/FileNameUtils>
#include <fstream>

using namespace osgEarth;

MapConfig::MapConfig()
{
    model_cstype = MapConfig::CSTYPE_GEOCENTRIC;
    tile_proj = MapConfig::PROJ_PLATE_CARRE;
    vertical_scale = 1.0f;
    skirt_ratio = 0.02;
    proxy_port = 8080;
    min_tile_range_factor = 8.0;
    north_cap_color = osg::Vec4ub(2,5,20,255);
    south_cap_color = osg::Vec4ub(255,255,255,255);
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
MapConfig::setCachePath( const std::string& _cache_path )
{
    cache_path = _cache_path;
}

const std::string&
MapConfig::getCachePath() const
{
    return cache_path;
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

const MapConfig::Projection&
MapConfig::getTileProjection() const 
{
    return tile_proj;
}

void
MapConfig::setTileProjection( const Projection& proj )
{
    tile_proj = proj;
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


/************************************************************************/

#define ELEM_MAP             "map"
#define ATTR_NAME            "name"
#define ATTR_CSTYPE          "type"
#define ELEM_PROJECTION      "projection"
#define ELEM_IMAGE           "image"
#define ELEM_HEIGHTFIELD     "heightfield"
#define ELEM_VERTICAL_SCALE  "vertical_scale"
#define ELEM_MIN_TILE_RANGE  "min_tile_range_factor"
#define ATTR_DRIVER          "driver"
#define ELEM_SKIRT_RATIO     "skirt_ratio"
#define ELEM_CACHE_PATH      "cache_path"
#define ELEM_PROXY_HOST      "proxy_host"
#define ELEM_PROXY_PORT      "proxy_port"
#define ELEM_NORTH_CAP_COLOR "north_cap_color"
#define ELEM_SOUTH_CAP_COLOR "south_cap_color"


static SourceConfig*
readSource( XmlElement* e_source )
{
    SourceConfig* source = new SourceConfig();

    source->setName( e_source->getAttr( ATTR_NAME ) );
    source->setDriver( e_source->getAttr( ATTR_DRIVER ) );

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
    else if ( a_cstype == "geographic" || a_cstype == "flat" || a_cstype == "plate carre" )
        map->setCoordinateSystemType( MapConfig::CSTYPE_GEOGRAPHIC );

    std::string proj = e_map->getSubElementText( ELEM_PROJECTION );
    if ( proj == "plate carre" || proj == "plate carree" || proj == "equirectangular" )
        map->setTileProjection( MapConfig::PROJ_PLATE_CARRE );
    else if ( proj == "mercator" || proj == "spherical mercator" )
        map->setTileProjection( MapConfig::PROJ_MERCATOR );

    map->setVerticalScale( as<float>( e_map->getSubElementText( ELEM_VERTICAL_SCALE ), map->getVerticalScale() ) );
    map->setMinTileRangeFactor( as<float>( e_map->getSubElementText( ELEM_MIN_TILE_RANGE ), map->getMinTileRangeFactor() ) );
    map->setSkirtRatio(as<float>(e_map->getSubElementText( ELEM_SKIRT_RATIO ), map->getSkirtRatio()));
    map->setCachePath( as<std::string>( e_map->getSubElementText( ELEM_CACHE_PATH ), map->getCachePath() ) );

    //If the OSGEARTH_FILE_CACHE environment variable is set, override whatever is in the map config.
    std::string cacheFilePath;
    const char* fileCachePath = getenv("OSGEARTH_FILE_CACHE");
    if (fileCachePath) //Env Cache Directory
    {
        osg::notify(osg::INFO) << "Overriding cache path with OSGEARTH_FILE_CACHE environment variable " << fileCachePath << std::endl;
        map->setCachePath(std::string(fileCachePath));
    }





    map->setProxyHost( as<std::string>( e_map->getSubElementText( ELEM_PROXY_HOST ), map->getProxyHost() ) );
    map->setProxyPort( as<unsigned short>( e_map->getSubElementText( ELEM_PROXY_PORT ), map->getProxyPort() ) );

    map->setNorthCapColor(getColor(e_map->getSubElementText(ELEM_NORTH_CAP_COLOR ), map->getNorthCapColor()));
    map->setSouthCapColor(getColor(e_map->getSubElementText(ELEM_SOUTH_CAP_COLOR ), map->getSouthCapColor()));



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

    //Set the cache path for each of the imagery sources
    for (SourceConfigList::iterator itr = map->getImageSources().begin();
         itr != map->getImageSources().end();
         ++itr)
    {
        (*itr)->getProperties()[ELEM_CACHE_PATH] = map->getCachePath();
    }

    //Set the cache path for each of the heightfield sources
    for (SourceConfigList::iterator itr = map->getHeightFieldSources().begin();
        itr != map->getHeightFieldSources().end();
        ++itr)
    {
        (*itr)->getProperties()[ELEM_CACHE_PATH] = map->getCachePath();
    }

    return map;
}

MapConfig*
MapConfigReader::readXml( std::istream& input )
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
MapConfigReader::readXml( const std::string& location )
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
        std::ifstream in( location.c_str() );
        map = readXml( in );
    }
    return map;
}
