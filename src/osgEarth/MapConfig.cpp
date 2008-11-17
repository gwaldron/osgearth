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
    offline_hint = false;
    filename = "";
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

std::string
MapConfig::getFullCachePath() const
{
    //Get the full path to the cache directory
    std::string real_path = osgDB::convertToLowerCase( osgDB::convertFileNameToNativeStyle( osgDB::getRealPath( cache_path ) ) );
    std::string tmp_cache_path = osgDB::convertToLowerCase( osgDB::convertFileNameToNativeStyle ( cache_path ) );

    //If the full path isn't equal to the cache_path, the path should be relative to the location of the map file
    if (real_path != tmp_cache_path)
    {
        return osgDB::getRealPath( osgDB::concatPaths( osgDB::getFilePath( filename ), tmp_cache_path ) );
    }

    return real_path;
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

void
MapConfig::setOfflineHint(const bool &hint)
{
    offline_hint = hint;
}

const bool
MapConfig::getOfflineHint() const

{
    return offline_hint;
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

#define ELEM_MAP               "map"
#define ATTR_NAME              "name"
#define ATTR_CSTYPE            "type"
#define ELEM_PROJECTION        "projection"
#define ELEM_IMAGE             "image"
#define ELEM_HEIGHTFIELD       "heightfield"
#define ELEM_VERTICAL_SCALE    "vertical_scale"
#define ELEM_MIN_TILE_RANGE    "min_tile_range_factor"
#define ATTR_DRIVER            "driver"
#define ELEM_SKIRT_RATIO       "skirt_ratio"
#define ELEM_CACHE_PATH        "cache_path"
#define ELEM_PROXY_HOST        "proxy_host"
#define ELEM_PROXY_PORT        "proxy_port"
#define ELEM_NORTH_CAP_COLOR   "north_cap_color"
#define ELEM_SOUTH_CAP_COLOR   "south_cap_color"
#define ELEM_CONNECTION_STATUS "connection_status"


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

static void writeSource( const SourceConfig* source, XmlElement* e_source )
{
    e_source->getAttrs()[ATTR_NAME] = source->getName();
    e_source->getAttrs()[ATTR_DRIVER] = source->getDriver();

    //Add all the properties
    for (SourceProperties::const_iterator i = source->getProperties().begin(); i != source->getProperties().end(); i++ )
    {
        e_source->addSubElement(i->first, i->second);
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
    else if ( a_cstype == "geographic" || a_cstype == "flat" || a_cstype == "plate carre" )
        map->setCoordinateSystemType( MapConfig::CSTYPE_GEOGRAPHIC );

    std::string proj = e_map->getSubElementText( ELEM_PROJECTION );
    if ( proj == "plate carre" || proj == "plate carree" || proj == "equirectangular" )
        map->setTileProjection( MapConfig::PROJ_PLATE_CARRE );
    else if ( proj == "mercator" || proj == "spherical mercator" )
        map->setTileProjection( MapConfig::PROJ_MERCATOR );

    std::string conn_status = e_map->getSubElementText(ELEM_CONNECTION_STATUS);
    if (conn_status == "offline")
        map->setOfflineHint(true);
    else
        map->setOfflineHint(false);



    map->setVerticalScale( as<float>( e_map->getSubElementText( ELEM_VERTICAL_SCALE ), map->getVerticalScale() ) );
    map->setMinTileRangeFactor( as<float>( e_map->getSubElementText( ELEM_MIN_TILE_RANGE ), map->getMinTileRangeFactor() ) );
    map->setSkirtRatio(as<float>(e_map->getSubElementText( ELEM_SKIRT_RATIO ), map->getSkirtRatio()));
    map->setCachePath( as<std::string>( e_map->getSubElementText( ELEM_CACHE_PATH ), map->getCachePath() ) );

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

    //If the OSGEARTH_FILE_CACHE environment variable is set, override whatever is in the map config.
    std::string cacheFilePath;
    const char* fileCachePath = getenv("OSGEARTH_FILE_CACHE");
    if (fileCachePath) //Env Cache Directory
    {
        osg::notify(osg::INFO) << "Overriding cache path with OSGEARTH_FILE_CACHE environment variable " << fileCachePath << std::endl;
        map->setCachePath(std::string(fileCachePath));
    }

    //If the OSGEARTH_OFFLINE environment variable is set, override whateve is in the map config
    const char* offline = getenv("OSGEARTH_OFFLINE");
    if (offline)
    {
        if (strcmp(offline, "YES") == 0)
        {
            osg::notify(osg::NOTICE) << "Setting osgEarth to offline mode due to OSGEARTH_OFFLINE environment variable " << std::endl;
            map->setOfflineHint(true);
        }
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
    doc->getChildren().push_back( e_map );

    //Write the map's name
    e_map->getAttrs()[ATTR_NAME] = map->getName();

    //Write the coordinate system
    std::string cs;
    if (map->getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC) cs = "geocentric";
    else if (map->getCoordinateSystemType() == MapConfig::PROJ_MERCATOR) cs = "geographic";
    else
    {
        osg::notify(osg::NOTICE) << "Unhandled CoordinateSystemType " << std::endl;
        return NULL;
    }
    e_map->getAttrs()[ATTR_CSTYPE] = cs;

    //Write the projection
    std::string projection;
    if (map->getTileProjection() == MapConfig::PROJ_PLATE_CARRE) projection = "geographic";
    else if (map->getTileProjection() == MapConfig::PROJ_MERCATOR) projection = "mercator";
    e_map->addSubElement( ELEM_PROJECTION, projection );

    //Write out the connection status
    std::string conn_status = map->getOfflineHint() ? "offline" : "online";
    e_map->addSubElement( ELEM_CONNECTION_STATUS, conn_status );

    e_map->addSubElement( ELEM_VERTICAL_SCALE, toString<float>( map->getVerticalScale() ) );
    e_map->addSubElement( ELEM_MIN_TILE_RANGE, toString<float>( map->getMinTileRangeFactor() ) );
    e_map->addSubElement( ELEM_SKIRT_RATIO, toString<float>( map->getSkirtRatio() ) );
    e_map->addSubElement( ELEM_CACHE_PATH, map->getCachePath() );

    e_map->addSubElement( ELEM_PROXY_HOST, map->getProxyHost() );
    e_map->addSubElement( ELEM_PROXY_PORT, toString<unsigned short>(map->getProxyPort() ) );
    e_map->addSubElement( ELEM_NORTH_CAP_COLOR , toString<osg::Vec4ub>( map->getNorthCapColor() ) );
    e_map->addSubElement( ELEM_SOUTH_CAP_COLOR , toString<osg::Vec4ub>( map->getSouthCapColor() ) );

    //Write all the image sources
    for (SourceConfigList::const_iterator i = map->getImageSources().begin(); i != map->getImageSources().end(); i++)
    {
        osg::ref_ptr<XmlElement> e_source = new XmlElement( ELEM_IMAGE );
        writeSource(i->get(), e_source.get());
        e_map->getChildren().push_back( e_source );
    }

    //Write all the heightfield sources
    for (SourceConfigList::const_iterator i = map->getHeightFieldSources().begin(); i != map->getHeightFieldSources().end(); i++)
    {
        osg::ref_ptr<XmlElement> e_source = new XmlElement( ELEM_HEIGHTFIELD );
        writeSource(i->get(), e_source.get());
        e_map->getChildren().push_back( e_source );
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
        std::ifstream in( location.c_str() );
        map = readXml( in );
    }

    map->setFilename( osgDB::getRealPath( location ) );
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