#include <osgEarth/MapConfig>
#include <osgEarth/XmlUtils>
#include <osgEarth/HTTPClient>
#include <osgDB/FileNameUtils>
#include <fstream>

using namespace osgEarth;

MapConfig::MapConfig()
{
    cstype = MapConfig::CSTYPE_GEOCENTRIC;
    vertical_scale = 1.0f;
    skirt_ratio = 0.02;
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

const MapConfig::CoordinateSystemType&
MapConfig::getCoordinateSystemType() const
{
    return cstype;
}

void
MapConfig::setCoordinateSystemType( const CoordinateSystemType& _cstype )
{
    cstype = _cstype;
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

#define ELEM_MAP            "map"
#define ATTR_NAME           "name"
#define ATTR_CSTYPE         "type"
#define ELEM_IMAGE          "image"
#define ELEM_HEIGHTFIELD    "heightfield"
#define ELEM_VERTICAL_SCALE "vertical_scale"
#define ATTR_DRIVER         "driver"
#define ELEM_SKIRT_RATIO    "skirt_ratio"

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

static MapConfig*
readMap( XmlElement* e_map )
{
    MapConfig* map = new MapConfig();
    
    map->setName( e_map->getAttr( ATTR_NAME ) );
    
    std::string a_cstype = e_map->getAttr( ATTR_CSTYPE );
    if ( a_cstype == "geocentric" || a_cstype == "round" || a_cstype == "globe" || a_cstype == "earth" )
        map->setCoordinateSystemType( MapConfig::CSTYPE_GEOCENTRIC );
    else if ( a_cstype == "plate carre" || a_cstype == "flat" || a_cstype == "geographic" || a_cstype == "equirectangular" || a_cstype == "projected" )
        map->setCoordinateSystemType( MapConfig::CSTYPE_PLATE_CARRE );

    map->setVerticalScale( as<float>( e_map->getSubElementText( ELEM_VERTICAL_SCALE ), map->getVerticalScale() ) );
    map->setSkirtRatio(as<float>(e_map->getSubElementText( ELEM_SKIRT_RATIO ), map->getSkirtRatio()));

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
