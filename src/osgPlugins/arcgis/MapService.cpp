#include "MapService.h"
#include <osgEarth/HTTPClient>
#include <osgEarth/JsonUtils>
#include <osg/Notify>

#include <limits.h>

using namespace osgEarth;


MapServiceLayer::MapServiceLayer(int in_id, 
                                 const std::string& in_name) :
id( in_id ),
name( in_name )
{
    //NOP
}

int
MapServiceLayer::getId() const {
    return id;
}

const std::string&
MapServiceLayer::getName() const {
    return name;
}

//===========================================================================

TileInfo::TileInfo()
: is_valid( false )
{
    //NOP
}

TileInfo::TileInfo( int _tile_size, const std::string& _format, int _min_level, int _max_level ) :
tile_size( _tile_size ),
format( _format ),
min_level( _min_level ),
max_level( _max_level ),
is_valid( true )
{ 
    //NOP
}

TileInfo::TileInfo( const TileInfo& rhs ) :
tile_size( rhs.tile_size ),
format( rhs.format ),
min_level( rhs.min_level ),
max_level( rhs.max_level ),
is_valid( rhs.is_valid )
{
    //NOP
}

bool
TileInfo::isValid() const {
    return is_valid;
}

int
TileInfo::getTileSize() const {
    return tile_size;
}

const std::string&
TileInfo::getFormat() const {
    return format;
}

int
TileInfo::getMinLevel() const {
    return min_level;
}

int
TileInfo::getMaxLevel() const {
    return max_level;
}
                                 

//===========================================================================


MapService::MapService()
: is_valid( false )
{
    //NOP
}

bool 
MapService::isValid() const {
    return is_valid;
}

const TileGridProfile&
MapService::getProfile() const {
    return profile;
}

const TileInfo&
MapService::getTileInfo() const {
    return tile_info;
}

bool
MapService::init( const std::string& _url )
{
    url = _url;
    std::string json_url = url + "?f=json";  // request the data in JSON format

    HTTPClient client;
    osg::ref_ptr<HTTPResponse> response = client.get( json_url );
    if ( !response.valid() )
        return setError( "Unable to read metadata from ArcGIS service" );

    Json::Value doc;
    Json::Reader reader;
    if ( !reader.parse( response->getPartStream(0), doc ) )
        return setError( "Unable to parse metadata; invalid JSON" );

    // Read the profile. We are using "fullExtent"; perhaps an option to use "initialExtent" instead?
    double xmin = doc["fullExtent"].get("xmin", 0).asDouble();
    double ymin = doc["fullExtent"].get("ymin", 0).asDouble();
    double xmax = doc["fullExtent"].get("xmax", 0).asDouble();
    double ymax = doc["fullExtent"].get("ymax", 0).asDouble();
    int srs = doc["fullExtent"].get("spatialReference", "").get("wkid", 0).asInt();
    if ( xmax > xmin && ymax > ymin && srs != 0 )
        profile = TileGridProfile( xmin, ymin, xmax, ymax, ""+srs );
    else
        return setError( "Map service does not define a full extent" );

    // Read the layers list
    Json::Value j_layers = doc["layers"];
    if ( j_layers.empty() )
        return setError( "Map service contains no layers" );

    for( int i=0; i<j_layers.size(); i++ )
    {
        Json::Value layer = j_layers[i];
        int id = i; // layer.get("id", -1).asInt();
        std::string name = layer["name"].asString();

        if ( id >= 0 && !name.empty() )
        {
            layers.push_back( MapServiceLayer( id, name ) );
        }
    }

    // Read the tiling schema
    Json::Value j_tileinfo = doc["tileInfo"];
    if ( j_tileinfo.empty() )
        return setError( "Map service does not define a tiling schema" );

    // TODO: what do we do if the width <> height?
    int tile_rows = j_tileinfo.get( "rows", 0 ).asInt();
    int tile_cols = j_tileinfo.get( "cols", 0 ).asInt();
    if ( tile_rows <= 0 && tile_cols <= 0 )
        return setError( "Map service tile size not specified" );

    std::string format = j_tileinfo.get( "format", "" ).asString();
    if ( format.empty() )
        return setError( "Map service tile schema does not specify an image format" );

    Json::Value j_levels = j_tileinfo["lods"];
    if ( j_levels.empty() )
        return setError( "Map service tile schema contains no LODs" );
    
    int min_level = INT_MAX;
    int max_level = 0;
    for( int i=0; i<j_levels.size(); i++ )
    {
        int level = j_levels[i].get( "level", -1 ).asInt();
        if ( level >= 0 && level < min_level )
            min_level = level;
        if ( level >= 0 && level > max_level )
            max_level = level;
    }

    // now we're good.
    tile_info = TileInfo( tile_rows, format, min_level, max_level );
    is_valid = true;
    return is_valid;
}

bool
MapService::setError( const std::string& msg ) {
    error_msg = msg;
    return false;
}

const std::string&
MapService::getError() const {
    return error_msg;
}
