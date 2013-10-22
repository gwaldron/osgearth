#include "MapService.h"
#include <osgEarth/JsonUtils>
#include <osgEarth/Registry>
#include <osg/Notify>
#include <sstream>
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

TileInfo::TileInfo( int _tile_size, const std::string& _format, int _min_level, int _max_level, int _num_tiles_wide, int _num_tiles_high ) :
format( _format ),
tile_size( _tile_size ),
min_level( _min_level ),
max_level( _max_level ),
is_valid( true ),
num_tiles_wide(_num_tiles_wide),
num_tiles_high(_num_tiles_high)
{ 
    //NOP
}

TileInfo::TileInfo( const TileInfo& rhs ) :
format( rhs.format ),
tile_size( rhs.tile_size ),
min_level( rhs.min_level ),
max_level( rhs.max_level ),
is_valid( rhs.is_valid ),
num_tiles_wide( rhs.num_tiles_wide ),
num_tiles_high( rhs.num_tiles_high )
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

int
TileInfo::getNumTilesWide() const {
    return num_tiles_wide;
}

int
TileInfo::getNumTilesHigh() const {
    return num_tiles_high;
}
                                 

//===========================================================================


MapService::MapService() :
is_valid( false ),
tiled( false )
{
    //NOP
}

bool 
MapService::isValid() const {
    return is_valid;
}

bool
MapService::isTiled() const {
    return tiled;
}

const Profile*
MapService::getProfile() const {
    return profile.get();
}

const TileInfo&
MapService::getTileInfo() const {
    return tile_info;
}

bool
MapService::init( const URI& _uri, const osgDB::ReaderWriter::Options* options )
{
    uri = _uri;
    std::string sep = uri.full().find( "?" ) == std::string::npos ? "?" : "&";
    std::string json_url = uri.full() + sep + std::string("f=pjson");  // request the data in JSON format

    ReadResult r = URI(json_url).readString( options );
    if ( r.failed() )
        return setError( "Unable to read metadata from ArcGIS service" );

    Json::Value doc;
    Json::Reader reader;
//    if ( !reader.parse( response.getPartStream(0), doc ) )
    if ( !reader.parse( r.getString(), doc ) )
        return setError( "Unable to parse metadata; invalid JSON" );

    // Read the profile. We are using "fullExtent"; perhaps an option to use "initialExtent" instead?
    double xmin = doc["fullExtent"].get("xmin", 0).asDouble();
    double ymin = doc["fullExtent"].get("ymin", 0).asDouble();
    double xmax = doc["fullExtent"].get("xmax", 0).asDouble();
    double ymax = doc["fullExtent"].get("ymax", 0).asDouble();
    int srs = doc["fullExtent"].get("spatialReference", osgEarth::Json::Value::null).get("wkid", 0).asInt();
    
    //Assumes the SRS is going to be an EPSG code
    std::stringstream ss;
    ss << "epsg:" << srs;
    
    if ( ! (xmax > xmin && ymax > ymin && srs != 0 ) )
    {
        return setError( "Map service does not define a full extent" );
    }

    // Read the layers list
    Json::Value j_layers = doc["layers"];
    if ( j_layers.empty() )
        return setError( "Map service contains no layers" );

    for( unsigned int i=0; i<j_layers.size(); i++ )
    {
        Json::Value layer = j_layers[i];
        int id = i; // layer.get("id", -1).asInt();
        std::string name = layer["name"].asString();

        if ( id >= 0 && !name.empty() )
        {
            layers.push_back( MapServiceLayer( id, name ) );
        }
    }

    tiled = false;
    std::string format = "png";
    int tile_rows = 256;
    int tile_cols = 256;
    int min_level = 25;
    int max_level = 0;
    int num_tiles_wide = 1;
    int num_tiles_high = 1;

    // Read the tiling schema
    Json::Value j_tileinfo = doc["tileInfo"];
    if ( !j_tileinfo.empty() )
    {
        tiled = true;

     //   return setError( "Map service does not define a tiling schema" );

        // TODO: what do we do if the width <> height?
        tile_rows = j_tileinfo.get( "rows", 0 ).asInt();
        tile_cols = j_tileinfo.get( "cols", 0 ).asInt();
        if ( tile_rows <= 0 && tile_cols <= 0 )
            return setError( "Map service tile size not specified" );

        format = j_tileinfo.get( "format", "" ).asString();
        if ( format.empty() )
            return setError( "Map service tile schema does not specify an image format" );

        Json::Value j_levels = j_tileinfo["lods"];
        if ( j_levels.empty() )
            return setError( "Map service tile schema contains no LODs" );
        
        min_level = INT_MAX;
        max_level = 0;
        for( unsigned int i=0; i<j_levels.size(); i++ )
        {
            int level = j_levels[i].get( "level", -1 ).asInt();
            if ( level >= 0 && level < min_level )
                min_level = level;
            if ( level >= 0 && level > max_level )
                max_level = level;
        }

        if (j_levels.size() > 0)
        {
            int l = j_levels[0u].get("level", -1).asInt();
            double res = j_levels[0u].get("resolution", 0.0).asDouble();
            num_tiles_wide = (int)osg::round((xmax - xmin) / (res * tile_cols));
            num_tiles_high = (int)osg::round((ymax - ymin) / (res * tile_cols));

            //In case the first level specified isn't level 0, compute the number of tiles at level 0
            for (int i = 0; i < l; i++)
            {
                num_tiles_wide /= 2;
                num_tiles_high /= 2;
            }

            //profile.setNumTilesWideAtLod0(num_tiles_wide);
            //profile.setNumTilesHighAtLod0(num_tiles_high);
        }
    }

	 std::string ssStr;
	 ssStr = ss.str();

    osg::ref_ptr< SpatialReference > spatialReference = SpatialReference::create( ssStr );
    if (spatialReference->isGeographic())
    {
        //If we have a geographic SRS, just use the geodetic profile
        profile = Registry::instance()->getGlobalGeodeticProfile();
    }
    else if (spatialReference->isMercator())
    {
        //If we have a mercator SRS, just use the mercator profile
        profile = Registry::instance()->getGlobalMercatorProfile();
    }
    else
    {
        //It's not geodetic or mercator, so try to use the full extent
        profile = Profile::create(
            spatialReference.get(),
            xmin, ymin, xmax, ymax,
            num_tiles_wide,
            num_tiles_high);
    }    



    // now we're good.
    tile_info = TileInfo( tile_rows, format, min_level, max_level, num_tiles_wide, num_tiles_high);
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
