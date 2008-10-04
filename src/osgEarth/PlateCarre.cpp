#include <osgEarth/PlateCarre>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>
#include <osgDB/ReadFile>
#include <sstream>

using namespace osgEarth;

const std::string PlateCarreTileKey::TYPE_CODE = "P";

// these bounds form a square tile set; the bottom half of LOD 0 is not used.
#define MIN_LON -180
#define MAX_LON  180
#define MIN_LAT -270
#define MAX_LAT   90
#define PIXELS_PER_TILE 256

PlateCarreTileKey::PlateCarreTileKey( const PlateCarreTileKey& rhs )
: TileKey( rhs )
{
    //NOP
}

PlateCarreTileKey::PlateCarreTileKey( const std::string& input )
: TileKey( input, TileGridProfile( MIN_LON, MIN_LAT, MAX_LON, MAX_LAT, PIXELS_PER_TILE ) )
{
    //NOP
}

PlateCarreTileKey::PlateCarreTileKey( const std::string& input, const TileGridProfile& profile )
: TileKey( input, profile )
{
    //NOP
}

TileKey*
PlateCarreTileKey::getSubkey( unsigned int quadrant ) const
{
    if ( !subkeys[quadrant].valid() )
        const_cast<PlateCarreTileKey*>(this)->subkeys[quadrant] = new PlateCarreTileKey( key + (char)('0' + quadrant), profile );
    return subkeys[quadrant].get();
}

unsigned int
PlateCarreTileKey::getLevelOfDetail() const
{
    return (unsigned int)key.length();
}

bool
PlateCarreTileKey::getGeoExtents(double& out_min_lon,
                                 double& out_min_lat,
                                 double& out_max_lon,
                                 double& out_max_lat ) const
{
    double width =  profile.xMax() - profile.xMin(); //MAX_LON-MIN_LON;
    double height = profile.yMax() - profile.yMin(); //MAX_LAT-MIN_LAT;

    out_max_lat = profile.yMax(); //MAX_LAT;
    out_min_lon = profile.xMin(); //MIN_LON;

    for( unsigned int lod = 0; lod < getLevelOfDetail(); lod++ )
    {
        width /= 2.0;
        height /= 2.0;

        char digit = key[lod];
        switch( digit )
        {
            case '1': out_min_lon += width; break;
            case '2': out_max_lat -= height; break;
            case '3': out_min_lon += width; out_max_lat -= height; break;
        }
    }

    out_min_lat = out_max_lat - height;
    out_max_lon = out_min_lon + width;
    return true;
}
