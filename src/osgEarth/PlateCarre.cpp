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

void
PlateCarreTileKey::getPixelExtents(unsigned int& xmin,
                                   unsigned int& ymin,
                                   unsigned int& xmax,
                                   unsigned int& ymax) const
{
    unsigned int lod = getLevelOfDetail();
    unsigned int px = 0, py = 0;
    unsigned int delta = getMapSizePixels() >> 1;
    for( unsigned int i=0; i<lod; i++ )
    {
        switch( key[i] ) {
            case '1': px += delta; break;
            case '2': py += delta; break;
            case '3': px += delta; py += delta; break;
        }
        delta >>= 1;
    }
    xmin = px;
    ymin = py;
    xmax = px + (delta << 1);
    ymax = py + (delta << 1);
}

void
PlateCarreTileKey::getTileXY(unsigned int& out_tile_x,
                           unsigned int& out_tile_y) const
{
    unsigned int xmin, ymin, xmax, ymax;
    getPixelExtents( xmin, ymin, xmax, ymax );
    out_tile_x = xmin/profile.pixelsPerTile();
    out_tile_y = ymin/profile.pixelsPerTile();
}

osgTerrain::TileID
PlateCarreTileKey::getTileId() const
{
  //Convert the quadkey to a TileId

  //First, convert the base 4 quadkey to a decimal number
  int mult = 1;
  int qkdec = 0;
  for (int i = (key.size()-1); i >= 0; i--)
  {   
    //Get current digit for the level
    char c = key[i];
    int digit = atoi(&c);
    qkdec += (mult * digit);
    mult *= 4;
  }

  //The bits are interleved
  int tileX = 0;
  int tileY = 0;   

  mult = 1;
  while (qkdec > 0)
  {
    //If the bit is a 1, add the appropriate value to tileX
    if (qkdec & 0x1) tileX += mult;

    //Shift the bits left
    qkdec = qkdec >> 1;

    //If the bit is a 1, add the appropriate value to tileY
    if (qkdec & 0x1) tileY += mult;

    //Shift the bits left
    qkdec = qkdec >> 1;
   
    mult *= 2;
  }

  return osgTerrain::TileID(getLevelOfDetail(), tileX, tileY);
}

