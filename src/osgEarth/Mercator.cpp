#include <osgEarth/Mercator>
#include <osgEarth/PlateCarre>
#include <osg/Math>
#include <osg/Notify>
#include <sstream>
#include <algorithm>

using namespace osgEarth;

/********************************************************************/

#define MIN_LON    -180.0
#define MAX_LON     180.0
#define MIN_LAT     -85.05112878  
#define MAX_LAT      85.05112878

MercatorCellKey::MercatorCellKey( const MercatorCellKey& rhs )
{
    qk = rhs.qk;
}

MercatorCellKey::MercatorCellKey( const std::string& input )
{
    qk = input;
}

MercatorCellKey::MercatorCellKey( unsigned int tile_x, unsigned int tile_y, unsigned int lod )
{       
    std::stringstream ss;
    for( unsigned i = lod; i > 0; i-- )
    {
        char digit = '0';
        unsigned int mask = 1 << (i-1);
        if ( (tile_x & mask) != 0 )
        {
            digit++;
        }
        if ( (tile_y & mask) != 0 )
        {
            digit += 2;
        }
        ss << digit;
    }
    qk = ss.str();
}

MercatorCellKey
MercatorCellKey::getSubkey( unsigned int quadrant ) const
{
    return MercatorCellKey( qk + (char)('0'+quadrant) );
}

const std::string&
MercatorCellKey::str() const
{
    return qk;
}

unsigned int
MercatorCellKey::getLevelOfDetail() const
{
    return (unsigned int)qk.length();
}

static 
unsigned int getMapSize( unsigned int lod )
{
    return (unsigned int) 256 << lod;
}

void
MercatorCellKey::getTileXY(unsigned int& out_tile_x,
                           unsigned int& out_tile_y) const
{
    unsigned int xmin, ymin, xmax, ymax;
    getPixelExtents( xmin, ymin, xmax, ymax );
    out_tile_x = xmin/256;
    out_tile_y = ymin/256;
}

void
MercatorCellKey::getPixelExtents(unsigned int& xmin,
                                 unsigned int& ymin,
                                 unsigned int& xmax,
                                 unsigned int& ymax) const
{
    unsigned int lod = getLevelOfDetail();
    unsigned int px = 0, py = 0;
    unsigned int delta = getMapSize( lod ) >> 1;
    for( unsigned int i=0; i<lod; i++ )
    {
        switch( qk[i] ) {
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
MercatorCellKey::getGeoExtents(double& lon_min, double& lat_min,
                               double& lon_max, double& lat_max) const
{
    unsigned int xmin, ymin, xmax, ymax;
    getPixelExtents( xmin, ymin, xmax, ymax );
    lat_min = getLatitude( ymax );
    lat_max = getLatitude( ymin );
    //TODO
    lon_min = 0;
    lon_max = 0;
}

double
MercatorCellKey::getLatitude( unsigned int pixel_y ) const
{
    unsigned int lod = getLevelOfDetail();
    double my  = -osg::PI + (1.0 - (double)pixel_y/(double)getMapSize(lod)) * (2.0*osg::PI);
    double deg = osg::RadiansToDegrees( 2.0 * atan( exp( my ) ) - .5*osg::PI );

    //osg::notify(osg::NOTICE)
    //    << "getLatitude: pixel_y = " << pixel_y << ", lod = " << lod << ", my = " << my << ", deg = " << deg
    //    << std::endl;

    return deg;

    //return osg::RadiansToDegrees(
    //    2.0 * atan( exp( osg::PI * (1.0 - 2.0 * (double)pixel_y/(double)getMapSize(lod) ))) - osg::PI*0.5 );
}
    
//    // Extent maps to +-180 for Longitude and +-85 for Latitude
//function LongLatFromCoordinate (coordinate, worldRect)
//{
//var x = ((coordinate.X - worldRect.Left) * 360.0 / worldRect.Width) - 180.0;
//var y = RadiansToDegrees (2 * Math.atan(Math.exp(Math.PI * (1.0 - 2.0 * (coordinate.Y - worldRect.Top) / worldRect.Height))) - Math.PI * 0.5);
//var point = eval('({"X":'+x+', "Y":'+y+'})');
//return point;
//
//}


static double
clamp( double n, double min_value, double max_value )
{
    return osg::minimum( osg::maximum( n, min_value ), max_value );
}

int
MercatorCellKey::longLatToPixelXY(double lon, double lat, unsigned int lod, 
                                  unsigned int& out_x, unsigned int& out_y )
{
    double x = (lon + 180.0) / 360.0;
    double sin_lat = sin( osg::DegreesToRadians( lat ) );
    double y = 0.5 - log( (1+sin_lat) / (1-sin_lat) ) / (4*osg::PI);

    double map_size = (double)getMapSize( lod );
    
    double raw_x = x * map_size + 0.5;
    double raw_y = y * map_size + 0.5;

    double clamp_x = clamp(raw_x, 0, map_size - 1);
    double clamp_y = clamp(raw_y, 0, map_size - 1);

    out_x = (unsigned int)clamp_x;
    out_y = (unsigned int)clamp_y;

    return raw_y < clamp_y? -1 : raw_y > clamp_y ? 1 : 0;
    //return raw_x == clamp_x && raw_y == clamp_y;
}

void
MercatorCellKey::pixelXYtoTileXY(unsigned int x, unsigned int y,
                                 unsigned int& out_tile_x,
                                 unsigned int& out_tile_y)
{
    out_tile_x = x/256;
    out_tile_y = y/256;
}

/********************************************************************/


struct MercatorTile
{
    MercatorTile() { }
    MercatorTile( MercatorCellKey& _key, MercatorTileSource* source ) : key( _key )
    {
        image = source->createImage( key );
        key.getPixelExtents( min_x, min_y, max_x, max_y );
        double dummy;
        key.getGeoExtents( dummy, min_lat, dummy, max_lat );
    }
    MercatorCellKey key;
    osg::ref_ptr<osg::Image> image;
    unsigned int min_x, min_y, max_x, max_y;
    double min_lat, max_lat;
};


/********************************************************************/


MercatorTileConverter::MercatorTileConverter( MercatorTileSource* _source )
{
    source = _source;
}

osg::Image*
MercatorTileConverter::createImage( const PlateCarreCellKey& pc_key )
{
    unsigned int lod = pc_key.getLevelOfDetail();

    double dst_min_lon, dst_min_lat, dst_max_lon, dst_max_lat;
    if ( !pc_key.getGeoExtents( dst_min_lon, dst_min_lat, dst_max_lon, dst_max_lat ) )
    {
        osg::notify( osg::WARN ) << "GET EXTENTS FAILED!" << std::endl;
        return NULL;
    }

    osg::Image* dst_tile = new osg::Image();
    dst_tile->allocateImage( 256, 256, 1, GL_RGB, GL_UNSIGNED_BYTE );

    // determine the mercator tiles that overlap the plate carre tile:
    // TODO: actually, no need to worry about X tiles, they will be identical
    unsigned int merc_pixel_min_x, merc_pixel_min_y;
    unsigned int merc_pixel_max_x, merc_pixel_max_y;
    MercatorCellKey::longLatToPixelXY( dst_min_lon, dst_min_lat, lod, merc_pixel_min_x, merc_pixel_max_y );
    MercatorCellKey::longLatToPixelXY( dst_max_lon, dst_max_lat, lod, merc_pixel_max_x, merc_pixel_min_y );

    // adjust:
    merc_pixel_max_x--;
    merc_pixel_max_y--;

    unsigned int merc_tile_min_x, merc_tile_min_y;
    MercatorCellKey::pixelXYtoTileXY( merc_pixel_min_x, merc_pixel_min_y, merc_tile_min_x, merc_tile_min_y );
    unsigned int merc_tile_max_x, merc_tile_max_y;
    MercatorCellKey::pixelXYtoTileXY( merc_pixel_max_x, merc_pixel_max_y, merc_tile_max_x, merc_tile_max_y );
    
    // loop through all the overlapping mercator tiles, fetch each one, and copy a portion of it
    // into the destination plate carre tile image. (NOTE: tiles will only overlap in the Y 
    // direction; X size will always be equal)
    std::vector<MercatorTile> src_tiles;
    for( unsigned int tile_y = merc_tile_min_y; tile_y <= merc_tile_max_y; tile_y++ )
    {
        MercatorCellKey mqk( merc_tile_min_x, tile_y, lod );
        MercatorTile src_tile( mqk, source.get() );
        if ( src_tile.image.valid() )
        {
            src_tiles.push_back( src_tile );
        }
    }

    double dst_lat = dst_max_lat;
    double dst_lat_interval = (dst_max_lat-dst_min_lat)/(double)dst_tile->t();
    for( unsigned int dst_row = 0; dst_row < (unsigned int)dst_tile->t(); dst_row++, dst_lat -= dst_lat_interval )
    {
        bool copied_line = false;

        // find the src tile and row for dst_lat
        for( unsigned int j=0; j<src_tiles.size(); j++ )
        {
            MercatorTile& src_tile = src_tiles[j];

            if ( dst_lat >= src_tile.min_lat && dst_lat <= src_tile.max_lat )
            {
                unsigned int src_x, src_y;
                src_tile.key.longLatToPixelXY( dst_min_lon, dst_lat, lod, src_x, src_y );

                unsigned int src_row = src_y - src_tile.min_y;

                if ( src_row < 0 ) src_row = 0;
                else if ( src_row > src_tile.image->t()-1 ) src_row = src_tile.image->t()-1;

                memcpy(
                    dst_tile->data( 0, (dst_tile->t()-1)-dst_row ),
                    src_tile.image->data( 0, (src_tile.image->t()-1)-src_row ), 
                    src_tile.image->getRowSizeInBytes() );

                break;
            }
        }
    }

    return dst_tile;
}


osg::HeightField*
MercatorTileConverter::createHeightField( const PlateCarreCellKey& pc_key )
{
    //TODO
    return NULL;
}