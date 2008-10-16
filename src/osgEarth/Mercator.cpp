#include <osgEarth/Mercator>
#include <osgEarth/PlateCarre>
#include <osg/Math>
#include <osg/Notify>
#include <sstream>
#include <algorithm>

using namespace osgEarth;

#define PROPERTY_FILTER     "filter"
#define VALUE_FILTER_NEAREST      "nearest"
#define VALUE_FILTER_LINEAR       "linear"

/********************************************************************/

#define MIN_LON -180.0
#define MAX_LON  180.0
#define MIN_LAT  -85.05112878  
#define MAX_LAT   85.05112878

#define MIN_X -20037508.342789244
#define MIN_Y -20037508.342789244
#define MAX_X 20037508.342789244
#define MAX_Y 20037508.342789244

#define PIXELS_PER_TILE 256

const std::string MercatorTileKey::TYPE_CODE = "M";

MercatorTileKey::MercatorTileKey( const MercatorTileKey& rhs ) :
TileKey( rhs )
{
}

MercatorTileKey::MercatorTileKey( const std::string& key ) :
TileKey( key, TileGridProfile( MIN_LON, MIN_LAT, MAX_LON, MAX_LAT, PIXELS_PER_TILE ) )
{
}

MercatorTileKey::MercatorTileKey( const std::string& input, const TileGridProfile& profile ) :
TileKey( input, profile )
{
}

MercatorTileKey::MercatorTileKey( unsigned int tile_x, unsigned int tile_y, unsigned int lod )
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

    key = ss.str();
    profile = TileGridProfile( MIN_LON, MIN_LAT, MAX_LON, MAX_LAT, PIXELS_PER_TILE );
}

TileKey*
MercatorTileKey::getSubkey( unsigned int quadrant ) const
{
    if ( !subkeys[quadrant].valid() )
        const_cast<MercatorTileKey*>(this)->subkeys[quadrant] = new MercatorTileKey( key + (char)('0'+quadrant), profile );
    return subkeys[quadrant].get();
}

unsigned int
MercatorTileKey::getLevelOfDetail() const
{
    return (unsigned int)key.length();
}

void
MercatorTileKey::getTileXY(unsigned int& out_tile_x,
                           unsigned int& out_tile_y) const
{
    unsigned int xmin, ymin, xmax, ymax;
    getPixelExtents( xmin, ymin, xmax, ymax );
    out_tile_x = xmin/profile.pixelsPerTile();
    out_tile_y = ymin/profile.pixelsPerTile();
}

void
MercatorTileKey::getPixelExtents(unsigned int& xmin,
                                 unsigned int& ymin,
                                 unsigned int& xmax,
                                 unsigned int& ymax) const
{
    unsigned int lod = getLevelOfDetail();
    unsigned int px = 0, py = 0;
    unsigned int delta = getMapSizePixels( lod ) >> 1;
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
MercatorTileKey::getMeterExtents(double &xmin,
                                 double &ymin,
                                 double &xmax,
                                 double &ymax) const
{
    unsigned int x,y;
    getTileXY(x,y);

    double tile_width = MAX_X - MIN_X;
    double tile_height = MAX_Y - MIN_Y;

    //Determine the width and height of a tile in meters at the given lod
    for (unsigned int lod = 0; lod < getLevelOfDetail(); ++lod)
    {
        tile_width /= 2.0;
        tile_height /= 2.0;
    }

    //Get the total number of tiles in each dimension
    int totalTiles = sqrt(pow(4.0, (double)getLevelOfDetail()));

    xmin = MIN_X + (x * tile_width);
    xmax = xmin + tile_width;

    ymin = MIN_Y + ((totalTiles - y - 1) * tile_height);
    ymax = ymin + tile_height;  
}

osgTerrain::TileID
MercatorTileKey::getTileId() const
{
    unsigned int x, y;
    getTileXY(x, y);
    return osgTerrain::TileID(getLevelOfDetail(), x, y);
}

bool
MercatorTileKey::getGeoExtents(double& lon_min, double& lat_min,
                               double& lon_max, double& lat_max) const
{
    unsigned int xmin, ymin, xmax, ymax;
    getPixelExtents( xmin, ymin, xmax, ymax );
    lat_min = getLatitude( ymax );
    lat_max = getLatitude( ymin );

    double width =  profile.xMax() - profile.xMin();
    lon_min = profile.xMin();

    for( unsigned int lod = 0; lod < getLevelOfDetail(); lod++ )
    {
        width /= 2.0;
        char digit = key[lod];
        switch( digit )
        {
            case '1':
            case '3': lon_min += width; break;
        }
    }

    lon_max = lon_min + width;
    return true;
}

double
MercatorTileKey::getLatitude( unsigned int pixel_y ) const
{
    unsigned int lod = getLevelOfDetail();
    double my  = -osg::PI + (1.0 - (double)pixel_y/(double)getMapSizePixels(lod)) * (2.0*osg::PI);
    double deg = osg::RadiansToDegrees( 2.0 * atan( exp( my ) ) - .5*osg::PI );

    return deg;
}

static double
clamp( double n, double min_value, double max_value )
{
    return osg::minimum( osg::maximum( n, min_value ), max_value );
}

int
MercatorTileKey::longLatToPixelXY(double lon, double lat, unsigned int lod, int tile_size,
                                  unsigned int& out_x, unsigned int& out_y )
{
    double x = (lon + 180.0) / 360.0;
    double sin_lat = sin( osg::DegreesToRadians( lat ) );
    double y = 0.5 - log( (1+sin_lat) / (1-sin_lat) ) / (4*osg::PI);

    double map_size = (double)getMapSizePixels( tile_size, lod );
    
    double raw_x = x * map_size + 0.5;
    double raw_y = y * map_size + 0.5;

    double clamp_x = clamp(raw_x, 0, map_size - 1);
    double clamp_y = clamp(raw_y, 0, map_size - 1);

    out_x = (unsigned int)clamp_x;
    out_y = (unsigned int)clamp_y;

    return raw_y < clamp_y? -1 : raw_y > clamp_y ? 1 : 0;
    //return raw_x == clamp_x && raw_y == clamp_y;
}

//void
//MercatorTileKey::longLatToUnclampedPixelXY(double lon, double lat, unsigned int lod, 
//                                           double& out_x, double& out_y )
//{
//    double x = (lon + 180.0) / 360.0;
//    double sin_lat = sin( osg::DegreesToRadians( lat ) );
//    double y = 0.5 - log( (1+sin_lat) / (1-sin_lat) ) / (4*osg::PI);
//
//    double map_size = (double)getMapSizePixels( lod );
//    
//    out_x = clamp( x * map_size + 0.5, 0, map_size-1 );
//    out_y = clamp( y * map_size + 0.5, 0, map_size-1 );
//}

void
MercatorTileKey::pixelXYtoTileXY(unsigned int x, unsigned int y,
                                 int tile_size,
                                 unsigned int& out_tile_x,
                                 unsigned int& out_tile_y)
{
    out_tile_x = x/tile_size;
    out_tile_y = y/tile_size;
}

/********************************************************************/


struct MercatorTile
{
    MercatorTile() { }
    MercatorTile( MercatorTileKey* _key, MercatorTileSource* source ) : key( _key )
    {
        image = source->createImage( key.get() );
        key->getPixelExtents( min_x, min_y, max_x, max_y );
        double dummy;
        key->getGeoExtents( dummy, min_lat, dummy, max_lat );
    }
    osg::ref_ptr<MercatorTileKey> key;
    osg::ref_ptr<osg::Image> image;
    unsigned int min_x, min_y, max_x, max_y;
    double min_lat, max_lat;
};


struct MercatorSuperTile : public std::vector<MercatorTile>
{
    MercatorSuperTile(int _tile_size) : tile_size(_tile_size) { }

    int getRow( double lon, double lat, unsigned int lod ) {
        unsigned int x,y;
        if ( lat < (*this)[0].min_lat ) {
            return 0;
        }
        for( int i=0; i<(int)size(); i++ ) {
            if ( lat >= (*this)[i].min_lat && lat <= (*this)[i].max_lat ) {
                (*this)[i].key->longLatToPixelXY( lon, lat, lod, tile_size, x, y );
                //osg::notify(osg::NOTICE) << "i=" << i << ", y=" << y << std::endl;
                return i*tile_size + tile_size-(y-(*this)[i].min_y);
            }
        }
        return t()-1;
    }

    unsigned char* data( int s, int t ) {
        return (*this)[t/tile_size].image->data( 0, t%tile_size );
    }

    int t(){ 
        return size()*tile_size;
    }
    double min_lat() { return (*this)[0].min_lat; }
    double max_lat() { return (*this)[size()-1].max_lat; }

private:
    int tile_size;
};

static osg::Image*
sharpen( osg::Image* input )
{
    int filter[9] = { 0, -1, 0, -1, 5, -1, 0, -1, 0 };
    osg::Image* output = new osg::Image( *input );
    for( int t=1; t<input->t()-1; t++ )
    {
        for( int s=1; s<input->s()-1; s++ )
        {
            int pixels[9] = {
                *(int*)input->data(s-1,t-1), *(int*)input->data(s,t-1), *(int*)input->data(s+1,t-1),
                *(int*)input->data(s-1,t  ), *(int*)input->data(s,t  ), *(int*)input->data(s+1,t  ),
                *(int*)input->data(s-1,t+1), *(int*)input->data(s,t+1), *(int*)input->data(s+1,t+1) };

            int shifts[4] = { 0, 8, 16, 32 };

            for( int c=0; c<4; c++ ) // components
            {
                int mask = 0xff << shifts[c];
                int sum = 0;
                for( int i=0; i<9; i++ )
                {
                    sum += ((pixels[i] & mask) >> shifts[c]) * filter[i];
                }
                sum = sum > 255? 255 : sum < 0? 0 : sum;
                output->data(s,t)[c] = sum;
            }
        }
    }
    return output;
}

/********************************************************************/


MercatorTileConverter::MercatorTileConverter(MercatorTileSource* _source,
                                             const osgDB::ReaderWriter::Options* options )
{
    filter = MercatorTileConverter::FILTER_NEAREST_NEIGHBOR;
    source = _source;

    if ( options )
    {
        if ( options->getPluginData( PROPERTY_FILTER ) )
        {
            std::string filter = (const char*)options->getPluginData( PROPERTY_FILTER );
            if ( filter == VALUE_FILTER_NEAREST )
                setFilter( MercatorTileConverter::FILTER_NEAREST_NEIGHBOR );
            else if ( filter == VALUE_FILTER_LINEAR )
                setFilter( MercatorTileConverter::FILTER_LINEAR );
        }
    }
}

void
MercatorTileConverter::setFilter( const MercatorTileConverter::Filter& _filter )
{
    filter = _filter;
}

osg::Image*
MercatorTileConverter::createImage( const PlateCarreTileKey* pc_key )
{
    unsigned int lod = pc_key->getLevelOfDetail();

    double dst_min_lon, dst_min_lat, dst_max_lon, dst_max_lat;
    if ( !pc_key->getGeoExtents( dst_min_lon, dst_min_lat, dst_max_lon, dst_max_lat ) )
    {
        osg::notify( osg::WARN ) << "GET EXTENTS FAILED!" << std::endl;
        return NULL;
    }

    int tile_size = pc_key->getProfile().pixelsPerTile();
    osg::ref_ptr<osg::Image> dst_tile = new osg::Image();
    dst_tile->allocateImage( tile_size, tile_size, 1, GL_RGB, GL_UNSIGNED_BYTE );

    // determine the mercator tiles that overlap the plate carre tile:
    // TODO: actually, no need to worry about X tiles, they will be identical
    unsigned int merc_pixel_min_x, merc_pixel_min_y;
    unsigned int merc_pixel_max_x, merc_pixel_max_y;
    MercatorTileKey::longLatToPixelXY( dst_min_lon, dst_min_lat, lod, tile_size, merc_pixel_min_x, merc_pixel_max_y );
    MercatorTileKey::longLatToPixelXY( dst_max_lon, dst_max_lat, lod, tile_size, merc_pixel_max_x, merc_pixel_min_y );

    // adjust:
    merc_pixel_max_x--;
    merc_pixel_max_y--;

    unsigned int merc_tile_min_x, merc_tile_min_y;
    MercatorTileKey::pixelXYtoTileXY( merc_pixel_min_x, merc_pixel_min_y, tile_size, merc_tile_min_x, merc_tile_min_y );
    unsigned int merc_tile_max_x, merc_tile_max_y;
    MercatorTileKey::pixelXYtoTileXY( merc_pixel_max_x, merc_pixel_max_y, tile_size, merc_tile_max_x, merc_tile_max_y );
    
    // collect all the mercator tiles that overlap the destination plate carre tile:
    MercatorSuperTile supertile( tile_size );
    //osg::notify(osg::NOTICE) << "Supertile: " << std::endl;
    for( unsigned int tile_y = merc_tile_min_y; tile_y <= merc_tile_max_y; tile_y++ )
    {
        //MercatorCellKey mqk( merc_tile_min_x, tile_y, lod );
        MercatorTile src_tile( new MercatorTileKey( merc_tile_min_x, tile_y, lod ), source.get() );
        if ( src_tile.image.valid() )
        {
            supertile.insert( supertile.begin(), src_tile );
            //osg::notify(osg::NOTICE) << "   tile: min_lat=" << src_tile.min_lat << ", max_lat=" << src_tile.max_lat << std::endl;
        }
    }

    if ( supertile.size() > 0 )
    {
        if ( filter == MercatorTileConverter::FILTER_LINEAR )
        {
            double dst_lat = dst_max_lat;
            double dst_lat_interval = (dst_max_lat-dst_min_lat)/(double)dst_tile->t();

            for( int dst_row = dst_tile->t()-1; dst_row >= 0; dst_row--, dst_lat -= dst_lat_interval )
            {
                int src_row_hi = supertile.getRow( dst_min_lon, dst_lat, lod );
                int src_row_lo = supertile.getRow( dst_min_lon, dst_lat-dst_lat_interval, lod );

                if ( src_row_hi < 0 || src_row_lo < 0 )
                    continue;

                if ( src_row_hi > supertile.t()-1 ) src_row_hi = supertile.t()-1;
                if ( src_row_lo < 0 ) src_row_lo = 0;

                for( int b = 0; b < (int)dst_tile->getRowSizeInBytes(); b++ )
                {
                    int total = 0, r;
                    for( r = src_row_lo; r <= src_row_hi; r++ )
                    {
                        total += (int)(supertile.data( 0, r )[b]);
                    }
                    total /= (src_row_hi-src_row_lo+1);
                    dst_tile->data( 0, dst_row )[b] = (unsigned char)total;
                }
            }
        }

        else if ( filter == MercatorTileConverter::FILTER_NEAREST_NEIGHBOR )
        {
            // loop through all the overlapping mercator tiles, fetch each one, and copy a portion of it
            // into the destination plate carre tile image. (NOTE: tiles will only overlap in the Y 
            // direction; X size will always be equal)
            double dst_lat = dst_max_lat;
            double dst_lat_interval = (dst_max_lat-dst_min_lat)/(double)dst_tile->t();
            for( int dst_row = dst_tile->t()-1; dst_row >= 0; dst_row--, dst_lat -= dst_lat_interval )
            {
                bool copied_line = false;

                // find the src tile and row for dst_lat
                int src_row = supertile.getRow( dst_min_lon, dst_lat, lod );
                 //unsigned int src_row = src_y - src_tile.min_y;

                if ( src_row < 0 ) src_row = 0;
                else if ( src_row > supertile.t()-1 ) src_row = supertile.t()-1;

                memcpy(
                    dst_tile->data( 0, dst_row ),
                    supertile.data( 0, src_row ),
                    dst_tile->getRowSizeInBytes() );
            }
        }
    }

    else
    {
        //TODO: deal with the case of no overlapping tiles
    }

    return dst_tile.release();
}


osg::HeightField*
MercatorTileConverter::createHeightField( const PlateCarreTileKey* pc_key )
{
    //TODO
    return NULL;
}


/*********************************************************************/


MercatorLocator::MercatorLocator( const osgTerrain::Locator& rhs, int _tile_size, unsigned int _lod ) :
osgTerrain::Locator(rhs), tile_size( _tile_size ), lod(_lod)
{
    //NOP
}

MercatorLocator::MercatorLocator( int _tile_size, unsigned int _lod ) :
tile_size(_tile_size), lod( _lod )
{
    //NOP
}

bool
MercatorLocator::convertLocalToModel(const osg::Vec3d& local, osg::Vec3d& world) const
{
    switch(_coordinateSystemType)
    {
        case(GEOCENTRIC):
        {
            return Locator::convertLocalToModel( local, world );
        }
        case(GEOGRAPHIC):
        {        
            return Locator::convertLocalToModel( local, world );
            //world = local * _transform;

            //double lon_deg = world.x();
            //double lat_deg = world.y();
            //unsigned int px, py;
            //MercatorTileKey::longLatToPixelXY( lon_deg, lat_deg, lod, px, py );
            //if ( py % tile_size != 0 )
            //{
            //    py %= tile_size;
            //    world.y() = ((double)tile_size-(double)py)/(double)tile_size;
            //    //osg::notify(osg::NOTICE) << "lat="<<lat_deg<<", lon="<<lon_deg<<", py="<<py<<", worldy="<<world.y()<<std::endl;
            //}
            return true;      
        }
        case(PROJECTED):
        {        
            return Locator::convertLocalToModel( local, world );
        }
    }    

    return false;
}

bool
MercatorLocator::convertModelToLocal(const osg::Vec3d& world, osg::Vec3d& local) const
{
    // OSG 2.7 bug workaround: bug fix in Locator submitted by GW on 10/3/2008:
    const_cast<MercatorLocator*>(this)->_inverse.invert( _transform );

    switch(_coordinateSystemType)
    {
    case(GEOCENTRIC):
        {        
            double longitude, latitude, height;

            _ellipsoidModel->convertXYZToLatLongHeight(world.x(), world.y(), world.z(),
                latitude, longitude, height );

            local = osg::Vec3d(longitude, latitude, height) * _inverse;

            double lon_deg = osg::RadiansToDegrees(longitude);
            double lat_deg = osg::RadiansToDegrees(latitude);
            unsigned int px, py;
            MercatorTileKey::longLatToPixelXY( lon_deg, lat_deg, lod, tile_size, px, py );
            if ( py % tile_size != 0 )
            {
                py %= tile_size;
                local.y() = ((double)tile_size-(double)py)/(double)tile_size;
                //osg::notify(osg::NOTICE) << "KEY="<<key->str()<<": lat="<<lat_deg<<", lon="<<lon_deg<<", py="<<py<<", localy="<<local.y()<<std::endl;
            }

            return true;      
        }
    case(GEOGRAPHIC):
        {        
            local = world * _inverse;

            osg::Vec3d w = world;
            double lon_deg = w.x();
            double lat_deg = w.y();

            unsigned int px, py;
            MercatorTileKey::longLatToPixelXY( lon_deg, lat_deg, lod, tile_size, px, py );
            if ( py % tile_size != 0 )
            {
                py %= tile_size;
                local.y() = ((double)tile_size-(double)py)/(double)tile_size;
               // osg::notify(osg::NOTICE) << "lod="<<lod<<", lat="<<lat_deg<<", lon="<<lon_deg<<", py="<<py<<", local.y="<<local.y()<<std::endl;
            }

            return true;      
        }
    case(PROJECTED):
        {        
            local = world * _inverse;
            return true;      
        }
    }    

    return false;
}
