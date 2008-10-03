#include <osgEarth/PlateCarre>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>
#include <osgDB/ReadFile>
#include <sstream>

using namespace osgEarth;

const std::string PlateCarreTileKey::TYPE_CODE = "P";

PlateCarreTileKey::PlateCarreTileKey( const PlateCarreTileKey& rhs )
: TileKey( rhs )
{
    //NOP
}

PlateCarreTileKey::PlateCarreTileKey( const std::string& input )
: TileKey( input, TileGridProfile( -180, -270, 180, 90 ) ) // whole-earth
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

/*************************************************************************/

//ReaderWriterPlateCarreTileSource::ReaderWriterPlateCarreTileSource(const std::string& _extension,
//                                                                   const osgDB::ReaderWriter::Options* _options )
//: extension( _extension ),
//  options( _options )
//{
//}
//
//osg::Image*
//ReaderWriterPlateCarreTileSource::createImage( const PlateCarreTileKey* key )
//{
//    std::string uri = key->str() + "." + extension;
//    osg::Image* image = NULL;
//
//    image = osgDB::readImageFile( uri, options.get() );
//    if ( !image )
//    {
//        osg::notify(osg::WARN) << "ReaderWriterPlateCarreTileSource: osgDB::readImageFile FAILED for \"" << uri << "\"" << std::endl;
//    }
//    return image;
//}
//
//osg::HeightField*
//ReaderWriterPlateCarreTileSource::createHeightField( const PlateCarreTileKey* key )
//{
//    std::string uri = key->str() + "." + extension;
//    osg::HeightField* field = NULL;
//
//    field = osgDB::readHeightFieldFile( uri, options.get() );
//    if ( !field )
//    {
//        osg::notify(osg::WARN) << "ReaderWriterPlateCarreTileSource: osgDB::readHeightField FAILED for \"" << uri << "\"" << std::endl;
//    }
//    return field;
//}
