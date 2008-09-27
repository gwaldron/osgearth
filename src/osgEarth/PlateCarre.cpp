#include <osgEarth/PlateCarre>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>
#include <osgDB/ReadFile>
#include <sstream>

using namespace osgEarth;

//#define MIN_LON    -180.0
//#define MAX_LON     180.0
//#define MIN_LAT    -270.0
//#define MAX_LAT      90.0

PlateCarreCellKey::PlateCarreCellKey( const PlateCarreCellKey& rhs )
: qk( rhs.qk ),
  profile( rhs.profile )
{
    //NOP
}

PlateCarreCellKey::PlateCarreCellKey( const std::string& input )
: qk( input ),
  profile( TileGridProfile( -180, -270, 180, 90 ) ) // whole-earth
{
    //NOP
}

PlateCarreCellKey::PlateCarreCellKey( const std::string& input, const TileGridProfile& _profile )
: qk( input ),
  profile( _profile )
{
    //NOP
}

PlateCarreCellKey
PlateCarreCellKey::getSubkey( unsigned int quadrant ) const
{
    return PlateCarreCellKey( qk + (char)('0' + quadrant) );
}

const std::string&
PlateCarreCellKey::str() const
{
    return qk;
}

unsigned int
PlateCarreCellKey::getLevelOfDetail() const
{
    return (unsigned int)qk.length();
}

bool
PlateCarreCellKey::getGeoExtents(double& out_min_lon,
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

        char digit = qk[lod];
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

ReaderWriterPlateCarreTileSource::ReaderWriterPlateCarreTileSource(const std::string& _extension,
                                                                   const osgDB::ReaderWriter::Options* _options )
: extension( _extension ),
  options( _options )
{
    //source_uri_suffix = osgDB::getNameLessExtension( uri_suffix );
}

osg::Image*
ReaderWriterPlateCarreTileSource::createImage( const PlateCarreCellKey& key )
{
    std::string uri = key.str() + "." + extension; //source_uri_suffix;
    osg::Image* image = NULL;

    image = osgDB::readImageFile( uri, options.get() );
    if ( !image )
    {
        osg::notify(osg::WARN) << "ReaderWriterPlateCarreTileSource: osgDB::readImageFile FAILED for \"" << uri << "\"" << std::endl;
    }
    return image;
}

osg::HeightField*
ReaderWriterPlateCarreTileSource::createHeightField( const PlateCarreCellKey& key )
{
    std::string uri = key.str() + "." + extension;
    osg::HeightField* field = NULL;

    field = osgDB::readHeightFieldFile( uri, options.get() );
    if ( !field )
    {
        osg::notify(osg::WARN) << "ReaderWriterPlateCarreTileSource: osgDB::readHeightField FAILED for \"" << uri << "\"" << std::endl;
    }
    return field;
}

//std::string
//ReaderWriterPlateCarreTileSource::createURI( const PlateCarreCellKey& key ) const
//{
//    return key.str() + "." + uri_suffix;
//}