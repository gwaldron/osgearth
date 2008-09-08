#include <osgEarth/PlateCarre>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <sstream>
#include <stdlib.h>

using namespace osgEarth;

class WCSSource : public PlateCarreTileSource
{
public:
    WCSSource()
    {
        prefix     = "http://192.168.0.101/cgi-bin/mapserv?map=/var/www/maps/srtm.map";
        coverage   = "srtm";
        cov_format = "GEOTIFFINT16"; //FLOAT32";
        osg_format = "tif";
    }

public:
    osg::HeightField* createHeightField( const PlateCarreQuadKey& key )
    {
        std::string uri = createURI( key );
        
        // tif reader doesn't support height fields, so read as image and convert
        osg::HeightField* field = NULL;
        osg::ref_ptr<osg::Image> image = osgDB::readImageFile( uri );
        if ( image.valid() )
        {
            //osg::notify(osg::NOTICE) << "IMAGE: pixelbytes=" << image->getPixelSizeInBits() << ", pixelformat=" << image->getPixelFormat() << std::endl;
            field = new osg::HeightField();
            field->allocate( image->s(), image->t() );
            
            float vert_exag = 1.0f; //4.0f;

            for( unsigned int col=0; col<image->s(); col++ ) {
                for( unsigned int row=0; row <image->t(); row++ ) {
                    unsigned char* ptr = image->data( col, row );
                    if ( image->getPixelSizeInBits() == 16 ) {
                        short val = (short)*(short*)ptr;
                        field->setHeight( col, row, vert_exag * (float)val );
                    }
                    else if ( image->getPixelSizeInBits() == 32 ) {
                        float val = (float)*(float*)ptr;
                        field->setHeight( col, row, vert_exag * val );
                    }
                }
            }
            //osg::notify(osg::NOTICE) << std::endl;
        }

        //osg::HeightField* field = osgDB::readHeightFieldFile( uri );
        //if ( !field )
        //{
        //    osg::notify(osg::WARN) << "Failed to load height field from " << uri << std::endl;
        //}
        return field;
    }

    osg::Image* createImage( const PlateCarreQuadKey& key )
    {
        return NULL;
    }

    std::string createURI( const PlateCarreQuadKey& key ) const
    {
        double lon_min, lat_min, lon_max, lat_max;
        key.getGeoExtents( lon_min, lat_min, lon_max, lat_max );

        // http://192.168.0.101/cgi-bin/mapserv?map=/var/www/maps/srtm.map&SERVICE=WCS&VERSION=1.0.0&
        //  REQUEST=GetCoverage&COVERAGE=srtm&crs=epsg:4326&width=16&height=16&format=GEOTIFFFLOAT32


    //http://192.168.0.101/cgi-bin/mapserv?
    // map=/var/www/maps/srtm.map&SERVICE=WCS&VERSION=1.0.0&
    // REQUEST=GetCoverage&COVERAGE=srtm&crs=epsg:4326&
    // width=8&height=8&format=GEOTIFFINT16&BBOX=-180,-90,180,90

        // build the WCS request:
        std::stringstream buf;
        buf
            << prefix
            << "&SERVICE=WCS&VERSION=1.0.0&REQUEST=GetCoverage"
            << "&COVERAGE=" << coverage
            << "&FORMAT=" << cov_format
            << "&CRS=EPSG:4326"
            << "&WIDTH=16"
            << "&HEIGHT=16"
            << "&BBOX=" << lon_min << "," << lat_min << "," << lon_max << "," << lat_max;

        // add this to trick OSG into using the right image loader:
        buf << "&." << osg_format << ".curl";

        return buf.str();
    }

private:
    std::string prefix, coverage, cov_format, osg_format;
};


class ReaderWriterWCS : public osgDB::ReaderWriter
{
    public:
        ReaderWriterWCS() {}

        virtual const char* className()
        {
            return "WCS Reader";
        }
        
        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive( extension, "wcs" );
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* opt) const
        {
            return readNode( file_name, opt );
        }

        virtual ReadResult readNode(const std::string& file_name, const Options* opt) const
        {
            return ReadResult( "WCS: illegal usage (readNode); please use readImage/readHeightField" );
        }

        virtual ReadResult readImage(const std::string& file_name, const Options* opt) const
        {
            return ReadResult::FILE_NOT_HANDLED;
            //return ReadResult( "WCS: illegal usage (readImage); NOT YET IMPLEMENTED" );
        }

        virtual ReadResult readHeightField(const std::string& file_name, const Options* opt) const
        {            
            std::string ext = osgDB::getFileExtension( file_name );
            if ( !acceptsExtension( ext ) )
            {
                return ReadResult::FILE_NOT_HANDLED;
            }

            // extract the PC KEY from the filename:
            PlateCarreQuadKey key( file_name.substr( 0, file_name.find_first_of( '.' ) ) );
            
            osg::ref_ptr<PlateCarreTileSource> source = new WCSSource(); //TODO: config/cache it
            osg::HeightField* field = source->createHeightField( key );
            return field? ReadResult( field ) : ReadResult( "Unable to load WCS height field" );

            //TODO
            return NULL;

        }
};

REGISTER_OSGPLUGIN(wcs, ReaderWriterWCS)
