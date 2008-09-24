#include <osgEarth/PlateCarre>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <sstream>
#include <stdlib.h>

using namespace osgEarth;

class WMSSource : public PlateCarreTileSource
{
public:
    WMSSource()
    {
        std::string host = "192.168.0.101";
        if ( ::getenv( "OSGEARTH_HOST" ) )
            host = std::string( ::getenv( "OSGEARTH_HOST" ) );

        std::stringstream buf;
        buf << "http://" << host << "/tilecache-2.04/tilecache.py";
        prefix     = buf.str();

        //prefix = "http://192.168.0.101/tilecache-2.04/tilecache.py";
        layers = "bluemarble2002-dc";
        format = "png";

        //prefix = "http://labs.metacarta.com/wms-c/Basic.py";
        //layers = "basic"; format = "png";
        //layers = "satellite"; format = "jpg";
    }

public:
    osg::Image* createImage( const PlateCarreCellKey& key )
    {
        std::string uri = createURI( key );
        osg::Image* image = osgDB::readImageFile( uri );
        if ( !image )
        {
            osg::notify(osg::WARN) << "Failed to load image from " << uri << std::endl;
        }
        return image;
    }

    osg::HeightField* createHeightField( const PlateCarreCellKey& key )
    {
        return NULL;
    }

    std::string createURI( const PlateCarreCellKey& key ) const
    {
        double lon_min, lat_min, lon_max, lat_max;
        key.getGeoExtents( lon_min, lat_min, lon_max, lat_max );

        // http://labs.metacarta.com/wms-c/Basic.py?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap&LAYERS=basic&BBOX=-180,-90,0,90

        // build the WMS request:
        std::stringstream buf;
        buf
            << prefix
            << "?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap"
            << "&LAYERS=" << layers
            << "&FORMAT=image/" << format
            << "&STYLES="
            << "&SRS=EPSG:4326"
            << "&WIDTH=256"
            << "&HEIGHT=256"
            << "&BBOX=" << lon_min << "," << lat_min << "," << lon_max << "," << lat_max;

        // add this to trick OSG into using the right image loader:
        buf << "&." << format << ".curl";

        return buf.str();
    }

private:
    std::string prefix;
    std::string layers;
    std::string format;
};


class ReaderWriterWMS : public osgDB::ReaderWriter
{
    public:
        ReaderWriterWMS() {}

        virtual const char* className()
        {
            return "WMS/WMS-C Reader";
        }
        
        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive( extension, "wms" );
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* opt) const
        {
            return readNode( file_name, opt );
        }

        virtual ReadResult readNode(const std::string& file_name, const Options* opt) const
        {
            return ReadResult( "WMS: illegal usage (readNode); please use readImage/readHeightField" );
        }

        virtual ReadResult readImage(const std::string& file_name, const Options* opt) const
        {
            std::string ext = osgDB::getFileExtension( file_name );
            if ( !acceptsExtension( ext ) )
            {
                return ReadResult::FILE_NOT_HANDLED;
            }

            // extract the PC KEY from the filename:
            PlateCarreCellKey key( file_name.substr( 0, file_name.find_first_of( '.' ) ) );
            
            osg::ref_ptr<PlateCarreTileSource> source = new WMSSource(); //TODO: config/cache it
            osg::Image* image = source->createImage( key );
            return image? ReadResult( image ) : ReadResult( "Unable to load WMS tile" );

            //TODO
            return NULL;
        }

        virtual ReadResult readHeightField(const std::string& file_name, const Options* opt) const
        {
            return ReadResult::FILE_NOT_HANDLED;
        }
};

REGISTER_OSGPLUGIN(wms, ReaderWriterWMS)
