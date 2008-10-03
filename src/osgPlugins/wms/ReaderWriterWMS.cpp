#include <osgEarth/PlateCarre>
#include <osgEarth/FileCache>
#include <osgEarth/TileSource>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <sstream>
#include <stdlib.h>

using namespace osgEarth;

#define PROPERTY_URL         "url"
#define PROPERTY_LAYERS      "layers"
#define PROPERTY_STYLE       "style"
#define PROPERTY_FORMAT      "format"
#define PROPERTY_TILE_WIDTH  "tile_width"
#define PROPERTY_TILE_HEIGHT "tile_height"
#define PROPERTY_CACHE_PATH  "cache_path"


osg::HeightField* imageToHeightField(osg::Image* image)
{
	osg::HeightField *hf = new osg::HeightField;
	if (!image)
	{
		osg::notify(osg::WARN) << "Image null, returning empty heightfield" << std::endl;
		hf->allocate( 8, 8 );
        for(unsigned int i=0; i<hf->getHeightList().size(); i++ )
            hf->getHeightList()[i] = 0;//(double)((::rand() % 10000) - 5000);
	}
	else
	{
		//osg::notify(osg::NOTICE) << "Read heightfield image" << std::endl;
		hf->allocate(image->s(), image->t());
				
		for( unsigned int row=0; row < image->t(); row++ ) {
            //osg::notify(osg::NOTICE) << "ROW " << row << ":\t";
            for( unsigned int col=0; col < image->s(); col++ ) {
                unsigned char* ptr = image->data( col, row );
                if ( image->getPixelSizeInBits() == 16 ) {
                    short val = (short)*(short*)ptr;
                    hf->setHeight( col, row, (float)val );
                    //osg::notify(osg::NOTICE) << val << "\t";
                }
                else if ( image->getPixelSizeInBits() == 32 ) {
                    float val = (float)*(float*)ptr;
                    hf->setHeight( col, row, val );
					//osg::notify(osg::NOTICE) << val << "\t";
                }
            }
            //osg::notify(osg::NOTICE) << std::endl;
        }
	}
	return hf;
}

class WMSSource : public TileSource
{
public:
	WMSSource( const osgDB::ReaderWriter::Options* options ):
	  tile_width(256),
	  tile_height(256)
    {
        if ( options->getPluginData( PROPERTY_URL ) )
            prefix = std::string( (const char*)options->getPluginData( PROPERTY_URL ) );

        if ( options->getPluginData( PROPERTY_LAYERS ) )
            layers = std::string( (const char*)options->getPluginData( PROPERTY_LAYERS ) );

        if ( options->getPluginData( PROPERTY_STYLE ) )
            style = std::string( (const char*)options->getPluginData( PROPERTY_STYLE ) );

        if ( options->getPluginData( PROPERTY_FORMAT ) )
            format = std::string( (const char*)options->getPluginData( PROPERTY_FORMAT ) );

        if ( options->getPluginData( PROPERTY_CACHE_PATH))
             cache_path = std::string( (const char*)options->getPluginData( PROPERTY_CACHE_PATH ) );

		if ( options->getPluginData( PROPERTY_TILE_WIDTH ) )
            tile_width = as<int>( (const char*)options->getPluginData( PROPERTY_TILE_WIDTH ), 256 );

        if ( options->getPluginData( PROPERTY_TILE_HEIGHT ) )
            tile_height = as<int>( (const char*)options->getPluginData( PROPERTY_TILE_HEIGHT ), 256 );

        if ( format.empty() )
            format = "png";

        //prefix = "http://192.168.0.101/tilecache-2.04/tilecache.py";
        //layers = "bluemarble2002-dc";
        //format = "png";

        //prefix = "http://labs.metacarta.com/wms-c/Basic.py";
        //layers = "basic"; format = "png";
        //layers = "satellite"; format = "jpg";
    }

public:
    osg::Image* createImage( const TileKey* key )
    {
        std::string uri = createURI( key );


        //osg::Image* image = osgDB::readImageFile( uri );
        osgEarth::FileCache fc(cache_path);
        osg::Image* image = fc.readImageFile(uri);

        if ( !image )
        {
            osg::notify(osg::WARN) << "Failed to load image from " << uri << std::endl;
        }
        return image;
    }

    osg::HeightField* createHeightField( const TileKey* key )
    {
        osg::Image* image = createImage(key);
        if (!image)
        {
            osg::notify(osg::WARN) << "Failed to read heightfield from " << createURI(key) << std::endl;
        }
        return (imageToHeightField(image));        
    }

    std::string createURI( const TileKey* key ) const
    {
        double lon_min, lat_min, lon_max, lat_max;
        key->getGeoExtents( lon_min, lat_min, lon_max, lat_max );

        // http://labs.metacarta.com/wms-c/Basic.py?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap&LAYERS=basic&BBOX=-180,-90,0,90

        // build the WMS request:
        std::stringstream buf;
        buf
            << prefix
            << "?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap"
            << "&LAYERS=" << layers
            << "&FORMAT=image/" << format
            << "&STYLES=" << style
            << "&SRS=EPSG:4326"
            << "&WIDTH="<< tile_width
            << "&HEIGHT="<< tile_height
            << "&BBOX=" << lon_min << "," << lat_min << "," << lon_max << "," << lat_max;

        // add this to trick OSG into using the right image loader:
        buf << "&." << format;// << ".curl";

        return buf.str();
    }

private:
    std::string prefix;
    std::string layers;
    std::string style;
    std::string format;

	int tile_width;
	int tile_height;

    std::string cache_path;
};


class ReaderWriterWMS : public osgDB::ReaderWriter
{
    public:
        ReaderWriterWMS() {}

        virtual const char* className()
        {
            return "WMS Reader";
        }
        
        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive( extension, "wms" );
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* opt) const
        {
            return readNode( file_name, opt );
        }

        virtual ReadResult readNode(const std::string& file_name, const Options* options ) const
        {
            return ReadResult::FILE_NOT_HANDLED;
            //return ReadResult( "WMS: illegal usage (readNode); please use readImage/readHeightField" );
        }

        virtual ReadResult readImage(const std::string& file_name, const Options* options ) const
        {
            std::string ext = osgDB::getFileExtension( file_name );
            if ( !acceptsExtension( ext ) )
            {
                return ReadResult::FILE_NOT_HANDLED;
            }

            std::string keystr = file_name.substr( 0, file_name.find_first_of( '.' ) );
            osg::ref_ptr<TileKey> key = TileKeyFactory::createFromName( keystr );
            
            //TODO: support mercator keys?
            if ( dynamic_cast<PlateCarreTileKey*>(key.get()) )
            {
                osg::ref_ptr<TileSource> source = new WMSSource( options ); //TODO: config/cache it
                osg::Image* image = source->createImage( key.get() );
                return image? ReadResult( image ) : ReadResult( "Unable to load WMS tile" );
            }
            else
            {
                return ReadResult::FILE_NOT_HANDLED;
            }
        }

        virtual ReadResult readHeightField(const std::string& file_name, const Options* options) const
        {
            std::string ext = osgDB::getFileExtension( file_name );
            if ( !acceptsExtension( ext ) )
            {
                return ReadResult::FILE_NOT_HANDLED;
            }

            std::string keystr = file_name.substr( 0, file_name.find_first_of( '.' ) );
            osg::ref_ptr<TileKey> key = TileKeyFactory::createFromName( keystr );
            
            //TODO: support mercator keys?
            if ( dynamic_cast<PlateCarreTileKey*>(key.get()) )
            {
                osg::ref_ptr<TileSource> source = new WMSSource( options ); //TODO: config/cache it
                osg::HeightField* heightField = source->createHeightField( key.get() );
                return heightField? ReadResult( heightField ) : ReadResult( "Unable to load WMS tile" );
            }
            else
            {
                return ReadResult::FILE_NOT_HANDLED;
            }
        }
};

REGISTER_OSGPLUGIN(wms, ReaderWriterWMS)
