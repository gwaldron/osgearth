#include <osgEarth/Mercator>
#include <osgEarth/FileCache>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <sstream>

using namespace osgEarth;

#define PROPERTY_URL        "url"
#define PROPERTY_DATASET    "dataset"
#define PROPERTY_CACHE_PATH "cache_path"

class GoogleSource : public MercatorTileSource
{
public:
    GoogleSource( const osgDB::ReaderWriter::Options* _options ) :
      options( _options )
    {
        if ( options.valid() )
        {
            //if ( options->getPluginData( PROPERTY_URL ) )
            //    url = std::string( (const char*)options->getPluginData( PROPERTY_URL ) );

            if ( options->getPluginData( PROPERTY_DATASET ) )
                dataset = std::string( (const char*)options->getPluginData( PROPERTY_DATASET ) );

            if (options->getPluginData( PROPERTY_CACHE_PATH ))
                cache_path = std::string( (const char*)options->getPluginData( PROPERTY_CACHE_PATH ) );
        }

        // validate dataset
        if ( dataset.empty() ) dataset = "satellite"; // defaul to the satellite view
    }

    osg::Image* createImage( const TileKey* key )
    {
        const MercatorTileKey* mkey = static_cast<const MercatorTileKey*>( key );

        std::stringstream buf;
        
        if ( dataset == "satellite" )
        {            
            char server = key->str().length() > 0? key->str()[key->str().length()-1] : '0';
            unsigned int tile_x, tile_y;
            mkey->getTileXY( tile_x, tile_y );
            int zoom = key->getLevelOfDetail();

            buf << "http://khm" << server << ".google.com/kh/v=32&hl=en"
                << "&x=" << tile_x
                << "&y=" << tile_y
                << "&z=" << zoom
                << "&s=Ga&.jpg.curl";
            
            // http://khm0.google.com/kh/v=32&hl=en&x=4&y=6&z=4&s=Ga
        }
        else if ( dataset == "terrain" )
        {
            char server = key->str().length() > 0? key->str()[key->str().length()-1] : '0';
            unsigned int tile_x, tile_y;
            mkey->getTileXY( tile_x, tile_y );
            buf << "http://mt" << server << ".google.com/mt?v=app.81&hl=en&x="
                << tile_x << "&y=" << tile_y << "&zoom=" 
                << 17-key->getLevelOfDetail() << "&.jpg.curl";
        }
        else if ( dataset == "labels" )
        {
            char server = key->str().length() > 0? key->str()[key->str().length()-1] : '0';
            unsigned int tile_x, tile_y;
            mkey->getTileXY( tile_x, tile_y );
            int zoom = key->getLevelOfDetail();

            buf << "http://mt" << server << ".google.com/mt/v=w2t.83&hl=en"
                << "&x=" << tile_x
                << "&y=" << tile_y
                << "&z=" << zoom
                << "&s=Ga&.png.curl";

            //http://mt3.google.com/mt/v=w2t.83&hl=en&x=3&y=6&z=4&s=Galileo
        }
        else if ( dataset == "roads" )
        {
            char server = key->str().length() > 0? key->str()[key->str().length()-1] : '0';
            unsigned int tile_x, tile_y;
            mkey->getTileXY( tile_x, tile_y );
            int zoom = key->getLevelOfDetail();

            buf << "http://mt" << server << ".google.com/mt?v=w2.86&hl=en"
                << "&x=" << tile_x
                << "&y=" << tile_y
                << "&z=" << zoom
                << "&s=Ga&.png.curl";
        }

        osg::notify(osg::INFO) 
            << "[osgEarth] Google: option string = "
            << (options.valid()? options->getOptionString() : "<empty>")
            << std::endl;

        osgEarth::FileCache fc(cache_path);
        return fc.readImageFile( buf.str(), options.get() );
    }

    osg::HeightField* createHeightField( const TileKey* key )
    {
        //TODO
        return NULL;
    }

private:
    osg::ref_ptr<const osgDB::ReaderWriter::Options> options;
    //std::string url;
    std::string dataset;
    std::string cache_path;
};


class ReaderWriterGoogle : public osgDB::ReaderWriter
{
    public:
        ReaderWriterGoogle()
        {
            supportsExtension( "google", "Google maps imagery" );
        }

        virtual const char* className()
        {
            return "Google Imagery ReaderWriter";
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            return readNode( file_name, options );
        }

        virtual ReadResult readImage(const std::string& file_name, const Options* options) const
        {
            if ( osgDB::getLowerCaseFileExtension( file_name ) != "google" )
                return ReadResult::FILE_NOT_HANDLED;

            osg::ref_ptr<TileKey> key = TileKeyFactory::createFromName(
                file_name.substr( 0, file_name.find_first_of( '.' ) ) );

            osg::Image* image = NULL;

            osg::ref_ptr<MercatorTileSource> source = new GoogleSource( options );
            if ( dynamic_cast<PlateCarreTileKey*>( key.get() ) )
            {
                MercatorTileConverter converter( source.get(), options );
                image = converter.createImage( static_cast<PlateCarreTileKey*>( key.get() ) );
            }
            else
            {
                image = source->createImage( key.get() );
            }

            return image? ReadResult( image ) : ReadResult( "Unable to load Google tile" );
        }

        virtual ReadResult readHeightField(const std::string& file_name, const Options* opt) const
        {
            return ReadResult::FILE_NOT_HANDLED;
            //NYI
        }

        virtual ReadResult readNode(const std::string& file_name, const Options* opt) const
        {
            return ReadResult::FILE_NOT_HANDLED;
        }
};

REGISTER_OSGPLUGIN(google, ReaderWriterGoogle)
