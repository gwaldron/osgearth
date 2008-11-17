#include <osgEarth/MapConfig>
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
#define PROPERTY_MAP_CONFIG "map_config"

class YahooSource : public MercatorTileSource
{
public:
    YahooSource( const osgDB::ReaderWriter::Options* _options ) :
      options( _options ),
      map_config(0)
    {
        if ( options.valid() )
        {
            //if ( options->getPluginData( PROPERTY_URL ) )
            //    url = std::string( (const char*)options->getPluginData( PROPERTY_URL ) );

            if ( options->getPluginData( PROPERTY_DATASET ) )
                dataset = std::string( (const char*)options->getPluginData( PROPERTY_DATASET ) );

            if (options->getPluginData( PROPERTY_MAP_CONFIG ))
                map_config = (const MapConfig*)options->getPluginData( PROPERTY_MAP_CONFIG);
        }

        // validate dataset
        if ( dataset.empty() ) dataset = "roads"; // defaul to the satellite view
    }

    osg::Image* createImage( const TileKey* key )
    {
        const MercatorTileKey* mkey = static_cast<const MercatorTileKey*>( key );

        std::stringstream buf;
        
        if ( dataset == "roads" || dataset == "map" )
        {            
            // http://us.maps1.yimg.com/us.tile.maps.yimg.com/tl?v=4.1&md=2&x=0&y=0&z=2&r=1
            unsigned int tile_x, tile_y;
            mkey->getTileXY( tile_x, tile_y );
            int size = mkey->getMapSizeTiles();
            int zoom = key->getLevelOfDetail();

            buf << "http://us.maps1.yimg.com/us.tile.maps.yimg.com/tl"
                << "?v=4.1&md=2&r=1"
                << "&x=" << (int)tile_x
                << "&y=" << (size-1-(int)tile_y) - size/2
                << "&z=" << zoom + 1
                << "&.jpg.curl";
        }
        else if ( dataset == "aerial" || dataset == "satellite" )
        {
            unsigned int tile_x, tile_y;
            mkey->getTileXY( tile_x, tile_y );
            int size = mkey->getMapSizeTiles();
            int zoom = key->getLevelOfDetail();

            buf << "http://us.maps3.yimg.com/aerial.maps.yimg.com/ximg"
                << "?v=1.8&s=256&t=a&r=1"
                << "&x=" << (int)tile_x
                << "&y=" << (size-1-(int)tile_y) - size/2
                << "&z=" << zoom + 1
                << "&.jpg.curl";
        }

        //osg::notify(osg::NOTICE) << buf.str() << std::endl;

        std::string cache_path = map_config ? map_config->getFullCachePath() : std::string("");
        bool offline = map_config ? map_config->getOfflineHint() : false;

        osgEarth::FileCache fc(cache_path);
        fc.setOffline(offline);
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

    const MapConfig* map_config;
};


class ReaderWriterYahoo : public osgDB::ReaderWriter
{
    public:
        ReaderWriterYahoo()
        {
            supportsExtension( "yahoo", "Yahoo maps data" );
        }

        virtual const char* className()
        {
            return "Yahoo Imagery ReaderWriter";
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            return readNode( file_name, options );
        }

        virtual ReadResult readImage(const std::string& file_name, const Options* options) const
        {
            if ( osgDB::getLowerCaseFileExtension( file_name ) != "yahoo" )
                return ReadResult::FILE_NOT_HANDLED;

            osg::ref_ptr<TileKey> key = TileKeyFactory::createFromName(
                file_name.substr( 0, file_name.find_first_of( '.' ) ) );

            osg::Image* image = NULL;

            osg::ref_ptr<MercatorTileSource> source = new YahooSource( options );
            if ( dynamic_cast<PlateCarreTileKey*>( key.get() ) )
            {
                MercatorTileConverter converter( source.get(), options );
                image = converter.createImage( static_cast<PlateCarreTileKey*>( key.get() ) );
            }
            else
            {
                image = source->createImage( key.get() );
            }

            return image? ReadResult( image ) : ReadResult( "Unable to load Yahoo tile" );
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

REGISTER_OSGPLUGIN(yahoo, ReaderWriterYahoo)

