#include <osgEarth/TileSource>
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
#define PROPERTY_LAYER      "layer"
#define PROPERTY_FORMAT     "format"
#define PROPERTY_CACHE_PATH "cache_path"

class TileCacheSource : public TileSource
{
public:
    TileCacheSource( const osgDB::ReaderWriter::Options* _options ) :
      options( _options )
    {
        if ( options.valid() )
        {
            if ( options->getPluginData( PROPERTY_URL ) )
                url = std::string( (const char*)options->getPluginData( PROPERTY_URL ) );

            if ( options->getPluginData( PROPERTY_LAYER ) )
                layer = std::string( (const char*)options->getPluginData( PROPERTY_LAYER ) );

            if ( options->getPluginData( PROPERTY_FORMAT ) )
                format = std::string( (const char*)options->getPluginData( PROPERTY_FORMAT ) );

            if (options->getPluginData( PROPERTY_CACHE_PATH ))
                cache_path = std::string( (const char*)options->getPluginData( PROPERTY_CACHE_PATH ) );
        }
    }

    osg::Image* createImage( const TileKey* key )
    {
        unsigned int level, tile_x, tile_y;
        level = key->getLevelOfDetail();
        key->getTileXY( tile_x, tile_y );

        // need to invert the y-tile index
        tile_y = key->getMapSizeTiles() - tile_y - 1;

        char buf[2048];
        sprintf( buf, "%s/%s/%02d/%03d/%03d/%03d/%03d/%03d/%03d.%s%s",
            url.c_str(),
            layer.c_str(),
            level,
            (tile_x / 1000000),
            (tile_x / 1000) % 1000,
            (tile_x % 1000),
            (tile_y / 1000000),
            (tile_y / 1000) % 1000,
            (tile_y % 1000),
            format.c_str(),
            ( osgDB::containsServerAddress( url )? ".curl" : "" ) );

        osgEarth::FileCache fc( cache_path );
        return fc.readImageFile( buf, options.get() );
    }

    osg::HeightField* createHeightField( const TileKey* key )
    {
        //TODO
        osg::notify(osg::NOTICE) << "ReaderWriterTileCache::createHeightField() not yet implemented." << std::endl;
        return NULL;
    }

private:
    osg::ref_ptr<const osgDB::ReaderWriter::Options> options;
    std::string url;
    std::string layer;
    std::string format;
    std::string cache_path;
};

// Reads tiles from a TileCache disk cache.
class ReaderWriterTileCache : public osgDB::ReaderWriter
{
    public:
        ReaderWriterTileCache() {}

        virtual const char* className()
        {
            return "TileCache disk cache ReaderWriter";
        }
        
        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive( extension, "tilecache" );
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            return readNode( file_name, options );
        }

        virtual ReadResult readImage(const std::string& file_name, const Options* options) const
        {
            std::string ext = osgDB::getFileExtension( file_name );
            if ( !acceptsExtension( ext ) )
            {
                return ReadResult::FILE_NOT_HANDLED;
            }

            osg::ref_ptr<TileKey> key = TileKeyFactory::createFromName(
                file_name.substr( 0, file_name.find_first_of( '.' ) ) );

            osg::ref_ptr<TileCacheSource> source = new TileCacheSource( options );
            osg::Image* image = source->createImage( key.get() );

            return image? ReadResult( image ) : ReadResult( "Unable to load TileCache tile" );
        }

        virtual ReadResult readHeightField(const std::string& file_name, const Options* opt) const
        {
            return ReadResult::FILE_NOT_HANDLED;
            //NYI
        }

        virtual ReadResult readNode(const std::string& file_name, const Options* opt) const
        {
            return ReadResult::FILE_NOT_HANDLED;
            //NYI
        }
};

REGISTER_OSGPLUGIN(tilecache, ReaderWriterTileCache)
