#include <osgEarth/PlateCarre>
#include <osgEarth/Mercator>
#include <osgEarth/FileCache>
#include <osgEarth/TileSource>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/ImageOptions>
#include <sstream>
#include <stdlib.h>

using namespace osgEarth;

#define PROPERTY_URL            "url"
#define PROPERTY_TILE_WIDTH     "tile_width"
#define PROPERTY_TILE_HEIGHT    "tile_height"
#define PROPERTY_IMAGE_WIDTH     "image_width"
#define PROPERTY_IMAGE_HEIGHT    "image_height"


class GDALTileSource : public TileSource
{
public:
    GDALTileSource(const osgDB::ReaderWriter::Options* options):
      tile_width(256),
      tile_height(256)
      {
          if ( options->getPluginData( PROPERTY_URL ) )
              url = std::string( (const char*)options->getPluginData( PROPERTY_URL ) );

          if ( options->getPluginData( PROPERTY_TILE_WIDTH ) )
              tile_width = as<int>( (const char*)options->getPluginData( PROPERTY_TILE_WIDTH ), 256 );

          if ( options->getPluginData( PROPERTY_TILE_HEIGHT ) )
              tile_height = as<int>( (const char*)options->getPluginData( PROPERTY_TILE_HEIGHT ), 256 );
    }

    osg::Image* createImage( const TileKey* key )
    {
        std::string tileFile = createTileFile(key);
        if (!tileFile.empty())
        {
            return osgDB::readImageFile(tileFile);
        }
        return NULL;
    }

    osg::HeightField* createHeightField( const TileKey* key )
    {
        std::string tileFile = createTileFile(key);
        if (!tileFile.empty())
        {
            return osgDB::readHeightFieldFile(tileFile);
        }
        return NULL;
    }

    std::string createTileFile(const TileKey* key)
    {
        double minx, miny, maxx, maxy;

        //Get the GeoExtents of the file
        if (!key->getGeoExtents(minx, miny, maxx, maxy))
        {
            osg::notify(osg::WARN) << "gdal_tiles getGeoExtents failed" << std::endl;
        }

        std::stringstream ss;

        //Use gdalwarp to create an appropriate tile for this TileKey's bounds from the source file
        char temp_file[128];
        //Create a temp file for the VRT
        if (getenv("TEMP") != NULL)
        {
            strcpy(temp_file, getenv("TEMP"));
        }
        else
        {
            strcpy(temp_file, "./");
        }
        strcat(temp_file, "\\");
        strcat(temp_file, key->str().c_str());
        strcat(temp_file, ".vrt");

        std::string dest_srs = "EPSG:4326";

        ss << "gdalwarp -t_srs " << dest_srs << " -te " << minx << " " << miny << " " << maxx << " " << maxy
            << " -ts " << tile_width << " " << tile_height
            << " \"" << url << "\"  \"" << temp_file << "\"";

        //Remove file if it already exists
        if (osgDB::fileExists(temp_file))
        {
            if (remove(temp_file) != 0)
            {
                osg::notify(osg::FATAL) << "Error deleting file " << temp_file << std::endl;
                return "";
            }
        }

        //osg::notify(osg::NOTICE) << "Command is " << ss.str() << std::endl;

        if (system(ss.str().c_str()))
        {
            return "";
        }
        else
        {
            //Reset the stringstream
            ss.str("");
            //Force OSG into using the .gdal plugin
            ss << temp_file << ".gdal";
            return ss.str();
        }

        return "";
    }

private:
    std::string url;
    int tile_width;
    int tile_height;
};


class ReaderWriterGDALTile : public osgDB::ReaderWriter
{
    public:
        ReaderWriterGDALTile() {}

        virtual const char* className()
        {
            return "GDAL Tile Reader";
        }
        
        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive( extension, "gdal_tile" );
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* opt) const
        {
            return readNode( file_name, opt );
        }

        virtual ReadResult readNode(const std::string& file_name, const Options* options ) const
        {
            return ReadResult::FILE_NOT_HANDLED;
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

            osg::ref_ptr<GDALTileSource> tileSource = new GDALTileSource(options);
            return tileSource->createImage(key.get());
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

            osg::ref_ptr<GDALTileSource> tileSource = new GDALTileSource(options);
            return tileSource->createHeightField(key.get());
        }
};

REGISTER_OSGPLUGIN(gdal_tile, ReaderWriterGDALTile)
