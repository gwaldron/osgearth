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

class MSVESource : public MercatorTileSource
{
public:
    MSVESource( const osgDB::ReaderWriter::Options* _options ) :
      options( _options )
    {
        if ( options.valid() )
        {
            if ( options->getPluginData( PROPERTY_URL ) )
                url = std::string( (const char*)options->getPluginData( PROPERTY_URL ) );

            if ( options->getPluginData( PROPERTY_DATASET ) )
                dataset = std::string( (const char*)options->getPluginData( PROPERTY_DATASET ) );

            if (options->getPluginData( PROPERTY_CACHE_PATH ))
                cache_path = std::string( (const char*)options->getPluginData( PROPERTY_CACHE_PATH ) );
        }

        // validate dataset
        if ( dataset == "hybrid" ) dataset = "h";
        else if ( dataset == "roads" ) dataset = "r";
        else if ( dataset == "aerial" || dataset == "satellite" ) dataset = "a";
        if ( dataset.empty() ) dataset = "h"; // default to the "hybrid" dataset (imagery+labels)

        // validate/default URL
        if ( url.empty() )
            url = "http://h0.ortho.tiles.virtualearth.net/tiles";
    }

    osg::Image* createImage( const TileKey* key )
    {
        // a=aerial(jpg), r=map(png), h=hybrid(jpg), t=elev(wmphoto?)

        std::stringstream buf;

        std::string format;

        buf << url << "/" << dataset << key->str() << "?g=1&";
        buf << "." << getFormat();

        //osg::Image* image = osgDB::readImageFile( buf.str() );

        osg::notify(osg::INFO) 
            << "[osgEarth] MSVE: option string = "
            << (options.valid()? options->getOptionString() : "<empty>")
            << std::endl;

        osgEarth::FileCache fc(cache_path);
        return fc.readImageFile( buf.str(), options.get() );
    }

    std::string getFormat() const
    {
        if (dataset == "h" || dataset == "a")
        {
            return "jpg";
        }
        else
        {
            return "png";
        }
    }

    //    //sprintf( buf, "http://192.168.0.8/gis/denverglobe/%s.jpg.curl",
    //    //    q.c_str() );

    //    //sprintf( buf, "http://192.168.0.8/gis/denver-source/SpatialData/wgs84/virtualearth/jpg/%s.jpg.curl",
    //    //    q.c_str() );

    //    // MSVE roads
    //    //sprintf( buf, "http://h%c.ortho.tiles.virtualearth.net/tiles/r%s?g=1&.png.curl",
    //    //    q[q.length()-1], q.c_str() );

    //    // MSVE hybrid
    //    sprintf( buf, "http://h%c.ortho.tiles.virtualearth.net/tiles/h%s?g=1&.jpg.curl",
    //        q[q.length()-1], q.c_str() );

    //    // Google "terrain" view: (max zoom=17)
    //    //char server = key.str().length() > 0? key.str()[key.str().length()-1] : '0';
    //    //unsigned int tile_x, tile_y;
    //    //key.getTileXY( tile_x, tile_y );
    //    //sprintf( buf, "http://mt%c.google.com/mt?v=app.81&hl=en&x=%d&y=%d&zoom=%d&.jpg.curl",
    //    //    server, tile_x, tile_y, 17-key.getLevelOfDetail() );

    //    // Google "satellite" view
    //    //std::string gkey = key.str();
    //    //for( int i=0; i<gkey.length(); i++ )
    //    //    gkey[i] = "qrts"[(int)(gkey[i]-'0')];
    //    //sprintf( buf, "http://kh0.google.com/kh?n=404&v=8&t=t%s&.jpg.curl",
    //    //    gkey.c_str() );        


    osg::HeightField* createHeightField( const TileKey* key )
    {
        //TODO
        return NULL;
    }

private:
    osg::ref_ptr<const osgDB::ReaderWriter::Options> options;
    std::string url;
    std::string dataset;
    std::string cache_path;
};


class ReaderWriterMSVE : public osgDB::ReaderWriter
{
    public:
        ReaderWriterMSVE() {}

        virtual const char* className()
        {
            return "MSVE Imagery ReaderWriter";
        }
        
        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive( extension, "msve" );
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

            osg::Image* image = NULL;

            osg::ref_ptr<MercatorTileSource> source = new MSVESource( options );
            if ( dynamic_cast<PlateCarreTileKey*>( key.get() ) )
            {
                MercatorTileConverter converter( source.get(), options );
                image = converter.createImage( static_cast<PlateCarreTileKey*>( key.get() ) );
            }
            else
            {
                image = source->createImage( key.get() );
            }

            return image? ReadResult( image ) : ReadResult( "Unable to load MSVE tile" );
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

REGISTER_OSGPLUGIN(msve, ReaderWriterMSVE)
