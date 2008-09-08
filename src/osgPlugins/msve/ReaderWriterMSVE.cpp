#include <osgEarth/Mercator>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>

using namespace osgEarth;

class MSVESource : public MercatorTileSource
{
public:
    MSVESource() { }

    osg::Image* createImage( const MercatorQuadKey& key )
    {
        std::string q = key.str();
        // a=aerial(jpg), r=map(png), h=hybrid(jpg), t=elev(wmphoto?)
        char buf[128];

        //sprintf( buf, "http://192.168.0.8/gis/denverglobe/%s.jpg.curl",
        //    q.c_str() );

        //sprintf( buf, "http://192.168.0.8/gis/denver-source/SpatialData/wgs84/virtualearth/jpg/%s.jpg.curl",
        //    q.c_str() );

        // MSVE roads
        //sprintf( buf, "http://h%c.ortho.tiles.virtualearth.net/tiles/r%s?g=1&.png.curl",
        //    q[q.length()-1], q.c_str() );

        // MSVE hybrid
        sprintf( buf, "http://h%c.ortho.tiles.virtualearth.net/tiles/h%s?g=1&.jpg.curl",
            q[q.length()-1], q.c_str() );

        // Google "terrain" view: (max zoom=17)
        //char server = key.str().length() > 0? key.str()[key.str().length()-1] : '0';
        //unsigned int tile_x, tile_y;
        //key.getTileXY( tile_x, tile_y );
        //sprintf( buf, "http://mt%c.google.com/mt?v=app.81&hl=en&x=%d&y=%d&zoom=%d&.jpg.curl",
        //    server, tile_x, tile_y, 17-key.getLevelOfDetail() );

        // Google "satellite" view
        //std::string gkey = key.str();
        //for( int i=0; i<gkey.length(); i++ )
        //    gkey[i] = "qrts"[(int)(gkey[i]-'0')];
        //sprintf( buf, "http://kh0.google.com/kh?n=404&v=8&t=t%s&.jpg.curl",
        //    gkey.c_str() );        


        //osg::notify(osg::NOTICE) << "Fetch: " << buf << std::endl;

        return osgDB::readImageFile( buf );
    }

    osg::HeightField* createHeightField( const MercatorQuadKey& key )
    {
        //TODO
        return NULL;
    }
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

            // extract the PC KEY from the filename:
            PlateCarreQuadKey key( file_name.substr( 0, file_name.find_first_of( '.' ) ) );

            osg::ref_ptr<MercatorTileSource> source = new MSVESource(); //TODO: config/cache it
            MercatorTileConverter converter( source.get() );
            osg::Image* image = converter.createImage( key );
            return image? ReadResult( image ) : ReadResult( "Unable to load MSVE tile" );
        }

        virtual ReadResult readHeightField(const std::string& file_name, const Options* opt) const
        {
            //TODO
            return ReadResult( "MSVE: readHeightField NYI" );
        }

        virtual ReadResult readNode(const std::string& file_name, const Options* opt) const
        {
            //TODO (none)
            return ReadResult( "MSVE: readNode NYI" );
        }
};

REGISTER_OSGPLUGIN(msve, ReaderWriterMSVE)
