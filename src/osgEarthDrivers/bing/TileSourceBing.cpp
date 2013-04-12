#include <osgEarth/TileSource>
#include <osgEarth/Registry>
#include <osgEarth/URI>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <ctype.h>

#include <sstream>

using namespace osgEarth;

#define LC "[Bing] "

#define PROPERTY_URL        "url"
#define PROPERTY_DATASET    "dataset"


class BingTileSource : public TileSource
{
public:
    BingTileSource(const TileSourceOptions& options) : 
      TileSource(options)
    {
        //nop
    }

    Status initialize(const osgDB::Options* dbOptions)
    {
        _dbOptions = Registry::instance()->cloneOrCreateOptions(dbOptions);
        CachePolicy::NO_CACHE.apply( _dbOptions.get() );

        dataset = "a";

        // validate dataset
        if ( dataset == "hybrid" ) dataset = "h";
        else if ( dataset == "roads" ) dataset = "r";
        else if ( dataset == "aerial" || dataset == "satellite" ) dataset = "a";
        else if ( dataset == "terrain") dataset = "t";
        if ( dataset.empty() ) dataset = "h"; // default to the "hybrid" dataset (imagery+labels)

        // validate/default URL
        if ( url.empty() )
            url = "http://ecn.t0.tiles.virtualearth.net/tiles";

        
        const Profile* profile = Profile::create(
            SpatialReference::get("spherical-mercator"),
            MERC_MINX, MERC_MINY, MERC_MAXX, MERC_MAXY,
            2, 2 );

        setProfile( profile );

        return STATUS_OK;
    }

      static std::string getQuadKey(const TileKey& key)
      {
        unsigned int tile_x, tile_y;
        key.getTileXY(tile_x, tile_y);
        unsigned int lod = key.getLevelOfDetail();

        std::stringstream ss;
        for( unsigned i = (int)lod+1; i > 0; i-- )
        {
          char digit = '0';
          unsigned mask = 1 << (i-1);
          if ( (tile_x & mask) != 0 )
          {
            digit++;
          }
          if ( (tile_y & mask) != 0 )
          {
            digit += 2;
          }
          ss << digit;
        }
        return ss.str();
      }

      char getRandomServer()
      {
        //Gets a server from 0 - 7
        int server = rand() % 8;
        char serverChar[2];
        sprintf(serverChar, "%i", server);
        return serverChar[0];
      }


    osg::Image* createImage( const TileKey& key, ProgressCallback* progress )
    {
        //Return NULL if we are given a non-mercator key
        //if ( !key->isMercator() ) return 0;


        // a=aerial(jpg), r=map(png), h=hybrid(jpg), t=elev(wmphoto?)

        std::stringstream buf;

        std::string myurl = url;

#if 0
        //Round Robin the server number [0,3]
        if (osgDB::containsServerAddress(url))
        {
          //Assume URL is in the form http://[type][server].tiles.virtualearth.net/tiles/[type][location].[format]?g=45       
          if (url.size() >=9 && isdigit(myurl[8]))
          {
            char serverChar = getRandomServer();
            myurl[8] = serverChar;
          }
        }
#endif

        std::string quadKey = getQuadKey(key);

        buf << myurl << "/" << dataset << quadKey << ".jpeg";

        //Only add the ?g=1 if we are connecting to a server address
        if (osgDB::containsServerAddress(url)) buf << "?g=1236&";
        buf << "." << getExtension();

        std::string request = Stringify()
            << myurl
            << "/"
            << dataset
            << quadKey
            << ".jpeg"
            << "?g=1236";


        OE_DEBUG << "Loading MSVE tile " << key.str() << " from " << request << std::endl;

        //osg::Image* image = osgDB::readImageFile( buf.str() );

        //OE_INFO
        //    << "[osgEarth] MSVE: option string = "
        //    << (getOptions()? getOptions()->getOptionString() : "<empty>")
        //    << std::endl;

        osg::ref_ptr<osg::Image> image = URI(request).getImage( _dbOptions.get(), progress );

        /*if (!image)
        {
            //HACK:  The "no image" image that is returned from MSVE will have a JPG extension but it is really a
            //       PNG.  Try loading it as a PNG instead
            osg::notify(osg::INFO) << "Trying to read as a PNG " << std::endl;
            std::stringstream os;
            os << osgDB::getNameLessExtension(buf.str()) << ".png";

            image = osgDB::readImageFile(os.str(), options.get());

            if (image.valid())
            {
                osg::notify(osg::INFO) << "Read as a PNG successfully" << std::endl;
                osgDB::writeImageFile(*image, "bad_image.png");
            }
        }*/

        if (isBadImage(image.get())) return NULL;
        return image.release();
    }

    std::string getExtension() const
    {
        if (dataset == "h" || dataset == "a")
        {
            return "jpg";
        }
        else if (dataset == "t")
        {
            return "tif";
        }
        else
        {
            return "png";
        }
    }

    /*
     * Determines if the given image is the "bad" image that MSVE uses to denote no imagery in an area
     */
    static bool isBadImage(osg::Image* image)
    {
        if (image)
        {
            static bool tried_to_load_bad_image = false;
            if (!tried_to_load_bad_image)
            {
                if (!bad_image.valid())
                {
                    bad_image = osgDB::readImageFile("msve_bad_image.png");
                    tried_to_load_bad_image = true;
                    if (!bad_image.valid()) osg::notify(osg::WARN) << "Could not load msve_bad_image.png" << std::endl;
                }
            }

            if (bad_image.valid())
            {
                if ((image->s() == bad_image->s()) && (image->t() == bad_image->t()))
                {
                    for (int c = 0; c < image->s(); ++c)
                    {
                        for (int r = 0; r < image->t(); ++r)
                        {
                            if (*image->data(c, r) != *bad_image->data(c, r))
                            {
                                return false;
                            }
                        }
                    }
                    osg::notify(osg::NOTICE) << "Detected bad image" << std::endl;
                    return true;
                }
            }
        }
        return false;
    }        

private:
    std::string url;
    std::string dataset;
    static osg::ref_ptr<osg::Image> bad_image;
    osg::ref_ptr<osgDB::Options> _dbOptions;
};

osg::ref_ptr<osg::Image> BingTileSource::bad_image;



class BingTileSourceDriver : public TileSourceDriver
{
public:
    BingTileSourceDriver()
    {
        supportsExtension( "osgearth_bing", "Microsoft Bing Driver" );
    }

    virtual const char* className()
    {
        return "Microsoft Bing Driver";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new BingTileSource( getTileSourceOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_bing, BingTileSourceDriver)
