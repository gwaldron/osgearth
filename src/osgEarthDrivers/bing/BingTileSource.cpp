#include "BingOptions"

#include <osgEarth/TileSource>
#include <osgEarth/Registry>
#include <osgEarth/URI>
#include <osgEarth/StringUtils>
#include <osgEarth/Random>

#include <osgDB/FileNameUtils>

using namespace osgEarth;

#define LC "[Bing] "


class BingTileSource : public TileSource
{
private:
    osgEarth::Drivers::BingOptions _options;
    osg::ref_ptr<osgDB::Options>   _dbOptions;
    Random                         _prng;
    bool                           _debugDirect;

public:
    /**
     * Constructs the tile source
     */
    BingTileSource(const TileSourceOptions& options) : 
      TileSource  ( options ),
      _options    ( options ),
      _debugDirect( false )
    {
        if ( ::getenv("OSGEARTH_BING_DIRECT") )
            _debugDirect = true;
    }

    /**
     * One-tile tile source initialization.
     */
    Status initialize(const osgDB::Options* dbOptions)
    {
        // Always apply the NO CACHE policy.
        _dbOptions = Registry::instance()->cloneOrCreateOptions(dbOptions);
        CachePolicy::NO_CACHE.apply( _dbOptions.get() );

        // If the user did not include an API key, fail.
        if ( !_options.key().isSet() )
        {
            return Status::Error("Bing API key is required");
        }

        // If the user did not specify an imagery set, default to aerial.
        if ( !_options.imagerySet().isSet() )
        {
            _options.imagerySet() = "Aerial";
        }

        // Bing maps profile is spherical mercator with 2x2 tiles are the root.
        const Profile* profile = Profile::create(
            SpatialReference::get("spherical-mercator"),
            MERC_MINX, MERC_MINY, MERC_MAXX, MERC_MAXY,
            2, 2);
        
        setProfile( profile );

        return STATUS_OK;
    }
    
    /**
     * Tell the terrain engine not to cache tiles form this source.
     */
    CachePolicy getCachePolicyHint() const
    {
        return CachePolicy::NO_CACHE;
    }


    /**
     * Create and return an image for the given TileKey.
     */
    osg::Image* createImage( const TileKey& key, ProgressCallback* progress )
    {
        if (_debugDirect)
        {
            return URI(getDirectURI(key)).getImage(_dbOptions.get(), progress);
        }

        // center point of the tile (will be in spherical mercator)
        double x, y;
        key.getExtent().getCentroid(x, y);

        // transform it to lat/long:
        GeoPoint geo;

        GeoPoint( getProfile()->getSRS(), x, y ).transform(
            getProfile()->getSRS()->getGeographicSRS(),
            geo );

        // contact the REST API. Docs are here:
        // http://msdn.microsoft.com/en-us/library/ff701716.aspx

        // construct the request URI:
        std::string request = Stringify()
            << std::setprecision(12)
            << _options.imageryMetadataAPI().get()     // base REST API
            << "/"    << _options.imagerySet().get()   // imagery set to use
            << "/"    << geo.y() << "," << geo.x()     // center point in lat/long
            << "?zl=" << key.getLOD() + 1              // zoom level
            << "&o=json"                               // response format
            << "&key=" << _options.key().get();        // API key

        // fetch it:
        ReadResult metadataResult = URI(request).readString(_dbOptions, progress);

        if ( metadataResult.failed() )
        {
            // check for a REST error:
            if ( metadataResult.code() == ReadResult::RESULT_SERVER_ERROR )
            {
                OE_WARN << LC << "REST API request error!" << std::endl;

                Config metadata;
                std::string content = metadataResult.getString();
                metadata.fromJSON( content );
                ConfigSet errors = metadata.child("errorDetails").children();
                for(ConfigSet::const_iterator i = errors.begin(); i != errors.end(); ++i )
                {
                    OE_WARN << LC << "REST API: " << i->value() << std::endl;
                }
                return 0L;
            }
            else
            {
                OE_WARN << LC << "Request error: " << metadataResult.getResultCodeString() << std::endl;
            }
            return 0L;
        }

        // decode it:
        Config metadata;
        if ( !metadata.fromJSON(metadataResult.getString()) )
        {
            OE_WARN << LC << "Error decoding REST API response" << std::endl;
            return 0L;
        }

        // check the vintage field. If it's empty, that means we got a "no data" tile.
        Config* vintageEnd = metadata.find("vintageEnd");
        if ( !vintageEnd || vintageEnd->value().empty() )
        {
            OE_DEBUG << LC << "NO data image encountered." << std::endl;
            return 0L;
        }

        // find the tile URI:
        Config* location = metadata.find("imageUrl");
        if ( !location )
        {
            OE_WARN << LC << "REST API JSON parsing error (imageUrl not found)" << std::endl;
            return 0L;
        }

        // request the actual tile
        //OE_INFO << "key = " << key.str() << ", URL = " << location->value() << std::endl;

        osg::Image* image = URI(location->value()).getImage(_dbOptions.get(), progress);
        return image;
    }

private:

    std::string getQuadKey(const TileKey& key)
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

    std::string getDirectURI(const TileKey& key)
    {
        return Stringify()
            << "http://ecn.t"
            << _prng.next(4)
            << ".tiles.virtualearth.net/tiles/h"
            << getQuadKey(key)
            << ".jpeg?g=1236";
    }
};


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
