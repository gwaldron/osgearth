#include "BingOptions"

#include <osgEarth/TileSource>
#include <osgEarth/Registry>
#include <osgEarth/URI>
#include <osgEarth/StringUtils>
#include <osgEarth/Random>
#include <osgEarth/ImageUtils>
#include <osgEarth/Containers>

#include <osgEarthSymbology/Geometry>
#include <osgEarthSymbology/GeometryRasterizer>

#include <osgDB/FileNameUtils>
#include <osgText/Font>

#include <OpenThreads/Atomic>

using namespace osgEarth;
using namespace osgEarth::Symbology;

#define LC "[Bing] "

namespace
{
    struct AlphaBlend
    {
        bool operator()( const osg::Vec4f& src, osg::Vec4f& dest )
        {
            float sa = src.a();
            dest.set(
                dest.r()*(1.0f-sa) + src.r()*sa,
                dest.g()*(1.0f-sa) + src.g()*sa,
                dest.b()*(1.0f-sa) + src.b()*sa,
                dest.a() );
            return true;
        }
    };

    typedef LRUCache<std::string, std::string> TileURICache;
}


class BingTileSource : public TileSource, public osgEarth::Drivers::BingOptions
{
private:
    //osgEarth::Drivers::BingOptions _options;
    osg::ref_ptr<const osgDB::Options> _readOptions;
    Random                         _prng;
    bool                           _debugDirect;
    osg::ref_ptr<Geometry>         _geom;
    osg::ref_ptr<osgText::Font>    _font;
    TileURICache                   _tileURICache;
    OpenThreads::Atomic            _apiCount;

public:
    /**
     * Constructs the tile source
     */
    BingTileSource(const TileSourceOptions& options) : 
      TileSource   ( options ),
      BingOptions  ( options ),
      //_options     ( options ),
      _debugDirect ( false ),
      _tileURICache( true, 1024u )
    {
        if ( ::getenv("OSGEARTH_BING_DIRECT") )
            _debugDirect = true;
        
        if ( ::getenv("OSGEARTH_BING_DEBUG") )
        {
            _geom = new Ring();
            _geom->push_back( osg::Vec3(10, 10, 0) );
            _geom->push_back( osg::Vec3(245, 10, 0) );
            _geom->push_back( osg::Vec3(245, 245, 0) );
            _geom->push_back( osg::Vec3(10, 245, 0) );
            _font = Registry::instance()->getDefaultFont();
        }
    }

    /**
     * One-tile tile source initialization.
     */
    Status initialize(const osgDB::Options* dbOptions)
    {
        _readOptions = dbOptions;

        // If the user did not include an API key, fail.
        if ( !apiKey().isSet() )
        {
            return Status::Error(Status::ConfigurationError, "Bing API key is required");
        }

        // If the user did not specify an imagery set, default to aerial.
        if ( !imagerySet().isSet() )
        {
            imagerySet() = "Aerial";
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
     * Always apply the NO CACHE policy.  Bing doesn't allow caching of their data.
     */
    CachePolicy getCachePolicyHint(const Profile*) const
    {
        return CachePolicy::NO_CACHE;
    }


    /**
     * Create and return an image for the given TileKey.
     */
    osg::Image* createImage( const TileKey& key, ProgressCallback* progress )
    {
        osg::Image* image = 0L;

        if (_debugDirect)
        {
            image = URI(getDirectURI(key)).getImage(_readOptions.get(), progress);
        }

        else
        {
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
                << imageryMetadataAPI().get()     // base REST API
                << "/"    << imagerySet().get()   // imagery set to use
                << "/"    << geo.y() << "," << geo.x()     // center point in lat/long
                << "?zl=" << key.getLOD() + 1              // zoom level
                << "&o=json"                               // response format
                << "&key=" << apiKey().get();        // API key

            // check the URI cache.
            URI                  location;
            TileURICache::Record rec;

            if ( _tileURICache.get(request, rec) )
            {
                location = URI(rec.value());
                //CacheStats stats = _tileURICache.getStats();
                //OE_INFO << "Ratio = " << (stats._hitRatio*100) << "%" << std::endl;
            }
            else
            {
                unsigned c = ++_apiCount;
                if ( c % 25 == 0 )
                    OE_DEBUG << LC << "API calls = " << c << std::endl;
            
                // fetch it:
                ReadResult metadataResult = URI(request).readString(_readOptions, progress);

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
                Config* locationConf= metadata.find("imageUrl");
                if ( !locationConf )
                {
                    OE_WARN << LC << "REST API JSON parsing error (imageUrl not found)" << std::endl;
                    return 0L;
                }

                location = URI( locationConf->value() );
                _tileURICache.insert( request, location.full() );
            }

            // request the actual tile
            //OE_INFO << "key = " << key.str() << ", URL = " << location->value() << std::endl;
            image = osgDB::readImageFile( location.full() );
        }

        if ( image &&  _geom.valid() )
        {
            GeometryRasterizer rasterizer( image->s(), image->t() );
            rasterizer.draw( _geom.get(), osg::Vec4(1,1,1,1) );
            osg::ref_ptr<osg::Image> overlay = rasterizer.finalize();
            ImageUtils::PixelVisitor<AlphaBlend> blend;
            blend.accept( overlay.get(), image );
        }

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

    virtual const char* className() const
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
