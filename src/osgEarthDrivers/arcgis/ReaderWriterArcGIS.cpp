/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include "MapService.h"
#include "ArcGISOptions"

#include <osgEarth/TileSource>
#include <osgEarth/Registry>
#include <osgEarth/URI>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <sstream>
#include <iomanip>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Drivers;

//#define PROPERTY_URL        "url"
//#define PROPERTY_PROFILE    "profile"

class ArcGISSource : public TileSource
{
public:
    ArcGISSource( const TileSourceOptions& options ) :
      TileSource( options ),
      _options( options ),
      _profileConf( ProfileOptions() )
    {
        //if ( options )
        //{
        //    const Config& conf = options->config();

        //    // this is the ArcGIS REST services URL for the map service,
        //    // e.g. http://server/ArcGIS/rest/services/Layer/MapServer
        //    _url = conf.value( PROPERTY_URL );

        //    // force a profile type
        //    // TODO? do we need this anymore? doesn't this happen with overrideprofile now?
        //    if ( conf.hasChild( PROPERTY_PROFILE ) )
        //        _profileConf = ProfileOptions( conf.child( PROPERTY_PROFILE ) );
        //}

        //TODO: allow single layers vs. "fused view"
        if ( _layer.empty() )
            _layer = "_alllayers"; // default to the AGS "fused view"

        //TODO: detect the format
        if ( _format.empty() )
            _format = "png";

        URI url = _options.url().value();
        //Add the token if necessary
        if (_options.token().isSet())
        {
            std::string token = _options.token().value();
            if (!token.empty())
            {
                std::string sep = url.full().find( "?" ) == std::string::npos ? "?" : "&";
                url = url.append( sep + std::string("token=") + token );
            }
        }

        // read metadata from the server
        if ( !_map_service.init( url.full() ) ) //, getOptions()) )
        {
            OE_WARN << "[osgearth] [ArcGIS] map service initialization failed: "
                << _map_service.getError() << std::endl;
        }
    }

    // override
    void initialize( const osgDB::Options* dbOptions, const Profile* overrideProfile)
    {
        _dbOptions = dbOptions;

        const Profile* profile = NULL;

        if ( _profileConf.isSet() )
        {
            profile = Profile::create( _profileConf.get() );
        }
        else if (overrideProfile)
        {
            profile = overrideProfile;
        }
        //if ( !_profile_str.empty() )
        //{
        //    profile = Profile::create( _profile_str );
        //}
        else if ( _map_service.getProfile() )
        {
            profile = _map_service.getProfile();

            /*
            if ( !_map_service.isTiled() )
            {
                // expand the profile's extents so they form a square.
                // AGS will return an image of a different extent than requested if the pixel aspect
                // ratio is not the same at the geoextent aspect ratio. By forcing a square full extent,
                // we can always request square tiles.

                const GeoExtent& oldEx = profile->getExtent();
                if ( oldEx.width() > oldEx.height() )
                {
                    double d = oldEx.width() - oldEx.height();
                    unsigned int tilesX, tilesY;
                    profile->getNumTiles( 0, tilesX, tilesY );
                    profile = Profile::create( profile->getSRS(), oldEx.xMin(), oldEx.yMin()-d/2, oldEx.xMax(), oldEx.yMax()+d/2, 0L, tilesX, tilesY );
                }
                else if ( oldEx.width() < oldEx.height() )
                {
                    double d = oldEx.height() - oldEx.width();
                    unsigned int tilesX, tilesY;
                    profile->getNumTiles( 0, tilesX, tilesY );
                    profile = Profile::create( profile->getSRS(), oldEx.xMin()-d/2, oldEx.yMin(), oldEx.xMax()+d/2, oldEx.yMax(), 0L, tilesX, tilesY );    
                }
            }
            */
        }        
        else
        {
            profile = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
        }

		//Set the profile
		setProfile( profile );
    }

    // override
    int getPixelsPerTile() const
    {
      return _map_service.getTileInfo().getTileSize();
    }

    // override
    osg::Image* createImage(const TileKey& key, ProgressCallback* progress)
    {
        std::stringstream buf;

        int level = key.getLevelOfDetail();

        unsigned int tile_x, tile_y;
        key.getTileXY( tile_x, tile_y );

        std::string f = _map_service.getTileInfo().getFormat();
        std::transform( f.begin(), f.end(), f.begin(), tolower );
        if ( f.length() > 3 && f.substr( 0, 3 ) == "png" )
            f = "png";

        if ( _map_service.isTiled() )
        {
            buf << _options.url()->full() << "/tile"
                << "/" << level
                << "/" << tile_y
                << "/" << tile_x << "." << f;
        }
        else
        {
            const GeoExtent& ex = key.getExtent();

            buf << std::setprecision(16)
                << _options.url()->full() << "/export"
                << "?bbox=" << ex.xMin() << "," << ex.yMin() << "," << ex.xMax() << "," << ex.yMax()
                << "&format=" << f 
                << "&size=256,256"
                << "&transparent=true"
                << "&f=image"
                << "&" << "." << f;
        }

        //Add the token if necessary
        if (_options.token().isSet())
        {
            std::string token = _options.token().value();
            if (!token.empty())
            {
                std::string str;
                str = buf.str();
                std::string sep = str.find( "?" ) == std::string::npos ? "?" : "&";
                buf << sep << "token=" << token;
            }
        }

        //OE_NOTICE << "Key = " << key->str() << ", URL = " << buf.str() << std::endl;
        //return osgDB::readImageFile( buf.str(), getOptions() );
        //return HTTPClient::readImageFile( buf.str(), getOptions(), progress );
        
        osg::ref_ptr<osg::Image> image;
		std::string bufStr;
		bufStr = buf.str();
        return URI(bufStr).readImage( 0L, CachePolicy::NO_CACHE, progress ).releaseImage();
    }

    // override
    osg::HeightField* createHeightField( const TileKey& key,
                                         ProgressCallback* progress)
    {
        //TODO
        return NULL;
    }

    // override
    virtual std::string getExtension() const 
    {
        return _format;
    }

private:
    const ArcGISOptions _options;
    optional<ProfileOptions> _profileConf;
    std::string _map;
    std::string _layer;
    std::string _format;
    MapService _map_service;
    osg::ref_ptr<const osgDB::Options> _dbOptions;
};


class ArcGISTileSourceFactory : public TileSourceDriver
{
public:
    ArcGISTileSourceFactory()
    {
        supportsExtension( "osgearth_arcgis", "ArcGIS Server" );
    }

    virtual const char* className()
    {
        return "ArcGIS Server REST ReaderWriter";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new ArcGISSource( getTileSourceOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_arcgis, ArcGISTileSourceFactory)


