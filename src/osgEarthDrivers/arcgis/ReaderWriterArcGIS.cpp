/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
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

#define LC "[ReaderWriterArcGIS] "

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
        //TODO: allow single layers vs. "fused view"
        if ( _layer.empty() )
		{
            _layer = "_alllayers"; // default to the AGS "fused view"
		}
		else
		{
			_layer = *_options.layers();
		}

        if ( _options.format().isSet() )
		{
            _format = *_options.format();
		}
        else
		{
            _format = _map_service.getTileInfo().getFormat();
		}

        _format = osgEarth::toLower(_format);

        if ( _format.length() > 3 && _format.substr( 0, 3 ) == "png" )
		{
            _format = "png";
		}

        if ( _format == "mixed" )
		{
            _format = "";
		}
        if ( !_format.empty() )
		{
            _dot_format = "." + _format;
		}
    }

    // override
    Status initialize( const osgDB::Options* dbOptions )
    {
        // add the security token to the URL if necessary:
        URI url = _options.url().value();

		OE_DEBUG << LC << "Initial URL: " << url.full() << std::endl;

		// append token to url in query string is set
        if (_options.token().isSet())
        {
            std::string token = _options.token().value();
            if (!token.empty())
            {
                std::string sep = url.full().find( "?" ) == std::string::npos ? "?" : "&";
                url = url.append( sep + std::string("token=") + token );
            }
        }
		else
		{
			OE_DEBUG << LC << "Token not set" << std::endl;
		}

		// append layers to url in query string is set .. format is show:1,2,3
		if (_options.layers().isSet())
        {
			std::string layers = _options.layers().value();
			OE_DEBUG << LC << "_Layers: " << layers << std::endl;
			if (!layers.empty())
			{
				std::string sep = url.full().find( "?" ) == std::string::npos ? "?" : "&";
				url = url.append( sep + std::string("layers=show:") + layers );
			}
		}
		else
		{
			OE_DEBUG << LC << "Layer options not set" << std::endl;
		}

        OE_DEBUG << LC << "_map_service URL: " << url.full() << std::endl;

        // read map service metadata from the server
        if ( !_map_service.init(url, dbOptions) )
        {
			OE_INFO << LC << "_map_service.init failed: " << _map_service.getError() << std::endl;

            return Status::Error(
                Status::ResourceUnavailable, Stringify()
                    << "[osgearth] [ArcGIS] map service initialization failed: "
                    << _map_service.getError() );
        }

        _copyright = _map_service.getCopyright();

        _dbOptions = Registry::instance()->cloneOrCreateOptions( dbOptions );

        // establish a profile if we don't already have one:
        if ( !getProfile() )
        {
            const Profile* profile = NULL;

            if ( _profileConf.isSet() )
            {
                profile = Profile::create( _profileConf.get() );
            }
            else if ( _map_service.getProfile() )
            {
                profile = _map_service.getProfile();
            }
            else
            {
                // finally, fall back on lat/long
                profile = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
            }
            setProfile( profile );
        }

        return STATUS_OK;
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

        if ( _map_service.isTiled() )
        {
            buf << _options.url()->full() << "/tile"
                << "/" << level
                << "/" << tile_y
                << "/" << tile_x << _dot_format;
        }
        else
        {
            const GeoExtent& ex = key.getExtent();

            buf << std::setprecision(16)
                << _options.url()->full() << "/export"
                << "?bbox=" << ex.xMin() << "," << ex.yMin() << "," << ex.xMax() << "," << ex.yMax()
                << "&format=" << _format
                << "&size=256,256"
                << "&transparent=true"
                << "&f=image";
                //<< "&" << "." << f;
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

		//Add the layers if necessary
        if (_options.layers().isSet())
        {
            std::string layers = _options.layers().value();
            if (!layers.empty())
            {
                std::string str;
                str = buf.str();
                std::string sep = str.find( "?" ) == std::string::npos ? "?" : "&";
				buf << sep << "layers=show:" << layers;
            }
        }

        std::string bufStr;
        bufStr = buf.str();
        return URI(bufStr, _options.url()->context()).getImage( _dbOptions.get(), progress );
    }

    // override
    osg::HeightField* createHeightField( const TileKey& key,
                                         ProgressCallback* progress)
    {
        //TODO
        return NULL;
    }

    virtual std::string getAttribution() const
    {
        return _copyright;
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
    std::string _format, _dot_format;
    std::string _copyright;
    MapService _map_service;
    osg::ref_ptr<osgDB::Options> _dbOptions;
};


class ArcGISTileSourceFactory : public TileSourceDriver
{
public:
    ArcGISTileSourceFactory()
    {
        supportsExtension( "osgearth_arcgis", "ArcGIS Server" );
    }

    virtual const char* className() const
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


