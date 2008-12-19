/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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

#include <osgEarth/PlateCarre>
#include <osgEarth/MapConfig>
#include <osgEarth/Mercator>
#include <osgEarth/FileCache>
#include <osgEarth/TileSource>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <sstream>
#include <stdlib.h>
#include <iomanip>

using namespace osgEarth;

#define PROPERTY_URL            "url"
#define PROPERTY_LAYERS         "layers"
#define PROPERTY_STYLE          "style"
#define PROPERTY_FORMAT         "format"
#define PROPERTY_TILE_WIDTH     "tile_width"
#define PROPERTY_TILE_HEIGHT    "tile_height"
#define PROPERTY_ELEVATION_UNIT "elevation_unit"
#define PROPERTY_SRS            "srs"
#define PROPERTY_MAP_CONFIG     "map_config"

class WMSSource : public TileSource
{
public:
	WMSSource( const osgDB::ReaderWriter::Options* options ):
	  tile_width(256),
	  tile_height(256),
      map_config(0)
    {
        if ( options->getPluginData( PROPERTY_URL ) )
            prefix = std::string( (const char*)options->getPluginData( PROPERTY_URL ) );

        if ( options->getPluginData( PROPERTY_LAYERS ) )
            layers = std::string( (const char*)options->getPluginData( PROPERTY_LAYERS ) );

        if ( options->getPluginData( PROPERTY_STYLE ) )
            style = std::string( (const char*)options->getPluginData( PROPERTY_STYLE ) );

        if ( options->getPluginData( PROPERTY_FORMAT ) )
            format = std::string( (const char*)options->getPluginData( PROPERTY_FORMAT ) );

        if ( options->getPluginData( PROPERTY_MAP_CONFIG))
             map_config = (const MapConfig*)options->getPluginData( PROPERTY_MAP_CONFIG );

        if ( options->getPluginData( PROPERTY_ELEVATION_UNIT))
             elevation_unit = std::string( (const char*)options->getPluginData( PROPERTY_ELEVATION_UNIT ) );

		if ( options->getPluginData( PROPERTY_TILE_WIDTH ) )
            tile_width = as<int>( (const char*)options->getPluginData( PROPERTY_TILE_WIDTH ), 256 );

        if ( options->getPluginData( PROPERTY_TILE_HEIGHT ) )
            tile_height = as<int>( (const char*)options->getPluginData( PROPERTY_TILE_HEIGHT ), 256 );

        if ( options->getPluginData( PROPERTY_SRS ) )
            srs = std::string( (const char*)options->getPluginData( PROPERTY_SRS ) );

        if ( format.empty() )
            format = "png";

        if ( srs.empty() )
        {
            srs = "EPSG:4326";
        }

        if ( elevation_unit.empty())
        {
            elevation_unit = "m";
        }

        //Set the profile based on on the SRS
        _profile = TileGridProfile(TileGridProfile::getProfileTypeFromSRS(srs), tile_width);
    }

public:
    osg::Image* createImage( const TileKey* key )
    {
        std::string uri = createURI( key );

        std::string cache_path = map_config ? map_config->getFullCachePath() : std::string("");
        bool offline = map_config ? map_config->getOfflineHint() : false;

        osgEarth::FileCache fc(cache_path);
        fc.setOffline(offline);
        osg::Image* image = fc.readImageFile(uri);

        if ( !image )
        {
            osg::notify(osg::WARN) << "Failed to load image from " << uri << std::endl;
        }
        return image;
    }

    osg::HeightField* createHeightField( const TileKey* key )
    {
        osg::Image* image = createImage(key);
        if (!image)
        {
            osg::notify(osg::WARN) << "Failed to read heightfield from " << createURI(key) << std::endl;
        }

        float scaleFactor = 1;

        //Scale the heightfield to meters
        if (elevation_unit == "ft")
        {
            scaleFactor = 0.3048;
        }
        return (ImageToHeightFieldConverter::convert(image, scaleFactor));        
    }

    std::string createURI( const TileKey* key ) const
    {
        double minx, miny, maxx, maxy;


        const PlateCarreTileKey* pk = dynamic_cast<const PlateCarreTileKey*>(key);
        const MercatorTileKey* mk = dynamic_cast<const MercatorTileKey*>(key);

        //TODO: Would using getGeoExtents and 4326 work well for both PlateCarre and Mercator?
        if (pk)
        {
            key->getGeoExtents( minx, miny, maxx, maxy );
        }
        else if (mk)
        {
            mk->getMeterExtents(minx, miny, maxx, maxy);
        }

        // http://labs.metacarta.com/wms-c/Basic.py?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap&LAYERS=basic&BBOX=-180,-90,0,90

        // build the WMS request:
        char sep = prefix.find_first_of('?') == std::string::npos? '?' : '&';

        std::stringstream buf;
        buf
            << std::fixed << prefix << sep
            << "SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap"
            << "&LAYERS=" << layers
            << "&FORMAT=image/" << format
            << "&STYLES=" << style
            << "&SRS=" << srs
            << "&WIDTH="<< tile_width
            << "&HEIGHT="<< tile_height
            << "&BBOX=" << minx << "," << miny << "," << maxx << "," << maxy;

        // add this to trick OSG into using the right image loader:
        buf << "&." << format;

        return buf.str();
    }

private:
    std::string prefix;
    std::string layers;
    std::string style;
    std::string format;
    std::string srs;

	int tile_width;
	int tile_height;

    const MapConfig *map_config;

    std::string elevation_unit;
};


class ReaderWriterWMS : public osgDB::ReaderWriter
{
    public:
        ReaderWriterWMS() {}

        virtual const char* className()
        {
            return "WMS Reader";
        }
        
        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive( extension, "wms" );
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* opt) const
        {
            std::string ext = osgDB::getFileExtension( file_name );
            if ( !acceptsExtension( ext ) )
            {
                return ReadResult::FILE_NOT_HANDLED;
            }

            return new WMSSource(opt);
        }
};

REGISTER_OSGPLUGIN(wms, ReaderWriterWMS)
