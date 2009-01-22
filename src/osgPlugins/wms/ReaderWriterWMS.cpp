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

#include <osgEarth/MapConfig>
#include <osgEarth/Mercator>
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

#include "Capabilities"

using namespace osgEarth;

#define PROPERTY_URL            "url"
#define PROPERTY_LAYERS         "layers"
#define PROPERTY_STYLE          "style"
#define PROPERTY_FORMAT         "format"
#define PROPERTY_TILE_SIZE     "tile_size"
#define PROPERTY_ELEVATION_UNIT "elevation_unit"
#define PROPERTY_SRS            "srs"
#define PROPERTY_MAP_CONFIG     "map_config"

class WMSSource : public TileSource
{
public:
	WMSSource( const osgDB::ReaderWriter::Options* options ):
	  tile_size(256),
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

		if ( options->getPluginData( PROPERTY_TILE_SIZE ) )
            tile_size = as<int>( (const char*)options->getPluginData( PROPERTY_TILE_SIZE ), 256 );

        if ( options->getPluginData( PROPERTY_SRS ) )
            srs = std::string( (const char*)options->getPluginData( PROPERTY_SRS ) );

        char sep = prefix.find_first_of('?') == std::string::npos? '?' : '&';
        std::string capabilitiesRequest = prefix + sep + "service=wms&version=1.1.1&request=GetCapabilities";

        //Try to read the WMS capabilities
        _capabilities = CapabilitiesReader::read(capabilitiesRequest);


        if (_capabilities.valid())
        {
            osg::notify(osg::INFO) << "Got capabilities from " << capabilitiesRequest << std::endl;
            if (format.empty())
            {
                format = _capabilities->suggestExtension();
                osg::notify(osg::NOTICE) << "No format specified, capabilities suggested extension " << format << std::endl;
            }
        }
        else
        {
            osg::notify(osg::NOTICE) << "Could not read capabilities from WMS service using request " << capabilitiesRequest << std::endl;
        }

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
        _profile = TileGridProfile(TileGridProfile::getProfileTypeFromSRS(srs));
    }

public:
    osg::Image* createImage( const TileKey* key )
    {
        std::string uri = createURI( key );
        return osgDB::readImageFile( uri );
    }

    osg::HeightField* createHeightField( const TileKey* key )
    {
        osg::Image* image = createImage(key);
        if (!image)
        {
            osg::notify(osg::INFO) << "Failed to read heightfield from " << createURI(key) << std::endl;
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


        key->getNativeExtents( minx, miny, maxx, maxy);

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
            << "&WIDTH="<< tile_size
            << "&HEIGHT="<< tile_size
            << "&BBOX=" << minx << "," << miny << "," << maxx << "," << maxy;

        // add this to trick OSG into using the right image loader:
        buf << "&." << format;

        return buf.str();
    }

    virtual int getPixelsPerTile() const
    {
        return tile_size;
    }

    virtual std::string getExtension()  const 
    {
        return format;
    }

private:
    std::string prefix;
    std::string layers;
    std::string style;
    std::string format;
    std::string srs;

	int tile_size;

    const MapConfig *map_config;

    std::string elevation_unit;

    osg::ref_ptr<Capabilities> _capabilities;
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
            return osgDB::equalCaseInsensitive( extension, "osgearth_wms" );
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

REGISTER_OSGPLUGIN(osgearth_wms, ReaderWriterWMS)
