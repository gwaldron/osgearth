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

#include "MapService.h"
#include <osgEarth/MapConfig>
#include <osgEarth/TileSource>
#include <osgEarth/Mercator>
#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <sstream>
#include <iomanip>

using namespace osgEarth;

#define PROPERTY_URL        "url"
#define PROPERTY_PROFILE    "profile"
#define PROPERTY_MAP_CONFIG "map_config"

class ArcGISSource : public TileSource
{
public:
    ArcGISSource( const osgDB::ReaderWriter::Options* _options ) :
      options( _options ),
      map_config(0)
    {
        if ( options.valid() )
        {
            // global map configuration
            if (options->getPluginData( PROPERTY_MAP_CONFIG ))
                map_config = (const MapConfig*)options->getPluginData( PROPERTY_MAP_CONFIG );

            // this is the ArcGIS REST services URL for the map service,
            // e.g. http://server/ArcGIS/rest/services/MyMapService
            if ( options->getPluginData( PROPERTY_URL ) )
                url = std::string( (const char*)options->getPluginData( PROPERTY_URL ) );

            // force a profile type
            if ( options->getPluginData( PROPERTY_PROFILE ) )
                manual_profile = TileGridProfile( std::string( (const char*)options->getPluginData( PROPERTY_PROFILE ) ) );
        }

        // validate dataset
        if ( layer.empty() ) layer = "_alllayers"; // default to the AGS "fused view"
        if ( format.empty() ) format = "png";

        // read metadata from the server
        if ( !map_service.init( url ) )
        {
            osg::notify(osg::WARN) << "[osgearth] [ArcGIS] map service initialization failed: "
                << map_service.getError() << std::endl;
        }
    }

    const TileGridProfile& getProfile() const
    {
        return manual_profile.isValid() ? manual_profile : map_service.getProfile();
    }

    osg::Image* createImage( const TileKey* key )
    {
        std::stringstream buf;

        int level = key->getLevelOfDetail()-1;

        unsigned int tile_x, tile_y;
        key->getTileXY( tile_x, tile_y );

        buf << url << "/tile"
            << "/" << level
            << "/" << tile_y
            << "/" << tile_x << "." << map_service.getTileInfo().getFormat();

        //osg::notify(osg::NOTICE) << "Key = " << key->str() << ", URL = " << buf.str() << std::endl;
        return osgDB::readImageFile( buf.str(), options.get() );
    }

    osg::HeightField* createHeightField( const TileKey* key )
    {
        //TODO
        return NULL;
    }

    virtual std::string getExtension()  const 
    {
        return format;
    }

private:
    osg::ref_ptr<const osgDB::ReaderWriter::Options> options;
    std::string url;
    std::string map;
    std::string layer;
    std::string format;
    MapService map_service;
    TileGridProfile manual_profile;

    const MapConfig* map_config;
};


class ReaderWriterArcGIS : public osgDB::ReaderWriter
{
    public:
        ReaderWriterArcGIS()
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

            return new ArcGISSource(options);
        }
};

REGISTER_OSGPLUGIN(osgearth_arcgis, ReaderWriterArcGIS);

