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
#define PROPERTY_DATASET        "dataset"
#define PROPERTY_FORMAT         "format"
#define PROPERTY_MAP_CONFIG     "map_config"

//http://www.worldwindcentral.com/wiki/TileService
class TileServiceSource : public TileSource
{
public:
	TileServiceSource( const osgDB::ReaderWriter::Options* options ):
      _mapConfig(0)
    {
        if ( options->getPluginData( PROPERTY_URL ) )
            _url = std::string( (const char*)options->getPluginData( PROPERTY_URL ) );

        if (options->getPluginData( PROPERTY_FORMAT ))
            _format = std::string( (const char*)options->getPluginData( PROPERTY_FORMAT ) );

        if (options->getPluginData( PROPERTY_DATASET ))
            _dataset = std::string( (const char*)options->getPluginData( PROPERTY_DATASET ) );

        if ( options->getPluginData( PROPERTY_MAP_CONFIG))
             _mapConfig = (const MapConfig*)options->getPluginData( PROPERTY_MAP_CONFIG );

         if ( _format.empty() )
            _format = "png";

         _profile = TileGridProfile(TileGridProfile::GLOBAL_GEODETIC);
    }

public:
    osg::Image* createImage( const TileKey* key )
    {
        std::string uri = createURI( key );

        std::string cache_path = _mapConfig ? _mapConfig->getFullCachePath() : std::string("");
        bool offline = _mapConfig ? _mapConfig->getOfflineHint() : false;

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
        //NOP
        return NULL;
    }

    std::string createURI( const TileKey* key ) const
    {
        unsigned int x, y;
        key->getTileXY(x, y);

        unsigned int lod = key->getLevelOfDetail();

        std::stringstream buf;
        //http://s0.tileservice.worldwindcentral.com/getTile?interface=map&version=1&dataset=bmng.topo.bathy.200401&level=0&x=0&y=0
        buf << _url << "interface=map&version=1"
            << "&dataset=" << _dataset
            << "&level=" << lod
            << "&x=" << x
            << "&y=" << y
            << "&." << _format;//Add this to trick osg into using the correct loader.
        return buf.str();
    }

    virtual int getPixelsPerTile() const
    {
        return 256;
    }
private:
    std::string _url;
    std::string _dataset;
    std::string _format;
    const MapConfig *_mapConfig;
};


class ReaderWriterTileService : public osgDB::ReaderWriter
{
    public:
        ReaderWriterTileService() {}

        virtual const char* className()
        {
            return "TileService Reader";
        }
        
        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive( extension, "osgearth_tileservice" );
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* opt) const
        {
            std::string ext = osgDB::getFileExtension( file_name );
            if ( !acceptsExtension( ext ) )
            {
                return ReadResult::FILE_NOT_HANDLED;
            }

            return new TileServiceSource(opt);
        }
};

REGISTER_OSGPLUGIN(osgearth_tileservice, ReaderWriterTileService)
