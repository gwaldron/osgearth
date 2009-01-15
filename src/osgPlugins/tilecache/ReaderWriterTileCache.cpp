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

#include <osgEarth/TileSource>
#include <osgEarth/MapConfig>
#include <osgEarth/ImageToHeightfieldConverter>
#include <osgEarth/FileUtils>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <sstream>

using namespace osgEarth;

#define PROPERTY_URL        "url"
#define PROPERTY_LAYER      "layer"
#define PROPERTY_FORMAT     "format"
#define PROPERTY_MAP_CONFIG "map_config"

class TileCacheSource : public TileSource
{
public:
    TileCacheSource( const osgDB::ReaderWriter::Options* _options ) :
      options( _options ),
      map_config(0)
    {
        if ( options.valid() )
        {
            if ( options->getPluginData( PROPERTY_URL ) )
                url = std::string( (const char*)options->getPluginData( PROPERTY_URL ) );

            if ( options->getPluginData( PROPERTY_LAYER ) )
                layer = std::string( (const char*)options->getPluginData( PROPERTY_LAYER ) );

            if ( options->getPluginData( PROPERTY_FORMAT ) )
                format = std::string( (const char*)options->getPluginData( PROPERTY_FORMAT ) );

            if (options->getPluginData( PROPERTY_MAP_CONFIG ))
                map_config = (const MapConfig*)options->getPluginData( PROPERTY_MAP_CONFIG );
        }

        _profile = TileGridProfile(map_config->getProfile());
        if (_profile.profileType() == TileGridProfile::UNKNOWN)
        {
            //Set the profile to global geodetic if there is no override
            _profile = TileGridProfile(TileGridProfile::GLOBAL_GEODETIC);
        }
    }

    osg::Image* createImage( const TileKey* key )
    {
        unsigned int level, tile_x, tile_y;
        level = key->getLevelOfDetail();
        key->getTileXY( tile_x, tile_y );

        // need to invert the y-tile index
        tile_y = key->getMapSizeTiles() - tile_y - 1;

        char buf[2048];
        sprintf( buf, "%s/%s/%02d/%03d/%03d/%03d/%03d/%03d/%03d.%s",
            url.c_str(),
            layer.c_str(),
            level,
            (tile_x / 1000000),
            (tile_x / 1000) % 1000,
            (tile_x % 1000),
            (tile_y / 1000000),
            (tile_y / 1000) % 1000,
            (tile_y % 1000),
            format.c_str());

        //If we are in offline mode, don't connect to the web
        if (osgDB::containsServerAddress( buf ) && map_config->getOfflineHint()) return 0;
        
        std::string path = buf;

        //If we have a relative path and the map file contains a server address, just concat the server path and the cache together.
        if (osgEarth::isRelativePath(path) && osgDB::containsServerAddress(map_config->getFilename()))
        {
            path = osgDB::getFilePath(map_config->getFilename()) + "/" + path;
        }

        //If the path doesn't contain a server address, get the full path to the file.
        if (!osgDB::containsServerAddress(path))
        {
            path = osgEarth::getFullPath(map_config->getFilename(), path);
        }
        return osgDB::readImageFile( path, options.get());
    }

    osg::HeightField* createHeightField( const TileKey* key )
    {
        osg::ref_ptr<osg::Image> image = createImage(key);
        osg::HeightField* hf = 0;
        if (image.valid())
        {
            hf = ImageToHeightFieldConverter::convert(image.get());
        }      
        return hf;
    }

    virtual std::string getExtension()  const 
    {
        return format;
    }

private:
    osg::ref_ptr<const osgDB::ReaderWriter::Options> options;
    std::string url;
    std::string layer;
    std::string format;
    const MapConfig* map_config;
};

// Reads tiles from a TileCache disk cache.
class ReaderWriterTileCache : public osgDB::ReaderWriter
{
    public:
        ReaderWriterTileCache() {}

        virtual const char* className()
        {
            return "TileCache disk cache ReaderWriter";
        }
        
        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive( extension, "osgearth_tilecache" );
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            std::string ext = osgDB::getFileExtension( file_name );
            if ( !acceptsExtension( ext ) )
            {
                return ReadResult::FILE_NOT_HANDLED;
            }

            return new TileCacheSource(options);
        }
};

REGISTER_OSGPLUGIN(osgearth_tilecache, ReaderWriterTileCache)
