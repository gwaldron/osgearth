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
#include <osgEarth/TileSource>
#include <osgEarth/Mercator>
#include <osgEarth/TMS>
#include <osgEarth/FileUtils>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <sstream>
#include <iomanip>
#include <string.h>

using namespace osgEarth;

#define PROPERTY_URL         "url"
#define PROPERTY_TMS_TYPE    "tms_type"
#define PROPERTY_TILE_SIZE   "tile_size"
#define PROPERTY_FORMAT      "format"


class TMSSource : public TileSource
{
public:
    TMSSource(const osgDB::ReaderWriter::Options* options):
      TileSource(options),
      _invertY(false)
    {
        if ( options->getPluginData( PROPERTY_URL ) )
            _url = std::string( (const char*)options->getPluginData( PROPERTY_URL ) );

        // tile_size and format are only used if no TMS tile map file can be found:
        if ( options->getPluginData( PROPERTY_TILE_SIZE ) )
            _tile_size = as<int>( (const char*)options->getPluginData( PROPERTY_TILE_SIZE ), 256 );

        if ( options->getPluginData( PROPERTY_FORMAT ) )
            _format = std::string( (const char*)options->getPluginData( PROPERTY_FORMAT ) );

        if (options->getPluginData( PROPERTY_TMS_TYPE ) )
        {
            std::string tms_type = std::string( (const char*)options->getPluginData( PROPERTY_TMS_TYPE ) );
            if (tms_type == "google")
            {
                _invertY = true;
                osg::notify(osg::INFO) << "[osgEarth::TMS] TMS driver inverting y" << std::endl;
            }
        }

        // quit now if there's no URL
        if (_url.empty())
        {
            osg::notify(osg::WARN) << "TMSSource: No URL specified " << std::endl;
        }
        else
        {
        }
    }

    const Profile* createProfile( const Profile* mapProfile, const std::string& configPath )
    {
        const Profile* result = NULL;

        std::string tmsPath = _url;

        //Find the full path to the URL
        //If we have a relative path and the map file contains a server address, just concat the server path and the url together
        if (osgEarth::isRelativePath(tmsPath) && osgDB::containsServerAddress(configPath))
        {
            tmsPath = osgDB::getFilePath(configPath) + "/" + tmsPath;
        }

        //If the path doesn't contain a server address, get the full path to the file.
        if (!osgDB::containsServerAddress(tmsPath))
        {
            tmsPath = osgEarth::getFullPath(configPath, tmsPath);
        }

        // Attempt to read the tile map parameters from a TMS TileMap XML tile on the server:
        _tileMap = TileMapReaderWriter::read( tmsPath );

        if (!_tileMap.valid())
        {
            osg::notify(osg::NOTICE) << "[osgEarth::TMS] No TileMap found; checking for client-side settings.." << std::endl;

            // If that fails, try to use an explicit profile if one exists. In this case, the map
            // config must also specify a width, height, and format.
            if ( mapProfile )
            {
                result = mapProfile;
                _tileMap = TileMap::create( _url, mapProfile, _format, _tile_size, _tile_size );
                if ( !_tileMap.valid() )
                {
                    osg::notify(osg::WARN) << "[osgEarth::TMS] No TileMap found, and no overrides set" << std::endl;
                }
            }
            else
            {
                osg::notify(osg::NOTICE) << "[osgEarth::TMS] Error reading Tile Map Resource " << _url << std::endl;
            }
        }
        else
        {
            result = _tileMap->createProfile();
        }

        //Automatically set the min and max level of the TileMap
        if (_tileMap.valid() && _tileMap->getTileSets().size() > 0)
        {
          osg::notify(osg::INFO) << "[osgEarth::TMS] TileMap min/max " << _tileMap->getMinLevel() << ", " << _tileMap->getMaxLevel() << std::endl;
          setMinLevel(_tileMap->getMinLevel());
          setMaxLevel(_tileMap->getMaxLevel());
        }

        return result;
    }


    osg::Image* createImage(const osgEarth::TileKey *key)
    {
        if (_tileMap.valid())
        {
            std::string image_url = _tileMap->getURL( key, _invertY);
                
            //osg::notify(osg::NOTICE) << "TMSSource: Key=" << key->str() << ", URL=" << image_url << std::endl;

            
            osg::ref_ptr<osg::Image> image;
            
            if (!image_url.empty())
            {
                image = osgDB::readImageFile( image_url, getOptions() );
            }

            if (!image.valid())
            {
                if (image_url.empty() || !_tileMap->intersectsKey(key))
                {
                    //We couldn't read the image from the URL or the cache, so check to see if the given key is less than the max level
                    //of the tilemap and create a transparent image.
                    if (key->getLevelOfDetail() <= _tileMap->getMaxLevel())
                    {
                        image = new osg::Image();
                        image->allocateImage(1,1,1, GL_RGBA, GL_UNSIGNED_BYTE);
                        unsigned char *data = image->data(0,0);
                        memset(data, 0, 4);
                    }
                }
            }
            return image.release();
        }
        return 0;
    }

    virtual int getPixelsPerTile() const
    {
        return _tileMap->getFormat().getWidth();
    }

    virtual std::string getExtension()  const 
    {
        return _tileMap->getFormat().getExtension();
    }

private:

    osg::ref_ptr<TileMap> _tileMap;
    std::string _url;
    bool _invertY;

    // these are backups in case no tilemap definition is found
    std::string _format;
    int _tile_size;
};




class ReaderWriterTMS : public osgDB::ReaderWriter
{
private:
    typedef std::map< std::string,osg::ref_ptr<TileMap> > TileMapCache;
    TileMapCache _tileMapCache;

public:
    ReaderWriterTMS()
    {
        supportsExtension( "osgearth_tms", "Tile Map Service" );
    }

    virtual const char* className()
    {
        return "Tile Map Service ReaderWriter";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new TMSSource(options);
    }
};

REGISTER_OSGPLUGIN(osgearth_tms, ReaderWriterTMS)

