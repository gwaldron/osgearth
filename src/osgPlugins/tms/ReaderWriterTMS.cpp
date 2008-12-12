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
#include <osgEarth/PlateCarre>
#include <osgEarth/FileCache>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <sstream>
#include <iomanip>

#include "TMS"

using namespace osgEarth;

#define PROPERTY_URL        "url"
#define PROPERTY_MAP_CONFIG "map_config"




class ReaderWriterTMS : public osgDB::ReaderWriter
{
private:
    typedef std::map<std::string,osg::ref_ptr<TileMap>> TileMapCache;
    TileMapCache _tileMapCache;

public:
    ReaderWriterTMS()
    {
        supportsExtension( "tms", "Tile Map Service" );
    }

    virtual const char* className()
    {
        return "Tile Map Service ReaderWriter";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        return readNode( file_name, options );
    }

    virtual ReadResult readImage(const std::string& file_name, const Options* options) const
    {
        if ( osgDB::getLowerCaseFileExtension( file_name ) != "tms" )
            return ReadResult::FILE_NOT_HANDLED;

        osg::ref_ptr<TileKey> key = TileKeyFactory::createFromName(
            file_name.substr( 0, file_name.find_first_of( '.' ) ) );

        //Read the URL
        std::string url;
        if ( options->getPluginData( PROPERTY_URL ) )
            url = std::string( (const char*)options->getPluginData( PROPERTY_URL ) );

        if (url.empty())
        {
            osg::notify(osg::WARN) << "ReaderWriterTMS: No URL specified " << std::endl;
            return ReadResult::FILE_NOT_FOUND;
        }

        const MapConfig *map_config;
        //Get the MapConfig
        if (options->getPluginData( PROPERTY_MAP_CONFIG ))
            map_config = (const MapConfig*)options->getPluginData( PROPERTY_MAP_CONFIG );


        osg::ref_ptr<TileMap> tileMap;
        TileMapCache::const_iterator itr = _tileMapCache.find( url );
        if (itr == _tileMapCache.end())
        {
            tileMap = TileMapReader::read( url );
            const_cast<ReaderWriterTMS*>(this)->_tileMapCache[ url ] = tileMap.get();
        }
        else
        {
            tileMap = itr->second;
        }

        if (tileMap.valid())
        {
            std::string image_url = tileMap->getURL( key.get() );

            std::string cache_path = map_config ? map_config->getFullCachePath() : std::string("");
            bool offline = map_config ? map_config->getOfflineHint() : false;
            osgEarth::FileCache fc(cache_path);
            fc.setOffline(offline);
            
            osg::ref_ptr<osg::Image> image;
            
            if (!image_url.empty())
            {
                image = fc.readImageFile( image_url, options );
            }

            if (!image.valid())
            {
                //We couldn't read the image, so check to see if the tile intersects the bounding box 
                //If it does, simplfy create a transparent texture to make the pager continue subdividing
                if (tileMap->intersectsKey( key.get()) && key->getLevelOfDetail() <= tileMap->_maxLevel)
                {
                    //We only want to replace the image with a transparent one if there isn't supposed to be a valid
                    //image in the TileSet.  If the URL is not empty, then there was supposed to be an image and this
                    //should result in a failure
                    if (image_url.empty())
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
        return ReadResult::FILE_NOT_FOUND;
    }

    virtual ReadResult readHeightField(const std::string& file_name, const Options* opt) const
    {
        return ReadResult::FILE_NOT_HANDLED;
        //NYI
    }

    virtual ReadResult readNode(const std::string& file_name, const Options* opt) const
    {
        return ReadResult::FILE_NOT_HANDLED;
    }
};

REGISTER_OSGPLUGIN(tms, ReaderWriterTMS)

