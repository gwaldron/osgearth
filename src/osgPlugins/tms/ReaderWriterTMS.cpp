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


class TMSSource : public TileSource
{
public:
    TMSSource(const osgDB::ReaderWriter::Options *options):
      _tileMap(0),
      _mapConfig(0)
    {

        if ( options->getPluginData( PROPERTY_URL ) )
            _url = std::string( (const char*)options->getPluginData( PROPERTY_URL ) );

        if (_url.empty())
        {
            osg::notify(osg::WARN) << "ReaderWriterTMS: No URL specified " << std::endl;
        }

        //Get the MapConfig
        if (options->getPluginData( PROPERTY_MAP_CONFIG ))
            _mapConfig = (const MapConfig*)options->getPluginData( PROPERTY_MAP_CONFIG );


        if (!_url.empty())
        {
            _tileMap = TileMapReader::read( _url );
            if (!_tileMap.valid())
            {
                osg::notify(osg::NOTICE) << "TMSSource:  error reading Tile Map Resource " << _url << std::endl;
            }
            else
            {
                if (_tileMap->_profile != TileGridProfile::PROJECTED)
                {
                    //If the source is not projected, then just create the default profile based on the profile type
                    _profile = TileGridProfile(_tileMap->_profile);
                }
                else
                {
                    //If the source is projected, then specify the bounds
                    _profile = TileGridProfile(_tileMap->_minX, _tileMap->_minY, _tileMap->_maxX, _tileMap->_maxY, _tileMap->_srs);
                }
            }
        }
      }

    osg::Image* createImage(const osgEarth::TileKey *key)
    {
        if (_tileMap.valid())
        {
            std::string image_url = _tileMap->getURL( key );

            //osg::notify(osg::NOTICE) << "URL " << image_url << std::endl;

            std::string cache_path = _mapConfig ? _mapConfig->getFullCachePath() : std::string("");
            bool offline = _mapConfig ? _mapConfig->getOfflineHint() : false;
            osgEarth::FileCache fc(cache_path);
            fc.setOffline(offline);
            
            osg::ref_ptr<osg::Image> image;
            
            if (!image_url.empty())
            {
                image = fc.readImageFile( image_url);
            }

            if (!image.valid())
            {
                //We couldn't read the image from the URL or the cache, so check to see if the given key is less than the max level
                //of the tilemap and create a transparent image.
                if (key->getLevelOfDetail() <= _tileMap->_maxLevel)
                {
                    image = new osg::Image();
                    image->allocateImage(1,1,1, GL_RGBA, GL_UNSIGNED_BYTE);
                    unsigned char *data = image->data(0,0);
                    memset(data, 0, 4);
                }
            }
            return image.release();
        }
        return 0;
    }

    osg::HeightField* createHeightField(const osgEarth::TileKey *key)
    {
        //NYI
        return 0;
    }

    virtual int getPixelsPerTile() const
    {
        return _tileMap->_format._width;
    }
private:

    osg::ref_ptr<TileMap> _tileMap;
    const MapConfig *_mapConfig;
    std::string _url;
};




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
        if ( osgDB::getLowerCaseFileExtension( file_name ) != "tms" )
            return ReadResult::FILE_NOT_HANDLED;

        return new TMSSource(options);
    }
};

REGISTER_OSGPLUGIN(tms, ReaderWriterTMS)

