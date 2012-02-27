/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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
#include <osgEarth/FileUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/HTTPClient>
#include <osgEarthUtil/TMS>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <sstream>
#include <iomanip>
#include <string.h>

#include "TMSOptions"

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Drivers;

#define LC "[TMS driver] "


class TMSSource : public TileSource
{
public:
    TMSSource(const TileSourceOptions& options) : TileSource(options), _options(options)
    {
        _invertY = _options.tmsType() == "google";
    }


    void initialize(const osgDB::Options* dbOptions,
                    const Profile*        overrideProfile )
    {
        _dbOptions = dbOptions;

        const Profile* result = NULL;

        URI tmsURI = _options.url().value();
        if ( tmsURI.empty() )
        {
            OE_WARN << LC << "Fail: TMS driver requires a valid \"url\" property" << std::endl;
            return;
        }

		// Attempt to read the tile map parameters from a TMS TileMap XML tile on the server:
        _tileMap = TMS::TileMapReaderWriter::read( tmsURI.full(), 0L );


		//Take the override profile if one is given
		if (overrideProfile)
		{
            OE_INFO << LC 
                << "Using override profile \"" << overrideProfile->toString() 
                << "\" for URI \"" << tmsURI.base() << "\"" 
                << std::endl;

			result = overrideProfile;
            _tileMap = TMS::TileMap::create( 
                _options.url()->full(),
                overrideProfile, 
                _options.format().value(),
                _options.tileSize().value(), 
                _options.tileSize().value() );
		}
		else
		{
     		if (_tileMap.valid())
			{
				result = _tileMap->createProfile();
			}
			else
			{
                OE_WARN << LC << "Error reading TMS TileMap, and no overrides set (url=" << tmsURI.full() << ")" << std::endl;		
			    return;
			}
		}

        //Automatically set the min and max level of the TileMap
        if (_tileMap.valid() && _tileMap->getTileSets().size() > 0)
        {
          OE_DEBUG << LC << "TileMap min/max " << _tileMap->getMinLevel() << ", " << _tileMap->getMaxLevel() << std::endl;
          if (_tileMap->getDataExtents().size() > 0)
          {
              for (DataExtentList::iterator itr = _tileMap->getDataExtents().begin(); itr != _tileMap->getDataExtents().end(); ++itr)
              {
                  this->getDataExtents().push_back(*itr);
              }
          }
          else
          {
              //Push back a single area that encompasses the whole profile going up to the max level
              this->getDataExtents().push_back(DataExtent(result->getExtent(), 0, _tileMap->getMaxLevel()));
          }
        }

		setProfile( result );
    }


    osg::Image* createImage(const TileKey&        key,
                            ProgressCallback*     progress )
    {
        if (_tileMap.valid() && key.getLevelOfDetail() <= getMaxDataLevel() )
        {
            std::string image_url = _tileMap->getURL( key, _invertY );
                
            //OE_NOTICE << "TMSSource: Key=" << key.str() << ", URL=" << image_url << std::endl;

            osg::ref_ptr<osg::Image> image;            
            if (!image_url.empty())
            {
                image = URI(image_url).readImage( _dbOptions.get(), CachePolicy::NO_CACHE, progress ).getImage();
            }

            if (!image.valid())
            {
                if (image_url.empty() || !_tileMap->intersectsKey(key))
                {
                    //We couldn't read the image from the URL or the cache, so check to see if the given key is less than the max level
                    //of the tilemap and create a transparent image.
                    if (key.getLevelOfDetail() <= _tileMap->getMaxLevel())
                    {
                        OE_DEBUG << LC << "Returning empty image " << std::endl;
                        return ImageUtils::createEmptyImage();
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

    osg::ref_ptr<TMS::TileMap> _tileMap;
    bool                       _invertY;
    const TMSOptions           _options;
    osg::ref_ptr<const osgDB::Options> _dbOptions;
};




class ReaderWriterTMS : public TileSourceDriver
{
private:
    typedef std::map< std::string,osg::ref_ptr<TMS::TileMap> > TileMapCache;
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

        return new TMSSource( getTileSourceOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_tms, ReaderWriterTMS)

