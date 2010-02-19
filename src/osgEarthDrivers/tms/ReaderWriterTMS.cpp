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
#include <osgEarth/TMS>
#include <osgEarth/FileUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/HTTPClient>

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
using namespace osgEarth::Drivers;

//#define PROPERTY_URL         "url"
//#define PROPERTY_TMS_TYPE    "tms_type"
//#define PROPERTY_TILE_SIZE   "tile_size"
//#define PROPERTY_FORMAT      "format"


class TMSSource : public TileSource
{
public:
    TMSSource(const PluginOptions* options) : TileSource(options)
    {
        _settings = dynamic_cast<const TMSOptions*>( options );
        if ( !_settings.valid() )
            _settings = new TMSOptions( options );

        _invertY = _settings->tmsType() == "google";
    }


    void initialize( const std::string& referenceURI, const Profile* overrideProfile)
    {
        const Profile* result = NULL;

        std::string tmsPath = _settings->url().value();

        //Find the full path to the URL
        //If we have a relative path and the map file contains a server address, just concat the server path and the url together
        if (osgEarth::isRelativePath(tmsPath) && osgDB::containsServerAddress(referenceURI))
        {
            tmsPath = osgDB::getFilePath(referenceURI) + std::string("/") + tmsPath;
        }

        //If the path doesn't contain a server address, get the full path to the file.
        if (!osgDB::containsServerAddress(tmsPath))
        {
            tmsPath = osgEarth::getFullPath(referenceURI, tmsPath);
        }

		// Attempt to read the tile map parameters from a TMS TileMap XML tile on the server:
    	_tileMap = TileMapReaderWriter::read( tmsPath, getOptions() );


		//Take the override profile if one is given
		if (overrideProfile)
		{
		    osg::notify(osg::NOTICE) << "[osgEarth::TMS] Using override profile " << overrideProfile->toString() << std::endl;				
			result = overrideProfile;
			_tileMap = TileMap::create( 
                _settings->url().value(), 
                overrideProfile, 
                _settings->format().value(),
                _settings->tileSize().value(), 
                _settings->tileSize().value() );
		}
		else
		{
     		if (_tileMap.valid())
			{
				result = _tileMap->createProfile();
			}
			else
			{
		      osg::notify(osg::WARN) << "[osgEarth::TMS] Error reading TileMap and no overrides set" << std::endl;		
			  return;
			}
		}

        //Automatically set the min and max level of the TileMap
        if (_tileMap.valid() && _tileMap->getTileSets().size() > 0)
        {
          osg::notify(osg::INFO) << "[osgEarth::TMS] TileMap min/max " << _tileMap->getMinLevel() << ", " << _tileMap->getMaxLevel() << std::endl;
          //TODO:  Replace this with a concept of the "data" having a min/max level, not the min-max visible level
          //setMinLevel(_tileMap->getMinLevel());
          //setMaxLevel(_tileMap->getMaxLevel());

          this->setMaxDataLevel( _tileMap->getMaxLevel() );
        }

		setProfile( result );
    }


    osg::Image* createImage(const osgEarth::TileKey *key,
                            ProgressCallback* progress)
    {
        if (_tileMap.valid() && key->getLevelOfDetail() <= getMaxDataLevel() )
        {
            std::string image_url = _tileMap->getURL( key, _invertY );
                
            //osg::notify(osg::NOTICE) << "TMSSource: Key=" << key->str() << ", URL=" << image_url << std::endl;

            
            osg::ref_ptr<osg::Image> image;
            
            if (!image_url.empty())
            {
                HTTPClient::readImageFile( image_url, image, getOptions(), progress );
            }

            if (!image.valid())
            {
                if (image_url.empty() || !_tileMap->intersectsKey(key))
                {
                    //We couldn't read the image from the URL or the cache, so check to see if the given key is less than the max level
                    //of the tilemap and create a transparent image.
                    if (key->getLevelOfDetail() <= _tileMap->getMaxLevel())
                    {
                        osg::notify(osg::NOTICE) << "Returning empty image " << std::endl;
                        return ImageUtils::getEmptyImage();
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
    bool _invertY;

    //std::string _url;

    // these are backups in case no tilemap definition is found
    //std::string _format;
    //int _tile_size;

    osg::ref_ptr<const TMSOptions> _settings;
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

        return new TMSSource( static_cast<const PluginOptions*>(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_tms, ReaderWriterTMS)

