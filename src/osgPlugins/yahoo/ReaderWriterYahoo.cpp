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

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <sstream>

using namespace osgEarth;

#define PROPERTY_DATASET    "dataset"
#define PROPERTY_MAP_CONFIG "map_config"

class YahooSource : public TileSource
{
public:
    YahooSource( const osgDB::ReaderWriter::Options* _options ) :
      options( _options ),
      map_config(0)
    {
        //Set the profile to global mercator
        _profile = TileGridProfile(TileGridProfile::GLOBAL_MERCATOR);

        if ( options.valid() )
        {
            if ( options->getPluginData( PROPERTY_DATASET ) )
                dataset = std::string( (const char*)options->getPluginData( PROPERTY_DATASET ) );

            if (options->getPluginData( PROPERTY_MAP_CONFIG ))
                map_config = (const MapConfig*)options->getPluginData( PROPERTY_MAP_CONFIG);
        }

        // validate dataset
        if ( dataset.empty() ) dataset = "roads"; // defaul to the satellite view
    }

    osg::Image* createImage( const TileKey* key )
    {
        //If we are given a PlateCarreTileKey, use the MercatorTileConverter to create the image
        if ( key->isGeodetic() )
        {
            MercatorTileConverter converter( this );
            return converter.createImage(  key );
        }

        //Return NULL if we are given a non-mercator key
        if ( !key->isMercator() ) return 0;

        std::stringstream buf;
        
        if ( dataset == "roads" || dataset == "map" )
        {            
            // http://us.maps1.yimg.com/us.tile.maps.yimg.com/tl?v=4.1&md=2&x=0&y=0&z=2&r=1
            unsigned int tile_x, tile_y;
            key->getTileXY( tile_x, tile_y );
            int size = key->getMapSizeTiles();
            int zoom = key->getLevelOfDetail();

            buf << "http://us.maps1.yimg.com/us.tile.maps.yimg.com/tl"
                << "?v=4.1&md=2&r=1"
                << "&x=" << (int)tile_x
                << "&y=" << (size-1-(int)tile_y) - size/2
                << "&z=" << zoom + 1
                << "&.jpg";
        }
        else if ( dataset == "aerial" || dataset == "satellite" )
        {
            unsigned int tile_x, tile_y;
            key->getTileXY( tile_x, tile_y );
            int size = key->getMapSizeTiles();
            int zoom = key->getLevelOfDetail();

            buf << "http://us.maps3.yimg.com/aerial.maps.yimg.com/ximg"
                << "?v=1.8&s=256&t=a&r=1"
                << "&x=" << (int)tile_x
                << "&y=" << (size-1-(int)tile_y) - size/2
                << "&z=" << zoom + 1
                << "&.jpg";
        }

        //osg::notify(osg::NOTICE) << buf.str() << std::endl;
        return osgDB::readImageFile( buf.str(), options.get() );
    }

    osg::HeightField* createHeightField( const TileKey* key )
    {
        //TODO
        return NULL;
    }

    virtual std::string getExtension()  const 
    {
        //All Yahoo tiles are in JPEG format
        return "jpg";
    }

private:
    osg::ref_ptr<const osgDB::ReaderWriter::Options> options;
    std::string dataset;

    const MapConfig* map_config;
};


class ReaderWriterYahoo : public osgDB::ReaderWriter
{
    public:
        ReaderWriterYahoo()
        {
            supportsExtension( "osgearth_yahoo", "Yahoo maps data" );
        }

        virtual const char* className()
        {
            return "Yahoo Imagery ReaderWriter";
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
                return ReadResult::FILE_NOT_HANDLED;

            return new YahooSource(options);
        }
};

REGISTER_OSGPLUGIN(osgearth_yahoo, ReaderWriterYahoo)

