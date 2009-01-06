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
#include <osgEarth/FileCache>

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

class GoogleSource : public TileSource
{
public:
    GoogleSource( const osgDB::ReaderWriter::Options* _options ) :
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
                map_config = (const MapConfig*)options->getPluginData(PROPERTY_MAP_CONFIG);
        }

        // validate dataset
        if ( dataset.empty() ) dataset = "satellite"; // defaul to the satellite view
    }

    osg::Image* createImage( const TileKey* key )
    {
        //If we are given a PlateCarreTileKey, use the MercatorTileConverter to create the image
        if ( dynamic_cast<const PlateCarreTileKey*>( key ) )
        {
            MercatorTileConverter converter( this );
            return converter.createImage( static_cast<const PlateCarreTileKey*>( key ) );
        }

        const MercatorTileKey* mkey = static_cast<const MercatorTileKey*>( key );

        std::stringstream buf;
        
        if ( dataset == "satellite" )
        {            
            char server = key->str().length() > 0? key->str()[key->str().length()-1] : '0';
            unsigned int tile_x, tile_y;
            mkey->getTileXY( tile_x, tile_y );
            int zoom = key->getLevelOfDetail();

            buf << "http://khm" << server << ".google.com/kh/v=32&hl=en"
                << "&x=" << tile_x
                << "&y=" << tile_y
                << "&z=" << zoom
                << "&s=Ga&.jpg";
            
            // http://khm0.google.com/kh/v=32&hl=en&x=4&y=6&z=4&s=Ga
        }
        else if ( dataset == "terrain" )
        {
            char server = key->str().length() > 0? key->str()[key->str().length()-1] : '0';
            unsigned int tile_x, tile_y;
            mkey->getTileXY( tile_x, tile_y );
            buf << "http://mt" << server << ".google.com/mt?v=app.81&hl=en&x="
                << tile_x << "&y=" << tile_y << "&zoom=" 
                << 17-key->getLevelOfDetail() << "&.jpg";
        }
        else if ( dataset == "labels" )
        {
            char server = key->str().length() > 0? key->str()[key->str().length()-1] : '0';
            unsigned int tile_x, tile_y;
            mkey->getTileXY( tile_x, tile_y );
            int zoom = key->getLevelOfDetail();

            buf << "http://mt" << server << ".google.com/mt/v=w2t.83&hl=en"
                << "&x=" << tile_x
                << "&y=" << tile_y
                << "&z=" << zoom
                << "&s=Ga&.png";

            //http://mt3.google.com/mt/v=w2t.83&hl=en&x=3&y=6&z=4&s=Galileo
        }
        else if ( dataset == "roads" )
        {
            char server = key->str().length() > 0? key->str()[key->str().length()-1] : '0';
            unsigned int tile_x, tile_y;
            mkey->getTileXY( tile_x, tile_y );
            int zoom = key->getLevelOfDetail();

            buf << "http://mt" << server << ".google.com/mt?v=w2.86&hl=en"
                << "&x=" << tile_x
                << "&y=" << tile_y
                << "&z=" << zoom
                << "&s=Ga&.png";
        }

        osg::notify(osg::INFO) 
            << "[osgEarth] Google: option string = "
            << (options.valid()? options->getOptionString() : "<empty>")
            << std::endl;

        std::string cache_path = map_config ? map_config->getFullCachePath() : std::string("");
        bool offline = map_config ? map_config->getOfflineHint() : false;

        osgEarth::FileCache fc(cache_path);
        fc.setOffline(offline);
        return fc.readImageFile( buf.str(), options.get() );
    }

    osg::HeightField* createHeightField( const TileKey* key )
    {
        //TODO
        return NULL;
    }

private:
    osg::ref_ptr<const osgDB::ReaderWriter::Options> options;
    std::string dataset;
    const MapConfig *map_config;
};


class ReaderWriterGoogle : public osgDB::ReaderWriter
{
    public:
        ReaderWriterGoogle()
        {
            supportsExtension( "google", "Google maps imagery" );
        }

        virtual const char* className()
        {
            return "Google Imagery ReaderWriter";
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            if ( osgDB::getLowerCaseFileExtension( file_name ) != "google" )
                return ReadResult::FILE_NOT_HANDLED;
            return new GoogleSource(options);
        }
};

REGISTER_OSGPLUGIN(google, ReaderWriterGoogle)
