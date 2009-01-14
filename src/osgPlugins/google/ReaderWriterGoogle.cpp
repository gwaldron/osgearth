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

            buf << "http://khm" << server << ".google.com/kh?v=34&hl=en"
                << "&x=" << tile_x
                << "&y=" << tile_y
                << "&z=" << zoom
                << "&s=Ga&.jpg";
        }
        else if ( dataset == "terrain" )
        {
            char server = key->str().length() > 0? key->str()[key->str().length()-1] : '0';
            unsigned int tile_x, tile_y;
            mkey->getTileXY( tile_x, tile_y );
            buf << "http://mt" << server << ".google.com/mt?v=w2p.87&hl=en&x="
                << tile_x << "&y=" << tile_y << "&zoom=" 
                << 17-key->getLevelOfDetail() << "&.jpg";
        }
        else if ( dataset == "labels" )
        {
            char server = key->str().length() > 0? key->str()[key->str().length()-1] : '0';
            unsigned int tile_x, tile_y;
            mkey->getTileXY( tile_x, tile_y );
            int zoom = key->getLevelOfDetail();

            buf << "http://mt" << server << ".google.com/mt?v=w2t.88&hl=en"
                << "&x=" << tile_x
                << "&y=" << tile_y
                << "&z=" << zoom
                << "&s=G&.png";
        }
        else if ( dataset == "roads" )
        {
            char server = key->str().length() > 0? key->str()[key->str().length()-1] : '0';
            unsigned int tile_x, tile_y;
            mkey->getTileXY( tile_x, tile_y );
            int zoom = key->getLevelOfDetail();

            buf << "http://mt" << server << ".google.com/mt?v=w2.89&hl=en"
                << "&x=" << tile_x
                << "&y=" << tile_y
                << "&z=" << zoom
                << "&s=Ga&.png";
        }

        osg::notify(osg::INFO) 
            << "[osgEarth] Google: option string = "
            << (options.valid()? options->getOptionString() : "<empty>")
            << std::endl;

        //If we are in offline mode, don't connect to the web
        if (osgDB::containsServerAddress( buf.str()) && map_config->getOfflineHint()) return 0;

        return osgDB::readImageFile ( buf.str(), options.get());
    }

    osg::HeightField* createHeightField( const TileKey* key )
    {
        //TODO
        return NULL;
    }

    virtual std::string getExtension()  const 
    {
        if ( dataset == "satellite" ) return "jpg";
        else if ( dataset == "terrain" ) return "jpg";
        else if ( dataset == "labels" ) return "png";
        else if ( dataset == "roads" ) return "png";
        else return "";
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
            supportsExtension( "osgearth_google", "Google maps imagery" );
        }

        virtual const char* className()
        {
            return "Google Imagery ReaderWriter";
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
                return ReadResult::FILE_NOT_HANDLED;
            return new GoogleSource(options);
        }
};

REGISTER_OSGPLUGIN(osgearth_google, ReaderWriterGoogle)
