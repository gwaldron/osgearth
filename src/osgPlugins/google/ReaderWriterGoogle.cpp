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
#define PROPERTY_VERSION    "version"
#define PROPERTY_LANGUAGE   "language"
#define PROPERTY_MAP_CONFIG "map_config"

class GoogleSource : public TileSource
{
public:
    GoogleSource( const osgDB::ReaderWriter::Options* _options ) :
      options( _options ),
      map_config(0),
      profile( TileGridProfile::GLOBAL_MERCATOR )
    {
        if ( options.valid() )
        {
            if ( options->getPluginData( PROPERTY_DATASET ) )
                dataset = std::string( (const char*)options->getPluginData( PROPERTY_DATASET ) );

            if ( options->getPluginData( PROPERTY_VERSION ) )
                version = std::string( (const char*)options->getPluginData( PROPERTY_VERSION ) );

            if ( options->getPluginData( PROPERTY_LANGUAGE ) )
                language = std::string( (const char*)options->getPluginData( PROPERTY_LANGUAGE ) );
            else
                language = "en";

            if (options->getPluginData( PROPERTY_MAP_CONFIG ))
                map_config = (const MapConfig*)options->getPluginData(PROPERTY_MAP_CONFIG);
        }

        // validate dataset
        if ( dataset.empty() ) dataset = "satellite"; // defaul to the satellite view
    }

    const TileGridProfile& getProfile() const
    {
        return profile;
    }

    char getRandomServer()
    {
        //Gets a server from 0 - 3
        int server = rand() % 4;
        char serverChar[2];
        sprintf(serverChar, "%i", server);
        return serverChar[0];
    }

    osg::Image* createImage( const TileKey* key )
    {
        //Return NULL if we are given a non-mercator key
        if ( !key->isMercator() ) return 0;

        std::stringstream buf;
        
        if ( dataset == "satellite" )
        {            
            if ( version.empty() )
                version = "37";

            char server = getRandomServer();
            unsigned int tile_x, tile_y;
            key->getTileXY( tile_x, tile_y );
            int zoom = key->getLevelOfDetail();

            buf << "http://khm" << server << ".google.com/kh"
                << "?v="  << version
                << "&hl=" << language
                << "&x="  << tile_x
                << "&y="  << tile_y
                << "&z="  << zoom
                << "&s=Ga&.jpg";
        }
        else if ( dataset == "traffic" )
        {
            char server = getRandomServer();
            unsigned int tile_x, tile_y;
            key->getTileXY( tile_x, tile_y );
            int zoom = key->getLevelOfDetail();

            buf << "http://mt" << server << ".google.com/mapstt"
                << "?zoom=" << zoom
                << "&x="  << tile_x
                << "&y="  << tile_y
                << "&.png";
        }
        else if ( dataset == "terrain" )
        {
            if ( version.empty() )
                version = "w2p.87";

            char server = getRandomServer();
            unsigned int tile_x, tile_y;
            key->getTileXY( tile_x, tile_y );
            buf << "http://mt" << server << ".google.com/mt"
                << "?v="  << version
                << "&hl=" << language
                << "&x="  << tile_x
                << "&y="  << tile_y
                << "&zoom=" << 17-key->getLevelOfDetail()
                << "&.jpg";
        }
        else if ( dataset == "labels" )
        {
            if ( version.empty() )
                version = "w2t.92";

            char server = getRandomServer();
            unsigned int tile_x, tile_y;
            key->getTileXY( tile_x, tile_y );
            int zoom = key->getLevelOfDetail();

            buf << "http://mt" << server << ".google.com/mt"
                << "?v="  << version
                << "&hl=" << language
                << "&x="  << tile_x
                << "&y="  << tile_y
                << "&z="  << zoom
                << "&s=G&.png";
        }
        else if ( dataset == "roads" )
        {
            if ( version.empty() )
                version = "w2.92";

            char server = getRandomServer();
            unsigned int tile_x, tile_y;
            key->getTileXY( tile_x, tile_y );
            int zoom = key->getLevelOfDetail();

            buf << "http://mt" << server << ".google.com/mt"
                << "?v="  << version
                << "&hl=" << language
                << "&x="  << tile_x
                << "&y="  << tile_y
                << "&z="  << zoom
                << "&s=Ga&.png";
        }

        osg::notify(osg::INFO) 
            << "[osgEarth] Google: option string = "
            << (options.valid()? options->getOptionString() : "<empty>")
            << std::endl;

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
        else if ( dataset == "traffic" ) return "png";
        else return "";
    }

private:
    osg::ref_ptr<const osgDB::ReaderWriter::Options> options;
    std::string dataset;
    std::string version;
    std::string language;
    TileGridProfile profile;
    const MapConfig* map_config;
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
