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

#include <osgEarth/Mercator>
#include <osgEarth/TileSource>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/Registry>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <sstream>
#include <stdlib.h>
#include <iomanip>

using namespace osgEarth;

#define PROPERTY_URL            "url"
#define PROPERTY_DATASET        "dataset"
#define PROPERTY_FORMAT         "format"

//http://www.worldwindcentral.com/wiki/TileService
class TileServiceSource : public TileSource
{
public:
	TileServiceSource( const osgDB::ReaderWriter::Options* options ):
    TileSource( options )
    {
        if ( options->getPluginData( PROPERTY_URL ) )
            _url = std::string( (const char*)options->getPluginData( PROPERTY_URL ) );

        if (options->getPluginData( PROPERTY_FORMAT ))
            _format = std::string( (const char*)options->getPluginData( PROPERTY_FORMAT ) );

        if (options->getPluginData( PROPERTY_DATASET ))
            _dataset = std::string( (const char*)options->getPluginData( PROPERTY_DATASET ) );

         if ( _format.empty() )
            _format = "png";
    }

public:
    const Profile* createProfile( const Profile* mapProfile, const std::string& configPath )
    {
        return osgEarth::Registry::instance()->getGlobalGeodeticProfile();
    }

public:
    osg::Image* createImage( const TileKey* key )
    {
        return osgDB::readImageFile(  createURI( key ), getOptions() );
    }

    osg::HeightField* createHeightField( const TileKey* key )
    {
        //NOP
        return NULL;
    }

    std::string createURI( const TileKey* key ) const
    {
        unsigned int x, y;
        key->getTileXY(x, y);

        unsigned int lod = key->getLevelOfDetail()+1;

        std::stringstream buf;
        //http://s0.tileservice.worldwindcentral.com/getTile?interface=map&version=1&dataset=bmng.topo.bathy.200401&level=0&x=0&y=0
        buf << _url << "interface=map&version=1"
            << "&dataset=" << _dataset
            << "&level=" << lod
            << "&x=" << x
            << "&y=" << y
            << "&." << _format;//Add this to trick osg into using the correct loader.
        return buf.str();
    }

    virtual int getPixelsPerTile() const
    {
        return 256;
    }

    virtual std::string getExtension()  const 
    {
        return _format;
    }

private:
    std::string _url;
    std::string _dataset;
    std::string _format;
};


class ReaderWriterTileService : public osgDB::ReaderWriter
{
    public:
        ReaderWriterTileService() {}

        virtual const char* className()
        {
            return "TileService Reader";
        }
        
        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive( extension, "osgearth_tileservice" );
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* opt) const
        {
            std::string ext = osgDB::getFileExtension( file_name );
            if ( !acceptsExtension( ext ) )
            {
                return ReadResult::FILE_NOT_HANDLED;
            }

            return new TileServiceSource(opt);
        }
};

REGISTER_OSGPLUGIN(osgearth_tileservice, ReaderWriterTileService)
