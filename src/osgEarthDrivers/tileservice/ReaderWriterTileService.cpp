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

#include "TileServiceOptions"

using namespace osgEarth;
using namespace osgEarth::Drivers;

//http://www.worldwindcentral.com/wiki/TileService
class TileServiceSource : public TileSource
{
public:
	TileServiceSource( const PluginOptions* options ) : TileSource( options )
    {
        _settings = dynamic_cast<const TileServiceOptions*>( options );
        if ( !_settings.valid() )
        {
            _settings = new TileServiceOptions( options );
        }
        
        _formatToUse =
            _settings->format()->empty() ? "png" : _settings->format().value();
    }

public:
    void initialize( const std::string& referenceURI, const Profile* overrideProfile)
    {
		//Take on the override profile if one is given.
		if (overrideProfile)
		{
			setProfile( overrideProfile );
		}
		else
		{
			//Assume it is global geodetic
			setProfile( osgEarth::Registry::instance()->getGlobalGeodeticProfile() );
		}
    }

public:
    osg::Image* createImage( const TileKey* key ,
                             ProgressCallback* progress)
    {
        //return osgDB::readImageFile(  createURI( key ), getOptions() );
        //return HTTPClient::readImageFile( createURI( key ), getOptions(), progress );
        
        osg::ref_ptr<osg::Image> image;
        HTTPClient::readImageFile( createURI( key ), image, getOptions(), progress );
        return image.release();
    }

    osg::HeightField* createHeightField( const TileKey* key,
                                         ProgressCallback* progress)
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
        buf << _settings->url().value() << "interface=map&version=1"
            << "&dataset=" << _settings->dataset().value()
            << "&level=" << lod
            << "&x=" << x
            << "&y=" << y
            << "&." << _formatToUse; //Add this to trick osg into using the correct loader.
        std::string bufStr;
		bufStr = buf.str();
		return bufStr;
    }

    virtual std::string getExtension()  const 
    {
        return _formatToUse;
    }

private:
    std::string _formatToUse;
    osg::ref_ptr<const TileServiceOptions> _settings;
};


class TileServiceSourceFactory : public osgDB::ReaderWriter
{
    public:
        TileServiceSourceFactory() {}

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

            return new TileServiceSource( static_cast<const PluginOptions*>(opt) );
        }
};

REGISTER_OSGPLUGIN(osgearth_tileservice, TileServiceSourceFactory)

