/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarth/URI>
#include <osgEarth/StringUtils>

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
	TileServiceSource( const TileSourceOptions& options ) : TileSource( options ), _options(options)
    {        
        _formatToUse =
            _options.format()->empty() ? "png" : _options.format().value();
    }

public:
    Status initialize( const osgDB::Options* dbOptions )
    {
        _dbOptions = Registry::instance()->cloneOrCreateOptions(dbOptions);
        CachePolicy::NO_CACHE.apply( _dbOptions.get() );

        if ( !getProfile() )
        {
            // Assume it is global geodetic
            setProfile( osgEarth::Registry::instance()->getGlobalGeodeticProfile() );
        }

        return STATUS_OK;
    }

public:
    osg::Image* createImage(
        const TileKey&        key,
        ProgressCallback*     progress )
    {        
        return URI( createURL(key) ).readImage( _dbOptions.get(), progress ).releaseImage();
    }

    osg::HeightField* createHeightField(
        const TileKey&        key,
        ProgressCallback*     progress )
    {
        //NOP
        return NULL;
    }

    std::string createURL( const TileKey& key ) const
    {
        unsigned int x, y;
        key.getTileXY(x, y);

        unsigned int lod = key.getLevelOfDetail()+1;

        //http://s0.tileservice.worldwindcentral.com/getTile?interface=map&version=1&dataset=bmng.topo.bathy.200401&level=0&x=0&y=0
        return Stringify() 
            << _options.url()->full() << "interface=map&version=1"
            << "&dataset=" << _options.dataset().value()
            << "&level=" << lod
            << "&x=" << x
            << "&y=" << y
            << "&." << _formatToUse; //Add this to trick osg into using the correct loader.
    }

    virtual std::string getExtension()  const 
    {
        return _formatToUse;
    }

private:
    std::string                  _formatToUse;
    const TileServiceOptions     _options;
    osg::ref_ptr<osgDB::Options> _dbOptions;
};


class TileServiceSourceFactory : public TileSourceDriver
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

            return new TileServiceSource( getTileSourceOptions(opt) );
        }
};

REGISTER_OSGPLUGIN(osgearth_tileservice, TileServiceSourceFactory)

