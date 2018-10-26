/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarth/Registry>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/FileUtils>
#include <osgEarth/URI>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include "TileCacheOptions"

#include <sstream>

using namespace osgEarth;
using namespace osgEarth::Drivers;


class TileCacheSource : public TileSource
{
public:
    TileCacheSource( const TileSourceOptions& options ) 
        : TileSource( options ), _options( options )
    {
    }

    Status initialize( const osgDB::Options* dbOptions )
    {
        _dbOptions = Registry::instance()->cloneOrCreateOptions(dbOptions);        

        if ( !getProfile() )
        {
            // Assume it is global-geodetic
            setProfile( osgEarth::Registry::instance()->getGlobalGeodeticProfile() );
        }

        return STATUS_OK;
    }

    osg::Image* createImage( const TileKey& key, ProgressCallback* progress)
    {
        unsigned int level, tile_x, tile_y;
        level = key.getLevelOfDetail();
        key.getTileXY( tile_x, tile_y );

        unsigned int numCols, numRows;
        key.getProfile()->getNumTiles(level, numCols, numRows);
        
        // need to invert the y-tile index
        tile_y = numRows - tile_y - 1;

        char buf[2048];
        sprintf( buf, "%s/%s/%02d/%03d/%03d/%03d/%03d/%03d/%03d.%s",
            _options.url()->full().c_str(),
            _options.layer()->c_str(),
            level,
            (tile_x / 1000000),
            (tile_x / 1000) % 1000,
            (tile_x % 1000),
            (tile_y / 1000000),
            (tile_y / 1000) % 1000,
            (tile_y % 1000),
            _options.format()->c_str() );

       
        std::string path(buf);
        return URI(path).readImage( _dbOptions.get(), progress ).releaseImage();
    }

    virtual std::string getExtension()  const 
    {
        return _options.format().value();
    }

private:
    const TileCacheOptions       _options;
    osg::ref_ptr<osgDB::Options> _dbOptions;
};

// Reads tiles from a TileCache disk cache.
class TileCacheSourceFactory : public TileSourceDriver
{
    public:
        TileCacheSourceFactory() {}

        virtual const char* className() const
        {
            return "TileCache disk cache ReaderWriter";
        }
        
        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive( extension, "osgearth_tilecache" );
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            std::string ext = osgDB::getFileExtension( file_name );
            if ( !acceptsExtension( ext ) )
            {
                return ReadResult::FILE_NOT_HANDLED;
            }

            return new TileCacheSource( getTileSourceOptions(options) );
        }
};

REGISTER_OSGPLUGIN(osgearth_tilecache, TileCacheSourceFactory)

