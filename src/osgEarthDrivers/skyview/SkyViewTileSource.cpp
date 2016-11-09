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
#include "SkyViewOptions"

#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>

#include <cstring>

#define LC "[SkyView Driver] "

using namespace osgEarth;
using namespace osgEarth::Drivers;


class SkyViewTileSource : public TileSource
{
public:
    SkyViewTileSource( const TileSourceOptions& options ) :
      TileSource( options ),            
      _options( options )
    {
        //nop
    }

    Status initialize( const osgDB::Options* dbOptions )
    {
        _dbOptions = Registry::instance()->cloneOrCreateOptions(dbOptions);        

        if (!_options.options().isSet())
        {
            return Status::Error(Status::ConfigurationError, "Please specify a image layer for the skyview driver.");
        }

        _source = TileSourceFactory::create( *_options.options());

        if (!_source.valid())
        {
            return Status::Error(Status::ServiceUnavailable, "Failed to load image layer for skyview driver");
        }

         // Open the tile source (if it hasn't already been started)
        Status status = _source->getStatus();
        if (status.isError())
        {
            status = _source->open(TileSource::MODE_READ, _dbOptions.get());
        }

        if (status.isError())
        {
            return status;
        }
        
        setProfile(_source->getProfile());           

        return osgEarth::STATUS_OK;
    }

    osg::Image*
    createImage( const TileKey& key, ProgressCallback* progress )
    {
        // We need to flip the image horizontally so that it's viewable from inside the globe.
        
        // Create a new key with the x coordinate flipped.
        unsigned int numRows, numCols;

        key.getProfile()->getNumTiles(key.getLevelOfDetail(), numCols, numRows);        
        unsigned int tileX = numCols - key.getTileX() -1;
        unsigned int tileY = key.getTileY();
        unsigned int level = key.getLevelOfDetail();

        osg::ref_ptr< osg::Image > image = _source->createImage(TileKey(level, tileX, tileY, key.getProfile()), 0, progress);
        if (image)
        {
            // If an image was read successfully, we still need to flip it horizontally
            image->flipHorizontal();
        }
        return image.release();      
    }
    

private:
    const SkyViewOptions _options;
    osg::ref_ptr< TileSource > _source;
    osg::ref_ptr<osgDB::Options>   _dbOptions;
};


class SkyViewTileSourceFactory : public TileSourceDriver
{
public:
    SkyViewTileSourceFactory()
    {
        supportsExtension( "osgearth_skyview", "SkyView driver for osgEarth" );
    }

    const char* className() const
    {
        return "SkyView Image Driver";
    }

    ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new SkyViewTileSource( getTileSourceOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_skyview, SkyViewTileSourceFactory)

