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
#include <osgEarth/Registry>
#include <osgEarth/URI>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <sstream>
#include <iomanip>

using namespace osgEarth;

#define PROPERTY_URL        "url"
#define PROPERTY_MAP        "map"
#define PROPERTY_LAYER      "layer"
#define PROPERTY_FORMAT     "format"

class AGSMapCacheSource : public TileSource
{
public:
    AGSMapCacheSource( const TileSourceOptions& options ) :
      TileSource( options )
    {
        const Config& conf = options.getConfig();

        // this is the AGS virtual directory pointing to the map cache
        _url = conf.value( PROPERTY_URL );

        // the name of the map service cache
        _map = conf.value( PROPERTY_MAP );

        // the layer, or null to use the fused "_alllayers" cache
        _layer = conf.value( PROPERTY_LAYER );

        // the image format (defaults to "png")
        // TODO: read this from the XML tile schema file
        _format = conf.value( PROPERTY_FORMAT );

        // validate dataset
        if ( _layer.empty() )
            _layer = "_alllayers"; // default to the AGS "fused view"

        if ( _format.empty() )
            _format = "png";
    }

    Status initialize( const osgDB::Options* dbOptions )
    {
        _dbOptions = Registry::instance()->cloneOrCreateOptions( dbOptions );
        CachePolicy::NO_CACHE.apply( _dbOptions.get() );

        //Set the profile to global geodetic.
        setProfile(osgEarth::Registry::instance()->getGlobalGeodeticProfile());

        return STATUS_OK;
    }

    // override
    osg::Image* createImage( const TileKey& key, ProgressCallback* progress)
    {
        std::stringstream buf;

        int level = key.getLevelOfDetail()-1;

        unsigned int tile_x, tile_y;
        key.getTileXY( tile_x, tile_y );

        std::string bufStr = Stringify()
            << _url << "/" << _map 
            << "/Layers/" << _layer
            << "/L" << std::hex << std::setw(2) << std::setfill('0') << level
            << "/R" << std::hex << std::setw(8) << std::setfill('0') << tile_y
            << "/C" << std::hex << std::setw(8) << std::setfill('0') << tile_x << "." << _format;
        
        osg::ref_ptr<osg::Image> image;

        return URI(bufStr).getImage( _dbOptions.get(), progress );
    }

    // override
    osg::HeightField* createHeightField( const TileKey& key, ProgressCallback* progress)
    {
        //TODO
        return NULL;
    }

    // override
    virtual std::string getExtension()  const 
    {
        return _format;
    }

private:
    std::string _url;
    std::string _map;
    std::string _layer;
    std::string _format;
    osg::ref_ptr<osgDB::Options> _dbOptions;
};


class ReaderWriterAGSMapCache : public TileSourceDriver
{
    public:
        ReaderWriterAGSMapCache()
        {
            supportsExtension( "osgearth_arcgis_map_cache", "ArcGIS Server Map Service Cache" );
        }

        virtual const char* className()
        {
            return "ArcGIS Server Map Service Cache Imagery ReaderWriter";
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
                return ReadResult::FILE_NOT_HANDLED;

            return new AGSMapCacheSource( getTileSourceOptions(options) );
        }
};

REGISTER_OSGPLUGIN(osgearth_arcgis_map_cache, ReaderWriterAGSMapCache)

