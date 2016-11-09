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
#include <osgEarth/URI>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <sstream>

#include "YahooOptions"

using namespace osgEarth;
using namespace osgEarth::Drivers;

class YahooSource : public TileSource
{
public:
    YahooSource( const TileSourceOptions& options ) : TileSource( options ), _options(options)
    {
        //nop
    }

    // Yahoo! uses spherical mercator, but the top LOD is a 2x2 tile set.
    Status initialize(const osgDB::Options* readOptions)
    {
        // no caching of source tiles.  Yahoo TOS does not allow it.
        _readOptions = readOptions;

        // always a sperhical mercator profile
        setProfile( Profile::create( "spherical-mercator", "", 2, 2 ) );

        return STATUS_OK;
    }

    osg::Image* createImage(const TileKey& key, ProgressCallback* progress)
    {
        std::stringstream buf;

        std::string dataset = 
            _options.dataset().isSet() ? _options.dataset().value() : "roads";
        
        if ( dataset == "roads" || dataset == "map" )
        {            
            // http://us.maps1.yimg.com/us.tile.maps.yimg.com/tl?v=4.1&md=2&x=0&y=0&z=2&r=1
            unsigned int tile_x, tile_y;
            key.getTileXY( tile_x, tile_y );
            unsigned int zoom = key.getLevelOfDetail();
            unsigned int size_x, size_y;
            key.getProfile()->getNumTiles( zoom, size_x, size_y );

            buf << "http://us.maps1.yimg.com/us.tile.maps.yimg.com/tl"
                << "?v=4.1&md=2&r=1"
                << "&x=" << (int)tile_x
                << "&y=" << ((int)size_y-1-(int)tile_y) - (int)size_y/2
                << "&z=" << zoom + 2;
        }
        else if ( dataset == "aerial" || dataset == "satellite" )
        {
            unsigned int tile_x, tile_y;
            key.getTileXY( tile_x, tile_y );
            unsigned int zoom = key.getLevelOfDetail();
            unsigned int size_x, size_y;
            key.getProfile()->getNumTiles( zoom, size_x, size_y );

            buf << "http://us.maps3.yimg.com/aerial.maps.yimg.com/ximg"
                << "?v=1.8&s=256&t=a&r=1"
                << "&x=" << (int)tile_x
                << "&y=" << ((int)size_y-1-(int)tile_y) - (int)size_y/2
                << "&z=" << zoom + 2;
        }

        std::string base;
        base = buf.str();

        OE_DEBUG << key.str() << "=" << base << std::endl;

        return URI(base).readImage( _readOptions.get() ).releaseImage();
    }

    osg::HeightField* createHeightField(const TileKey&        key,
                                        ProgressCallback*     progress )
    {
        //NI
        OE_WARN << "[Yahoo] Driver does not support heightfields" << std::endl;
        return NULL;
    }

    std::string getExtension()  const 
    {
        //All Yahoo tiles are in JPEG format
        return "jpg";
    }
    
    /** Tell the terrain engine not to cache tiles form this source. */
    CachePolicy getCachePolicyHint(const Profile*) const
    {
        return CachePolicy::NO_CACHE;
    }

private:
    const YahooOptions                 _options;
    osg::ref_ptr<const osgDB::Options> _readOptions;
};


class ReaderWriterYahoo : public TileSourceDriver
{
    public:
        ReaderWriterYahoo()
        {
            supportsExtension( "osgearth_yahoo", "Yahoo maps data" );
        }

        virtual const char* className() const
        {
            return "Yahoo Imagery ReaderWriter";
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
                return ReadResult::FILE_NOT_HANDLED;

            return new YahooSource( getTileSourceOptions(options) );
        }
};

REGISTER_OSGPLUGIN(osgearth_yahoo, ReaderWriterYahoo)

