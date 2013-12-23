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
#include <osgEarth/ImageUtils>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <sstream>

#include <noise/noise.h>

using namespace noise;

#include "SplatMaskOptions"

using namespace osgEarth;
using namespace osgEarth::Drivers;

class SplatMaskTileSource : public TileSource
{
public:
    SplatMaskTileSource( const TileSourceOptions& options ) : 
      TileSource( options ), 
      _options(options)
    {
        //nop
    }

public: // TileSource interface

    Status initialize(const osgDB::Options* dbOptions)
    {
        // no caching of source tiles (there are none..)
        _dbOptions = Registry::instance()->cloneOrCreateOptions( dbOptions );
        CachePolicy::NO_CACHE.apply( _dbOptions.get() );
        setProfile( osgEarth::Registry::instance()->getGlobalGeodeticProfile() );

        return STATUS_OK;
    }
    
    /** Tell the terrain engine not to cache tiles form this source by default. */
    CachePolicy getCachePolicyHint(const Profile*) const
    {
        return CachePolicy::NO_CACHE;
    }

    osg::Image* createImage(const TileKey&        key,
                            ProgressCallback*     progress)
    {
        module::Perlin noise;
        noise.SetFrequency( 1.0e-3 );
        noise.SetOctaveCount( 10 );
        noise.SetPersistence( 0.49 );
        noise.SetLacunarity( 3.0 );

        const SpatialReference* srs = key.getProfile()->getSRS();

        osg::Image* image = new osg::Image();
        image->allocateImage( getPixelsPerTile(), getPixelsPerTile(), 1, GL_RGBA, GL_UNSIGNED_BYTE );

        double dx = key.getExtent().width()  / (double)(image->s()-1);
        double dy = key.getExtent().height() / (double)(image->t()-1);

        ImageUtils::PixelWriter write(image);
        for(int s=0; s<image->s(); ++s)
        {
            for(int t=0; t<image->t(); ++t)
            {
                double lon = key.getExtent().xMin() + (double)s * dx;
                double lat = key.getExtent().yMin() + (double)t * dy;

                osg::Vec3d world(lon, lat, 0.0);
                if ( srs->isGeographic() )
                    srs->transform(world, srs->getECEF(), world);

                double n = noise.GetValue(world.x(), world.y(), world.z());

                float r = n <  -0.5            ? 1.0f : 0.0f;
                float g = n >= -0.5 && n < 0.0 ? 1.0f : 0.0f;
                float b = n >=  0.0 && n < 0.5 ? 1.0f : 0.0f;
                float a = n >=  0.5            ? 1.0f : 0.0f;

                write(osg::Vec4f(r,g,b,a), s, t);
            }
        }

        return image;
    }



private:
    SplatMaskOptions             _options;
    osg::ref_ptr<osgDB::Options> _dbOptions;
};


class SplatMaskDriver : public TileSourceDriver
{
    public:
        SplatMaskDriver()
        {
            supportsExtension( "osgearth_splat_mask", "Detail texture splat mask generator" );
        }

        virtual const char* className()
        {
            return "SplatMaks Driver";
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
                return ReadResult::FILE_NOT_HANDLED;

            return new SplatMaskTileSource( getTileSourceOptions(options) );
        }
};

REGISTER_OSGPLUGIN(osgearth_splat_mask, SplatMaskDriver)
