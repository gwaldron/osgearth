/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include <osgEarth/ElevationLayer>

#include <osgEarthUtil/SimplexNoise>

#include <osgEarthDrivers/gdal/GDALOptions>
using namespace osgEarth::Drivers;

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>


#include "SplatMaskOptions"

using namespace osgEarth;
using namespace osgEarth::Drivers;

namespace osgEarth { namespace Drivers { namespace SplatMask
{
    class SplatMaskTileSource : public TileSource
    {
    public:
        SplatMaskTileSource( const TileSourceOptions& options ) : 
          TileSource( options ), 
          _options(options)
        {
            _noise.setOctaves( 24 );
        }

    public: // TileSource interface

        Status initialize(const osgDB::Options* dbOptions)
        {            
            _dbOptions = Registry::instance()->cloneOrCreateOptions( dbOptions );         
            setProfile( osgEarth::Registry::instance()->getGlobalGeodeticProfile() );

            GDALOptions gdal;
            gdal.url() = *_options.classificationPath();
            gdal.interpolation() = osgEarth::INTERP_NEAREST;
            gdal.bilinearReprojection() = false;
            gdal.tileSize() = 5;
            ElevationLayerOptions classOptions("splat", gdal);
            classOptions.cachePolicy() = CachePolicy::NO_CACHE;
            _classLayer = new ElevationLayer(classOptions);

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
            TileKey hfkey( key );
            GeoHeightField classTable;
            while( !classTable.valid() && hfkey.valid() )
            {
                classTable = _classLayer->createHeightField(hfkey, progress);
                if ( !classTable.valid() )
                    hfkey = hfkey.createParentKey();
            }

            if ( !classTable.valid() )
            {
                OE_WARN << "no classification. sorry." << std::endl;
                return 0L;
            }

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
                    {
                        srs->transform(world, srs->getECEF(), world);
                        world.normalize();
                    }

                    double n = _noise.getValue(world.x(), world.y(), world.z());

                    float r = n <  -0.5            ? 1.0f : 0.0f;
                    float g = n >= -0.5 && n < 0.0 ? 1.0f : 0.0f;
                    float b = n >=  0.0 && n < 0.5 ? 1.0f : 0.0f;
                    float a = n >=  0.5            ? 1.0f : 0.0f;

                    float rs = (float)s / (float)image->s();
                    float rt = (float)t / (float)image->t();

                    float elevation = 0.0f;
                
                    classTable.getElevation(
                        key.getExtent().getSRS(),
                        lon, lat, osgEarth::INTERP_NEAREST,
                        key.getExtent().getSRS(),
                        elevation);

                    int cv = (int)elevation;
                    float contrast = _options.contrast().get();

                    if ( cv > 220 ) {
                        r = g = b = a = 0.0f; // nodata
                    }
                    else if ( cv == 210 ) {
                        a += contrast;
                        r *= 0.1f, b *= 0.1f, g *= 0.1f;
                    }
                    else if ( cv < 130 ) {
                        b += contrast;
                        a = 0.0f;
                    }
                    else if ( cv >= 130 && cv <= 200 ) {
                        g += contrast;
                        a = 0.0f;
                    }
                    else {
                        r += contrast;
                        a = 0.0f;
                    }

                    osg::Vec4f value(r,g,b,a);
                    value /= r+g+b+a;

                    write(value, s, t);
                }
            }

            return image;
        }



    private:
        osg::ref_ptr<ElevationLayer> _classLayer;
        SplatMaskOptions             _options;
        osg::ref_ptr<osgDB::Options> _dbOptions;
        osgEarth::Util::SimplexNoise _noise;
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
                return "Detail Splat Mask Driver";
            }

            virtual ReadResult readObject(const std::string& file_name, const Options* options) const
            {
                if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
                    return ReadResult::FILE_NOT_HANDLED;

                return new SplatMaskTileSource( getTileSourceOptions(options) );
            }
    };

    REGISTER_OSGPLUGIN(osgearth_splat_mask, SplatMaskDriver)

} } } // namespace osgEarth::Drivers::SplatMask
