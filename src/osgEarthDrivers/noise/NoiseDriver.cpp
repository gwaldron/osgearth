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

#include <osgEarthUtil/SimplexNoise>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <sstream>
#include <cmath>

#include "NoiseOptions"

using namespace osgEarth;
using namespace osgEarth::Util;

namespace osgEarth { namespace Drivers { namespace Noise
{
    class NoiseSource : public TileSource
    {
    public:
        NoiseSource( const TileSourceOptions& options ) : TileSource( options ), _options(options)
        {
            _noise.setFrequency  ( *_options.frequency() );
            _noise.setPersistence( *_options.persistence() );
            _noise.setLacunarity ( *_options.lacunarity() );
            _noise.setOctaves    ( *_options.octaves() );
            _noise.setRange      ( -1.0, 1.0 );
        }

        // Yahoo! uses spherical mercator, but the top LOD is a 2x2 tile set.
        Status initialize(const osgDB::Options* dbOptions)
        {            
            _dbOptions = Registry::instance()->cloneOrCreateOptions( dbOptions );            
            setProfile( osgEarth::Registry::instance()->getGlobalGeodeticProfile() );

            // resolve frequency if the user set resolution
            if (_options.resolution().isSet() && !_options.resolution().isSetTo(0.0))
            {
                _noise.setFrequency( 1.0 / *_options.resolution() );
            }

            return STATUS_OK;
        }
    
        /** Tell the terrain engine not to cache tiles form this source by default. */
        CachePolicy getCachePolicyHint(const Profile*) const
        {
            return CachePolicy::NO_CACHE;
        }

        inline double sample(double x, double y, double z)
        {
            return _noise.getValue(x, y, z);
        }

        inline double sample(const osg::Vec3d& v)
        {
            return _noise.getValue(v.x(), v.y(), v.z());
        }

        inline double turbulence(const osg::Vec3d& v, double f)
        {
            double t = -0.5;
            for( ; f<getPixelsPerTile()/2; f *= 2 ) 
                t += std::abs(_noise.getValue(v.x(), v.y(), v.z())/f);
            return t;
        }

        inline double stripes(double x, double f)
        {
            double t = 0.5 + 0.5 * asin(f * 2*osg::PI * x);
            return t * t - 0.5;
        }


        osg::Image* createImage(const TileKey&    key,
                                ProgressCallback* progress)
        {
            if ( _options.normalMap() == true )
            {
                return createNormalMap(key, progress);
            }
            else
            {
                const SpatialReference* srs = key.getProfile()->getSRS();

                double projNormX, projNormY;
                if ( srs->isProjected() )
                {
                    projNormX = 1.0/key.getProfile()->getExtent().width();
                    projNormY = 1.0/key.getProfile()->getExtent().height();
                }

                osg::Image* image = new osg::Image();
                image->allocateImage( getPixelsPerTile(), getPixelsPerTile(), 1, GL_RGB, GL_UNSIGNED_BYTE );

                double dx = key.getExtent().width()  / (double)(image->s()-1);
                double dy = key.getExtent().height() / (double)(image->t()-1);

                ImageUtils::PixelWriter write(image);
                for(int s=0; s<image->s(); ++s)
                {
                    for(int t=0; t<image->t(); ++t)
                    {
                        double x = key.getExtent().xMin() + (double)s * dx;
                        double y = key.getExtent().yMin() + (double)t * dy;
                        
                        osg::Vec3d world(x, y, 0.0);

                        if ( srs->isGeographic() )
                        {
                            srs->transform(world, srs->getECEF(), world);
                            world.normalize();
                        }
                        else
                        {
                            world.x() *= projNormX;
                            world.y() *= projNormY;
                            world.z()  = 0.0;
                        }

                        double n = sample(world); //world.x(), world.y(), world.z());
                        //double n = 0.1 * stripes(world.x() + 2.0*turbulence(noise, world, 1.0), 1.6);
                        //double n = -.10 * turbulence(noise, world, 0.2);

                        // scale and bias from[-1..1] to [0..1] for coloring.
                        n = osg::clampBetween( (n+1.0)*0.5, 0.0, 1.0 );

                        write(osg::Vec4f(n,n,n,1), s, t);
                    }
                }

                return image;
            }
        }


        osg::HeightField* createHeightField(const TileKey&        key,
                                            ProgressCallback*     progress )
        {
            const SpatialReference* srs = key.getProfile()->getSRS();

            osg::HeightField* hf = new osg::HeightField();
            hf->allocate( getPixelsPerTile(), getPixelsPerTile() );

            double dx = key.getExtent().width() / (double)(hf->getNumColumns()-1);
            double dy = key.getExtent().height() / (double)(hf->getNumRows()-1);

            double bias  = _options.bias().get();
            double scale = _options.scale().get();
        
            //Initialize the heightfield
            for (unsigned int c = 0; c < hf->getNumColumns(); c++) 
            {
                for (unsigned int r = 0; r < hf->getNumRows(); r++)
                {                
                    double lon = key.getExtent().xMin() + (double)c * dx;
                    double lat = key.getExtent().yMin() + (double)r * dy;

                    osg::Vec3d world(lon, lat, 0.0);
                    if ( srs->isGeographic() )
                    {
                        srs->transform(world, srs->getECEF(), world);
                        world.normalize();
                    }

                    double n = sample( world );

                    // Scale the noise value.
                    double h = osg::clampBetween(
                        (float)(bias + scale * n),
                        *_options.minElevation(),
                        *_options.maxElevation() );

                    hf->setHeight( c, r, h );

                    // NOTE! The elevation engine treats extreme values (>32000, etc)
                    // as "no data" so be careful with your scale.
                }
            }     

            return hf;
        }


        osg::Image* createNormalMap(const TileKey& key, ProgressCallback* progress)
        {
            // set up the image and prepare to write to it.
            osg::Image* image = new osg::Image();
            image->allocateImage( getPixelsPerTile(), getPixelsPerTile(), 1, GL_RGB, GL_UNSIGNED_BYTE );
            ImageUtils::PixelWriter write(image);

            const GeoExtent&        ex     = key.getExtent();
            const SpatialReference* srs    = ex.getSRS();
            bool                    isGeo  = srs->isGeographic();
            const SpatialReference* ecef   = srs->getECEF();

            double dx = ex.width()  / (double)(image->s()-1);
            double dy = ex.height() / (double)(image->t()-1);

            double scale  = _options.scale().get();
            double bias   = _options.bias().get();

            // figure out the spacing between pixels in the same units as the height value:
            double udx = dx;
            double udy = dy;
            if ( isGeo )
            {
                udx = srs->transformUnits(dx, ecef, ex.south()+0.5*dy);
                udy = srs->transformUnits(dy, ecef, ex.south()+0.5*dy);
            }

            double z = 0.0;
            std::vector<osg::Vec3d> v(4);
            double samples[4];

            for(int s=0; s<image->s(); ++s)
            {
                for(int t=0; t<image->t(); ++t)
                {
                    double x = ex.xMin() + (double)s * dx;
                    double y = ex.yMin() + (double)t * dy;

                    if ( isGeo )
                    {
                        v[0].set(x-dx, y, z);
                        v[1].set(x+dx, y, z);
                        v[2].set(x, y+dy, z);
                        v[3].set(x, y-dy, z);
                        srs->transform(v, ecef);
                        for(int i=0; i<4; ++i )
                            samples[i] = bias + scale * sample(v[i]);
                    }
                    else
                    {
                        samples[0] = bias + scale * sample(x-dx, y, z);
                        samples[1] = bias + scale * sample(x+dx, y, z);
                        samples[2] = bias + scale * sample(x, y+dy, z);
                        samples[3] = bias + scale * sample(x, y-dy, z);
                    }

                    osg::Vec3d west (-udx,    0, samples[0]);
                    osg::Vec3d east ( udx,    0, samples[1]);
                    osg::Vec3d north(   0,  udy, samples[2]);
                    osg::Vec3d south(   0, -udy, samples[3]);

                    // calculate the normal at the center point.
                    osg::Vec3 normal = (east-west) ^ (north-south);
                    normal.normalize();

                    // encode as: xyz[-1..1]=>r[0..255]. (In reality Z will always fall 
                    // between [0..1] but a uniform encoding makes the shader code simpler.)
                    normal.x() = normal.x()*0.5f + 0.5f;
                    normal.y() = normal.y()*0.5f + 0.5f;
                    normal.z() = normal.z()*0.5f + 0.5f;
                    normal.normalize();

                    write(osg::Vec4f(normal,1), s, t);
                }
            }

            return image;
        }


    private:
        NoiseOptions                 _options;
        osg::ref_ptr<osgDB::Options> _dbOptions;
        SimplexNoise                 _noise;
    };


    class NoiseDriver : public TileSourceDriver
    {
        public:
            NoiseDriver()
            {
                supportsExtension( "osgearth_noise", "Procedural noise generator" );
            }

            virtual const char* className()
            {
                return "Procedural noise generator";
            }

            virtual ReadResult readObject(const std::string& file_name, const Options* options) const
            {
                if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
                    return ReadResult::FILE_NOT_HANDLED;

                return new NoiseSource( getTileSourceOptions(options) );
            }
    };

    REGISTER_OSGPLUGIN(osgearth_noise, NoiseDriver);

} } } // namespace osgEarth::Drivers::Noise
