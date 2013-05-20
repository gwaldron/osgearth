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

#include "NoiseOptions"

using namespace osgEarth;
using namespace osgEarth::Drivers;

class NoiseSource : public TileSource
{
public:
    NoiseSource( const TileSourceOptions& options ) : TileSource( options ), _options(options)
    {
        //nop
    }

    // Yahoo! uses spherical mercator, but the top LOD is a 2x2 tile set.
    Status initialize(const osgDB::Options* dbOptions)
    {
        // no caching of source tiles
        _dbOptions = Registry::instance()->cloneOrCreateOptions( dbOptions );
        CachePolicy::NO_CACHE.apply( _dbOptions.get() );

        //Setup the noise module with the specified options
        if ( _options.seed().isSet() )
        {
            _noise.SetSeed( *_options.seed() );
        }

        if ( _options.octaves().isSet() )
        {
            _noise.SetOctaveCount( *_options.octaves());
        }

        if ( _options.frequency().isSet() )
        {
            _noise.SetFrequency( *_options.frequency() );
        }

        if ( _options.persistence().isSet() )
        {
            _noise.SetPersistence( *_options.persistence() );
        }

        if ( _options.lacunarity().isSet() )
        {
            _noise.SetLacunarity( *_options.lacunarity() );
        }        
        
        setProfile( osgEarth::Registry::instance()->getGlobalGeodeticProfile() );

        return STATUS_OK;
    }
    
    /** Tell the terrain engine not to cache tiles form this source. */
    CachePolicy getCachePolicyHint() const
    {
        return CachePolicy::NO_CACHE;
    }

    osg::Image* createImage(const TileKey&        key,
                            ProgressCallback*     progress )
    {
        if ( _options.normalMap() == true )
        {
            return createNormalMap(key, progress);
        }
        else
        {
            osg::Image* image = new osg::Image();
            image->allocateImage( getPixelsPerTile(), getPixelsPerTile(), 1, GL_RGB, GL_UNSIGNED_BYTE );

            double dx = key.getExtent().width()  / (double)(image->s()-1);
            double dy = key.getExtent().height() / (double)(image->t()-1);

            ImageUtils::PixelWriter write(image);
            for(int s=0; s<image->s(); ++s)
            {
                for(int t=0; t<image->t(); ++t)
                {
                    double lon = key.getExtent().xMin() + (double)s * dx;
                    double lat = key.getExtent().yMin() + (double)t * dy;
                    double n = _noise.GetValue(lon, lat, 0.75);
                    n = osg::clampBetween( (n + 1.0) / 2.0, 0.0, 1.0 );

                    write(osg::Vec4f(n,n,n,1), s, t);
                }
            }

            return image;
        }
    }


    osg::HeightField* createHeightField(const TileKey&        key,
                                        ProgressCallback*     progress )
    {       
        osg::HeightField* hf = new osg::HeightField();
        hf->allocate( getPixelsPerTile(), getPixelsPerTile() );

        double dx = key.getExtent().width() / (double)(hf->getNumColumns()-1);
        double dy = key.getExtent().height() / (double)(hf->getNumRows()-1);

        double scale = *_options.maxElevation() - *_options.minElevation();

        //Initialize the heightfield
        for (unsigned int c = 0; c < hf->getNumColumns(); c++) 
        {
            for (unsigned int r = 0; r < hf->getNumRows(); r++)
            {                
                double lon = key.getExtent().xMin() + (double)c * dx;
                double lat = key.getExtent().yMin() + (double)r * dy;
                double n = _noise.GetValue(lon, lat, 0.75);             
                //Normalize the noise value between 0 and 1 instead of between -1 and 1
                n = (n + 1.0) / 2.0;

                //Scale the noise value which is between -1 and 1 between the min and max elevations
                double h = *_options.minElevation() + scale * n;
                hf->setHeight( c, r, h );
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

        // figure out the spacing between pixels in the same units as the height value:
        const GeoExtent& ex = key.getExtent();
        double dx = ex.width()  / (double)(image->s()-1);
        double dy = ex.height() / (double)(image->t()-1);
        double udx = dx;
        double udy = dy;
        const SpatialReference* srs  = ex.getSRS();
        if ( srs->isGeographic() )
        {
            const SpatialReference* ecef = srs->getECEF();
            udx = srs->transformUnits(dx, ecef, ex.south()+0.5*dy);
            udy = srs->transformUnits(dy, ecef, ex.south()+0.5*dy);
        }

        double a = *_options.minElevation();
        double b = *_options.maxElevation() - *_options.minElevation();
        double z = 0.75;

        for(int s=0; s<image->s(); ++s)
        {
            for(int t=0; t<image->t(); ++t)
            {
                double x = key.getExtent().xMin() + (double)s * dx;
                double y = key.getExtent().yMin() + (double)t * dy;

                // sample the surrounding locations. Obviously we could optimize
                // this so we only sample each location once per image.
                osg::Vec3 west (-udx, 0, a + b*(0.5*(_noise.GetValue(x-dx, y, z)+1.0)));
                osg::Vec3 east ( udx, 0, a + b*(0.5*(_noise.GetValue(x+dx, y, z)+1.0)));
                osg::Vec3 north( 0, udy, a + b*(0.5*(_noise.GetValue(x, y+dy, z)+1.0)));
                osg::Vec3 south( 0,-udy, a + b*(0.5*(_noise.GetValue(x, y-dy, z)+1.0)));

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
    module::Perlin               _noise;
    const NoiseOptions           _options;
    osg::ref_ptr<osgDB::Options> _dbOptions;
};


class ReaderWriterNoise : public TileSourceDriver
{
    public:
        ReaderWriterNoise()
        {
            supportsExtension( "osgearth_noise", "Procedurally generated terrain" );
        }

        virtual const char* className()
        {
            return "Noise ReaderWriter";
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
                return ReadResult::FILE_NOT_HANDLED;

            return new NoiseSource( getTileSourceOptions(options) );
        }
};

REGISTER_OSGPLUGIN(osgearth_noise, ReaderWriterNoise)

