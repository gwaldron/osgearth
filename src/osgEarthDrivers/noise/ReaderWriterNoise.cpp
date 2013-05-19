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
        osg::Image* image = 0L;

        // start by building a heightfield
        osg::ref_ptr<osg::HeightField> hf = createHeightField(key, progress);
        if ( hf.valid() )
        {
            // set up the image
            image = new osg::Image();
            image->allocateImage( hf->getNumColumns(), hf->getNumRows(), 1, GL_RGB, GL_UNSIGNED_BYTE );
            ImageUtils::PixelWriter write(image);

            // calculate the tangent space transformation matrix:
            osg::Matrix tm;

            // figure out the spacing between pixels in the same units as the height value:
            const GeoExtent& ex = key.getExtent();
            double dx = ex.width()/(double)(image->s()-1), dy = ex.height()/(double)(image->t()-1);
            const SpatialReference* srs  = ex.getSRS();
            if ( srs->isGeographic() )
            {
                const SpatialReference* ecef = srs->getECEF();
                dx = srs->transformUnits(dx, ecef, ex.south()+0.5*dy);
                dy = srs->transformUnits(dy, ecef, ex.south()+0.5*dy);

                osg::Vec3d m;
                ex.getCentroid(m.x(), m.y());
                srs->transform(m, ecef, m);
                m.normalize();
                tm.makeRotate(osg::Vec3d(0,0,1), m);
                //tm = osg::Matrix::inverse(tm);
            }

            for(int s=0; s<image->s(); ++s)
            {
                for(int t=0; t<image->t(); ++t)
                {
                    if ( s == 0 || t == 0 || s == image->s()-1 || t == image->t()-1 )
                    {
                        //TODO: deal with the edges properly.
                        write(osg::Vec4f(0,0,1,1), s, t);
                    }
                    else
                    {
                        // cross the EW and NS vectors to get the normal:
                        osg::Vec3 west (-dx,   0, hf->getHeight(s-1, t));
                        osg::Vec3 east ( dx,   0, hf->getHeight(s+1, t));
                        osg::Vec3 north(  0,  dy, hf->getHeight(s, t+1));
                        osg::Vec3 south(  0, -dy, hf->getHeight(s, t-1));

                        osg::Vec3 H = east-west;
                        osg::Vec3 V = north-south;
                        osg::Vec3 normal = H ^ V;

                        normal.normalize();

                        if ( normal.z() < 0 )
                            OE_WARN << "DOING it WRONG" << std::endl;

                        // encode as: x[-1..1]=>r[0..255]; y[-1..1]=>g[0..255]; z[0..1]=>[0..255]
                        normal.x() = normal.x()*0.5f + 0.5f;
                        normal.y() = normal.y()*0.5f + 0.5f;
                        normal.z() = normal.z()*0.5f + 0.5f;

                        normal.normalize();

                        write(osg::Vec4f(normal,1), s, t);
                    }
                }
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

