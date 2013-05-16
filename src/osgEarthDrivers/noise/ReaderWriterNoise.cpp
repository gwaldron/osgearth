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
        return 0;
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

