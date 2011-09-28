/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarth/ElevationLayer>
#include <osgEarth/Registry>
#include <osg/Version>

using namespace osgEarth;
using namespace OpenThreads;

#define LC "[ElevationLayer] "

//------------------------------------------------------------------------

ElevationLayerOptions::ElevationLayerOptions( const ConfigOptions& options ) :
TerrainLayerOptions( options )
{
    setDefaults();
    fromConfig( _conf );
}

ElevationLayerOptions::ElevationLayerOptions( const std::string& name, const TileSourceOptions& driverOptions ) :
TerrainLayerOptions( name, driverOptions )
{
    setDefaults();
    fromConfig( _conf );
}

void
ElevationLayerOptions::setDefaults()
{
    //NOP
}

Config
ElevationLayerOptions::getConfig() const
{
    Config conf = TerrainLayerOptions::getConfig();
    //NOP
    return conf;
}

void
ElevationLayerOptions::fromConfig( const Config& conf )
{
    //NOP
}

void
ElevationLayerOptions::mergeConfig( const Config& conf )
{
    TerrainLayerOptions::mergeConfig( conf );
    fromConfig( conf );
}

//------------------------------------------------------------------------

namespace
{
    struct ElevationLayerPreCacheOperation : public TileSource::HeightFieldOperation
    {
        osg::ref_ptr<CompositeValidValueOperator> ops;

        ElevationLayerPreCacheOperation( TileSource* source )
        {
		    ops = new CompositeValidValueOperator;
		    ops->getOperators().push_back(new osgTerrain::NoDataValue(source->getNoDataValue()));
		    ops->getOperators().push_back(new osgTerrain::ValidRange(source->getNoDataMinValue(), source->getNoDataMaxValue()));
        }

        void operator()( osg::ref_ptr<osg::HeightField>& hf )
        {
		    //Modify the heightfield data so that is contains a standard value for NO_DATA
		    ReplaceInvalidDataOperator op;
		    op.setReplaceWith(NO_DATA_VALUE);
		    op.setValidDataOperator(ops.get());
		    op( hf.get() );
        }
    };
}

//------------------------------------------------------------------------

ElevationLayer::ElevationLayer( const ElevationLayerOptions& options ) :
TerrainLayer   ( &_runtimeOptions ),
_runtimeOptions( options )
{
    init();
}

ElevationLayer::ElevationLayer( const std::string& name, const TileSourceOptions& driverOptions ) :
TerrainLayer   ( &_runtimeOptions ),
_runtimeOptions( ElevationLayerOptions(name, driverOptions) )
{
    init();
}

ElevationLayer::ElevationLayer( const ElevationLayerOptions& options, TileSource* tileSource ) :
TerrainLayer   ( &_runtimeOptions, tileSource ),
_runtimeOptions( options )
{
    init();
}

void
ElevationLayer::init()
{
    _tileSize = 32;
}

std::string
ElevationLayer::suggestCacheFormat() const
{
#if OSG_MIN_VERSION_REQUIRED(2,8,0)
        //OSG 2.8 onwards should use TIF for heightfields
        return "tif";
#else
        //OSG 2.8 and below should use DDS
        return "dds";
#endif
}

void
ElevationLayer::addCallback( ElevationLayerCallback* cb )
{
    _callbacks.push_back( cb );
}

void
ElevationLayer::removeCallback( ElevationLayerCallback* cb )
{
    ElevationLayerCallbackList::iterator i = std::find( _callbacks.begin(), _callbacks.end(), cb );
    if ( i != _callbacks.end() ) 
        _callbacks.erase( i );
}

void
ElevationLayer::fireCallback( TerrainLayerCallbackMethodPtr method )
{
    for( ElevationLayerCallbackList::const_iterator i = _callbacks.begin(); i != _callbacks.end(); ++i )
    {
        ElevationLayerCallback* cb = i->get();
        (cb->*method)( this );
    }
}

void
ElevationLayer::fireCallback( ElevationLayerCallbackMethodPtr method )
{
    for( ElevationLayerCallbackList::const_iterator i = _callbacks.begin(); i != _callbacks.end(); ++i )
    {
        ElevationLayerCallback* cb = i->get();
        (cb->*method)( this );
    }
}

void
ElevationLayer::initTileSource()
{
    // call superclass first.
    TerrainLayer::initTileSource();

    if ( _tileSource.valid() )
        _preCacheOp = new ElevationLayerPreCacheOperation( _tileSource.get() );
}

GeoHeightField
ElevationLayer::createGeoHeightField(const TileKey& key, ProgressCallback* progress)
{
    osg::HeightField* hf = 0L;
    //osg::ref_ptr<osg::HeightField> hf;

    TileSource* source = getTileSource();

    //Only try to get the tile if it isn't blacklisted
    if (!source->getBlacklist()->contains( key.getTileId() ))
    {
        //Only try to get data if the source actually has data
        if (source->hasData( key ) )
        {
            hf = source->createHeightField( key, _preCacheOp.get(), progress );

            //Blacklist the tile if we can't get it and it wasn't cancelled
            if ( !hf && (!progress || !progress->isCanceled()))
            {
                source->getBlacklist()->add(key.getTileId());
            }
        }
        else
        {
            OE_DEBUG << LC << "Source for layer \"" << getName() << "\" has no data at " << key.str() << std::endl;
        }
    }
    else
    {
        OE_DEBUG << LC << "Tile " << key.str() << " is blacklisted " << std::endl;
    }

    return hf ?
        GeoHeightField( hf, key.getExtent(), getProfile()->getVerticalSRS() ) :
        GeoHeightField::INVALID;
}

osg::HeightField*
ElevationLayer::createHeightField(const osgEarth::TileKey& key, ProgressCallback* progress )
{
    osg::HeightField* result = 0L;
    //osg::ref_ptr<osg::HeightField> result;

    const Profile* layerProfile = getProfile();
    const Profile* mapProfile = key.getProfile();

	if ( !layerProfile )
	{
		OE_WARN << LC << "Could not get a valid profile for Layer \"" << getName() << "\"" << std::endl;
        return 0L;
	}

	if ( !isCacheOnly() && !getTileSource() )
	{
		OE_WARN << LC << "Error: ElevationLayer does not have a valid TileSource, cannot create heightfield " << std::endl;
		return 0L;
	}

    //Write the layer properties if they haven't been written yet.  Heightfields are always stored in the map profile.
    if (!_cacheProfile.valid() && _cache.valid() && _runtimeOptions.cacheEnabled() == true && _tileSource.valid())
    {
        _cacheProfile = mapProfile;
        if ( _tileSource->isOK() )
        {
            _cache->storeProperties( _cacheSpec, _cacheProfile.get(),  _tileSource->getPixelsPerTile() );
        }
    }

	//See if we can get it from the cache.
	if (_cache.valid() && _runtimeOptions.cacheEnabled() == true )
	{
        osg::ref_ptr<const osg::HeightField> cachedHF;
		if ( _cache->getHeightField( key, _cacheSpec, cachedHF ) )
		{
			OE_DEBUG << LC << "ElevationLayer::createHeightField got tile " << key.str() << " from layer \"" << getName() << "\" from cache " << std::endl;

            // make a copy:
            result = new osg::HeightField( *cachedHF.get() );
		}
	}

    //in cache-only mode, if the cache fetch failed, bail out.
    if ( result == 0L && isCacheOnly() )
    {
        return 0L;
    }

	if ( result == 0L && getTileSource() && getTileSource()->isOK() )
    {
		//If the profiles are equivalent, get the HF from the TileSource.
		if (key.getProfile()->isEquivalentTo( getProfile() ))
		{
			if (isKeyValid( key ) )
			{
				GeoHeightField hf = createGeoHeightField( key, progress );
				if (hf.valid())
				{
					result = hf.takeHeightField();
				}
			}
		}

		else
		{
			//Collect the heightfields for each of the intersecting tiles.
			//typedef std::vector< GeoHeightField > HeightFields;
			GeoHeightFieldVector heightFields;

			//Determine the intersecting keys
			std::vector< TileKey > intersectingTiles;
			getProfile()->getIntersectingTiles(key, intersectingTiles);
			if (intersectingTiles.size() > 0)
			{
				for (unsigned int i = 0; i < intersectingTiles.size(); ++i)
				{
					if (isKeyValid( intersectingTiles[i] ) )
					{
                        GeoHeightField hf = createGeoHeightField( intersectingTiles[i], progress );
						if (hf.valid())
						{
							heightFields.push_back(hf);
						}
					}
				}
			}

			//If we actually got a HeightField, resample/reproject it to match the incoming TileKey's extents.
			if (heightFields.size() > 0)
			{		
				unsigned int width = 0;
				unsigned int height = 0;

                for (GeoHeightFieldVector::iterator itr = heightFields.begin(); itr != heightFields.end(); ++itr)
				{
					if (itr->getHeightField()->getNumColumns() > width)
                        width = itr->getHeightField()->getNumColumns();
					if (itr->getHeightField()->getNumRows() > height) 
                        height = itr->getHeightField()->getNumRows();
				}

                result = new osg::HeightField();
				result->allocate(width, height);

				//Go ahead and set up the heightfield so we don't have to worry about it later
				double minx, miny, maxx, maxy;
				key.getExtent().getBounds(minx, miny, maxx, maxy);
				double dx = (maxx - minx)/(double)(width-1);
				double dy = (maxy - miny)/(double)(height-1);

				//Create the new heightfield by sampling all of them.
				for (unsigned int c = 0; c < width; ++c)
				{
					double geoX = minx + (dx * (double)c);
					for (unsigned r = 0; r < height; ++r)
					{
						double geoY = miny + (dy * (double)r);

						//For each sample point, try each heightfield.  The first one with a valid elevation wins.
						float elevation = NO_DATA_VALUE;
						for (GeoHeightFieldVector::iterator itr = heightFields.begin(); itr != heightFields.end(); ++itr)
						{
							float e = 0.0;
                            if (itr->getElevation(key.getExtent().getSRS(), geoX, geoY, INTERP_BILINEAR, _profile->getVerticalSRS(), e))
							{
								elevation = e;
								break;
							}
						}
						result->setHeight( c, r, elevation );                
					}
				}
			}
		}
    
        //Write the result to the cache.
        if (result && _cache.valid() && _runtimeOptions.cacheEnabled() == true )
        {
            _cache->setHeightField( key, _cacheSpec, result );
        }
    }

	//Initialize the HF values for osgTerrain
	if ( result )
	{	
		//Go ahead and set up the heightfield so we don't have to worry about it later
		double minx, miny, maxx, maxy;
		key.getExtent().getBounds(minx, miny, maxx, maxy);
		result->setOrigin( osg::Vec3d( minx, miny, 0.0 ) );
		double dx = (maxx - minx)/(double)(result->getNumColumns()-1);
		double dy = (maxy - miny)/(double)(result->getNumRows()-1);
		result->setXInterval( dx );
		result->setYInterval( dy );
		result->setBorderWidth( 0 );
	}
	
    return result;
}
