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
#include <osgEarth/ElevationLayer>
#include <osgEarth/VerticalDatum>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Progress>
#include <osgEarth/MemCache>
#include <osg/Version>
#include <iterator>

using namespace osgEarth;
using namespace OpenThreads;

#define LC "[ElevationLayer] \"" << getName() << "\" : "

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
    _offset.init( false );
    _noDataPolicy.init( NODATA_INTERPOLATE );
}

Config
ElevationLayerOptions::getConfig( bool isolate ) const
{
    Config conf = TerrainLayerOptions::getConfig( isolate );
    conf.updateIfSet("offset", _offset);
    conf.updateIfSet("nodata_policy", "default",     _noDataPolicy, NODATA_INTERPOLATE );
    conf.updateIfSet("nodata_policy", "interpolate", _noDataPolicy, NODATA_INTERPOLATE );
    conf.updateIfSet("nodata_policy", "msl",         _noDataPolicy, NODATA_MSL );

    return conf;
}

void
ElevationLayerOptions::fromConfig( const Config& conf )
{
    conf.getIfSet("offset", _offset );
    conf.getIfSet("nodata_policy", "default",     _noDataPolicy, NODATA_INTERPOLATE );
    conf.getIfSet("nodata_policy", "interpolate", _noDataPolicy, NODATA_INTERPOLATE );
    conf.getIfSet("nodata_policy", "msl",         _noDataPolicy, NODATA_MSL );
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
    // Opeartion that replaces invalid heights with the NO_DATA_VALUE marker.
    struct NormalizeNoDataValues : public TileSource::HeightFieldOperation
    {
        NormalizeNoDataValues(TileSource* source)
        {
            _noDataValue   = source->getNoDataValue();
            _minValidValue = source->getMinValidValue();
            _maxValidValue = source->getMaxValidValue();
        }

        void operator()(osg::ref_ptr<osg::HeightField>& hf)
        {
            if ( hf.valid() )
            {
                osg::FloatArray* values = hf->getFloatArray();
                for(osg::FloatArray::iterator i = values->begin(); i != values->end(); ++i)
                {
                    float& value = *i;
                    if ( osg::equivalent(value, _noDataValue) || value < _minValidValue || value > _maxValidValue )
                    {
                        value = NO_DATA_VALUE;
                    }
                } 
            }
        }

        float _noDataValue, _minValidValue, _maxValidValue;
    };

    // perform very basic sanity-check validation on a heightfield.
    bool validateHeightField(osg::HeightField* hf)
    {
        if (!hf) 
            return false;
        if (hf->getNumRows() < 2 || hf->getNumRows() > 1024)
            return false;
        if (hf->getNumColumns() < 2 || hf->getNumColumns() > 1024)
            return false;
        if (hf->getHeightList().size() != hf->getNumColumns() * hf->getNumRows())
            return false;
        if (hf->getXInterval() < 1e-5 || hf->getYInterval() < 1e-5)
            return false;
        
        return true;
    }    
}

//------------------------------------------------------------------------

ElevationLayer::ElevationLayer( const ElevationLayerOptions& options ) :
TerrainLayer   ( options, &_runtimeOptions ),
_runtimeOptions( options )
{
    init();
}

ElevationLayer::ElevationLayer( const std::string& name, const TileSourceOptions& driverOptions ) :
TerrainLayer   ( ElevationLayerOptions(name, driverOptions), &_runtimeOptions ),
_runtimeOptions( ElevationLayerOptions(name, driverOptions) )
{
    init();
}

ElevationLayer::ElevationLayer( const ElevationLayerOptions& options, TileSource* tileSource ) :
TerrainLayer   ( options, &_runtimeOptions, tileSource ),
_runtimeOptions( options )
{
    init();
}

void
ElevationLayer::init()
{
    TerrainLayer::init();
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

TileSource::HeightFieldOperation*
ElevationLayer::getOrCreatePreCacheOp()
{
    if ( !_preCacheOp.valid() && getTileSource() )
    {
        Threading::ScopedMutexLock lock(_mutex);
        if ( !_preCacheOp.valid() )
        {
            _preCacheOp = new NormalizeNoDataValues( getTileSource() );
        }
    }
    return _preCacheOp.get();
}


osg::HeightField*
ElevationLayer::createHeightFieldFromTileSource(const TileKey&    key,
                                                ProgressCallback* progress)
{
    osg::HeightField* result = 0L;

    TileSource* source = getTileSource();
    if ( !source )
        return 0L;

    // If the key is blacklisted, fail.
    if ( source->getBlacklist()->contains( key ))
    {
        OE_DEBUG << LC << "Tile " << key.str() << " is blacklisted " << std::endl;
        return 0L;
    }

    // If the profiles are horizontally equivalent (different vdatums is OK), take the
    // quick route:
    if ( key.getProfile()->isHorizEquivalentTo( getProfile() ) )
    {
        // Only try to get data if the source actually has data
        if ( !source->hasData(key) )
        {
            OE_DEBUG << LC << "Source for layer has no data at " << key.str() << std::endl;
            return 0L;
        }

        // Make it from the source:
        result = source->createHeightField( key, getOrCreatePreCacheOp(), progress );
   
        // If the result is good, we how have a heightfield but it's vertical values
        // are still relative to the tile source's vertical datum. Convert them.
        if ( result )
        {
            if ( ! key.getExtent().getSRS()->isVertEquivalentTo( getProfile()->getSRS() ) )
            {
                VerticalDatum::transform(
                    getProfile()->getSRS()->getVerticalDatum(),    // from
                    key.getExtent().getSRS()->getVerticalDatum(),  // to
                    key.getExtent(),
                    result );
            }
        }
        
        // Blacklist the tile if it is the same projection as the source and
        // we can't get it and it wasn't cancelled
        if (result == 0L)
        {
            if ( progress == 0L ||
                 ( !progress->isCanceled() && !progress->needsRetry() ) )
            {
                source->getBlacklist()->add( key );
            }
        }
    }

    // Otherwise, profiles don't match so we need to composite:
    else
    {
        // note: this method takes care of the vertical datum shift internally.
        result = assembleHeightFieldFromTileSource( key, progress );
    }

    return result;
}


osg::HeightField*
ElevationLayer::assembleHeightFieldFromTileSource(const TileKey&    key,
                                                  ProgressCallback* progress)
{			
    osg::HeightField* result = 0L;

    // Collect the heightfields for each of the intersecting tiles.
    GeoHeightFieldVector heightFields;

    //Determine the intersecting keys
    std::vector< TileKey > intersectingTiles;
    getProfile()->getIntersectingTiles( key, intersectingTiles );

    // collect heightfield for each intersecting key. Note, we're hitting the
    // underlying tile source here, so there's no vetical datum shifts happening yet.
    // we will do that later.
    if ( intersectingTiles.size() > 0 )
    {
        for (unsigned int i = 0; i < intersectingTiles.size(); ++i)
        {
            const TileKey& layerKey = intersectingTiles[i];

            if ( isKeyInRange(layerKey) )
            {
                osg::HeightField* hf = createHeightFieldFromTileSource( layerKey, progress );
                if ( hf )
                {
                    heightFields.push_back( GeoHeightField(hf, layerKey.getExtent()) );
                }
            }
        }
    }

    // If we actually got a HeightField, resample/reproject it to match the incoming TileKey's extents.
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

        //Now sort the heightfields by resolution to make sure we're sampling the highest resolution one first.
        std::sort( heightFields.begin(), heightFields.end(), GeoHeightField::SortByResolutionFunctor());        

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
            double x = minx + (dx * (double)c);
            for (unsigned r = 0; r < height; ++r)
            {
                double y = miny + (dy * (double)r);

                //For each sample point, try each heightfield.  The first one with a valid elevation wins.
                float elevation = NO_DATA_VALUE;
                for (GeoHeightFieldVector::iterator itr = heightFields.begin(); itr != heightFields.end(); ++itr)
                {
                    // get the elevation value, at the same time transforming it vertically into the 
                    // requesting key's vertical datum.
                    float e = 0.0;
                    if (itr->getElevation(key.getExtent().getSRS(), x, y, INTERP_BILINEAR, key.getExtent().getSRS(), e))
                    {
                        elevation = e;
                        break;
                    }
                }
                result->setHeight( c, r, elevation );                
            }
        }
    }

    return result;
}


GeoHeightField
ElevationLayer::createHeightField(const TileKey&    key,
                                  ProgressCallback* progress )
{
    if (getStatus().isError())
    {
        return GeoHeightField::INVALID;
    }

    // If the layer is disabled, bail out.
    if ( getEnabled() == false )
    {
        return GeoHeightField::INVALID;
    }

    GeoHeightField result;
    osg::ref_ptr<osg::HeightField> hf;

    // Check the memory cache first
    bool fromMemCache = false;

    // cache key combines the key with the full signature (incl vdatum)
    std::string cacheKey = Stringify() << key.str() << "_" << key.getProfile()->getFullSignature();
    const CachePolicy& policy = getCacheSettings()->cachePolicy().get();

    if ( _memCache.valid() )
    {
        CacheBin* bin = _memCache->getOrCreateDefaultBin();
        ReadResult cacheResult = bin->readObject(cacheKey, 0L);
        if ( cacheResult.succeeded() )
        {
            result = GeoHeightField(
                static_cast<osg::HeightField*>(cacheResult.releaseObject()),
                key.getExtent());

            fromMemCache = true;
        }
    }

    if ( !result.valid() )
    {
        // See if there's a persistent cache.
        CacheBin* cacheBin = getCacheBin( key.getProfile() );

        // validate that we have either a valid tile source, or we're cache-only.
        if ( ! (getTileSource() || (policy.isCacheOnly() && cacheBin) ) )
        {
            disable("Error: layer does not have a valid TileSource, cannot create heightfield");
            return GeoHeightField::INVALID;
        }

        // validate the existance of a valid layer profile.
        if ( !policy.isCacheOnly() && !getProfile() )
        {
            disable("Could not establish a valid profile");
            return GeoHeightField::INVALID;
        }

        // Now attempt to read from the cache. Since the cached data is stored in the
        // map profile, we can try this first.
        bool fromCache = false;

        osg::ref_ptr< osg::HeightField > cachedHF;

        if ( cacheBin && policy.isCacheReadable() )
        {
            ReadResult r = cacheBin->readObject(cacheKey, 0L);
            if ( r.succeeded() )
            {            
                bool expired = policy.isExpired(r.lastModifiedTime());
                cachedHF = r.get<osg::HeightField>();
                if ( cachedHF && validateHeightField(cachedHF) )
                {
                    if (!expired)
                    {
                        hf = cachedHF;
                        fromCache = true;
                    }
                }
            }
        }

        // if we're cache-only, but didn't get data from the cache, fail silently.
        if ( !hf.valid() && policy.isCacheOnly() )
        {
            return GeoHeightField::INVALID;
        }

        if ( !hf.valid() )
        {
            // bad tilesource? fail
            if ( !getTileSource() || !getTileSource()->isOK() )
                return GeoHeightField::INVALID;

            if ( !isKeyInRange(key) )
                return GeoHeightField::INVALID;

            // build a HF from the TileSource.
            hf = createHeightFieldFromTileSource( key, progress );

            // validate it to make sure it's legal.
            if ( hf.valid() && !validateHeightField(hf.get()) )
            {
                OE_WARN << LC << "Driver " << getTileSource()->getName() << " returned an illegal heightfield" << std::endl;
                hf = 0L; // to fall back on cached data if possible.
            }

            // cache if necessary
            if ( hf            && 
                 cacheBin      && 
                 !fromCache    &&
                 policy.isCacheWriteable() )
            {
                cacheBin->write(cacheKey, hf, 0L);
            }

            // We have an expired heightfield from the cache and no new data from the TileSource.  So just return the cached data.
            if (!hf.valid() && cachedHF.valid())
            {
                OE_DEBUG << LC << "Using cached but expired heightfield for " << key.str() << std::endl;
                hf = cachedHF;
            }

            if ( !hf.valid() )
            {
                return GeoHeightField::INVALID;
            }

            // Set up the heightfield params.
            double minx, miny, maxx, maxy;
            key.getExtent().getBounds(minx, miny, maxx, maxy);
            hf->setOrigin( osg::Vec3d( minx, miny, 0.0 ) );
            double dx = (maxx - minx)/(double)(hf->getNumColumns()-1);
            double dy = (maxy - miny)/(double)(hf->getNumRows()-1);
            hf->setXInterval( dx );
            hf->setYInterval( dy );
            hf->setBorderWidth( 0 );
        }

        if ( hf.valid() )
        {
            result = GeoHeightField( hf.get(), key.getExtent() );
        }
    }

    // write to mem cache if needed:
    if ( result.valid() && !fromMemCache && _memCache.valid() )
    {
        CacheBin* bin = _memCache->getOrCreateDefaultBin();
        bin->write(cacheKey, result.getHeightField(), 0L);
    }

    // post-processing:
    if ( result.valid() )
    {
        if ( _runtimeOptions.noDataPolicy() == NODATA_MSL )
        {
            // requested VDatum:
            const VerticalDatum* outputVDatum = key.getExtent().getSRS()->getVerticalDatum();
            const Geoid* geoid = 0L;

            // if there's an output vdatum, just set all invalid's to zero MSL.
            if ( outputVDatum == 0L )
            {
                // if the output is geodetic (HAE), but the input has a geoid, 
                // use that geoid to populate the invalid data at sea level.
                const VerticalDatum* profileDatum  = getProfile()->getSRS()->getVerticalDatum();
                if ( profileDatum )
                    geoid = profileDatum->getGeoid();
            }

            HeightFieldUtils::resolveInvalidHeights(
                result.getHeightField(),
                result.getExtent(),
                NO_DATA_VALUE,
                geoid );
        }
    }

    return result;
}


//------------------------------------------------------------------------

#undef  LC
#define LC "[ElevationLayers] "

ElevationLayerVector::ElevationLayerVector()
{
    //nop
}


ElevationLayerVector::ElevationLayerVector(const ElevationLayerVector& rhs) :
osg::MixinVector< osg::ref_ptr<ElevationLayer> >( rhs )
{
    //nop
}



namespace
{
    typedef osg::ref_ptr<ElevationLayer>          RefElevationLayer;
    typedef std::pair<RefElevationLayer, TileKey> LayerAndKey;
    typedef std::vector<LayerAndKey>              LayerAndKeyVector;
}

bool
ElevationLayerVector::populateHeightField(osg::HeightField*      hf,
                                          const TileKey&         key,
                                          const Profile*         haeProfile,
                                          ElevationInterpolation interpolation,
                                          ProgressCallback*      progress ) const
{
    // heightfield must already exist.
    if ( !hf )
        return false;

    // if the caller provided an "HAE map profile", he wants an HAE elevation grid even if
    // the map profile has a vertical datum. This is the usual case when building the 3D
    // terrain, for example. Construct a temporary key that doesn't have the vertical
    // datum info and use that to query the elevation data.
    TileKey keyToUse = key;
    if ( haeProfile )
    {
        keyToUse = TileKey(key.getLOD(), key.getTileX(), key.getTileY(), haeProfile );
    }
    
    // Collect the valid layers for this tile.
    LayerAndKeyVector contenders;
    LayerAndKeyVector offsets;

    // Track the number of layers that would return fallback data.
    unsigned numFallbackLayers = 0;

    // Check them in reverse order since the highest priority is last.
    for(ElevationLayerVector::const_reverse_iterator i = this->rbegin(); i != this->rend(); ++i)
    {
        ElevationLayer* layer = i->get();

        if ( layer->getEnabled() && layer->getVisible() )
        {
            // calculate the resolution-mapped key (adjusted for tile resolution differential).            
            TileKey mappedKey = keyToUse.mapResolution(
                hf->getNumColumns(),
                layer->getTileSize() );

            bool useLayer = true;
            TileKey bestKey( mappedKey );

            // Is there a tilesource? If not we are cache-only and cannot reject the layer.
            if ( layer->getTileSource() )
            {
                // Check whether the non-mapped key is valid according to the user's min/max level settings:
                if ( !layer->isKeyInRange(key) )
                {
                    useLayer = false;
                }
                

                // Find the "best available" mapped key from the tile source:
                else 
                {
                    if ( layer->getTileSource()->getBestAvailableTileKey(mappedKey, bestKey) )
                    {
                        // If the bestKey is not the mappedKey, this layer is providing
                        // fallback data (data at a lower resolution than requested)
                        if ( mappedKey != bestKey )
                        {
                            numFallbackLayers++;
                        }
                    }
                    else
                    {
                        useLayer = false;
                    }
                }
            }

            if ( useLayer )
            {
                if ( layer->isOffset() )
                {
                    offsets.push_back( std::make_pair(layer, bestKey) );
                }
                else
                {
                    contenders.push_back( std::make_pair(layer, bestKey) );
                }
            }
        }
    }

    // nothing? bail out.
    if ( contenders.empty() && offsets.empty() )
    {
        return false;
    }

    // if everything is fallback data, bail out.
    if ( contenders.size() + offsets.size() == numFallbackLayers )
    {
        return false;
    }
    
    // Sample the layers into our target.
    unsigned numColumns = hf->getNumColumns();
    unsigned numRows    = hf->getNumRows();    
    double   xmin       = key.getExtent().xMin();
    double   ymin       = key.getExtent().yMin();
    double   dx         = key.getExtent().width() / (double)(numColumns-1);
    double   dy         = key.getExtent().height() / (double)(numRows-1);
    
    // We will load the actual heightfields on demand. We might not need them all.
    GeoHeightFieldVector heightFields(contenders.size());
    GeoHeightFieldVector offsetFields(offsets.size());
    std::vector<bool>    heightFallback(contenders.size(), false);
    std::vector<bool>    heightFailed(contenders.size(), false);
    std::vector<bool>    offsetFailed(offsets.size(), false);

    // The maximum number of heightfields to keep in this local cache
    unsigned int maxHeightFields = 50;
    unsigned numHeightFieldsInCache = 0;

    const SpatialReference* keySRS = keyToUse.getProfile()->getSRS();

    bool realData = false;

    unsigned int total = numColumns * numRows;
    unsigned int completed = 0;
    int nodataCount = 0;

    for (unsigned c = 0; c < numColumns; ++c)
    {
        double x = xmin + (dx * (double)c);
        for (unsigned r = 0; r < numRows; ++r)
        {
            double y = ymin + (dy * (double)r);

            // Collect elevations from each layer as necessary.
            bool resolved = false;

            for(int i=0; i<contenders.size() && !resolved; ++i)
            {
                if ( heightFailed[i] )
                    continue;

                ElevationLayer* layer = contenders[i].first.get();

                GeoHeightField& layerHF = heightFields[i];
                TileKey actualKey = contenders[i].second;

                if (!layerHF.valid())
                {
                    // We couldn't get the heightfield from the cache, so try to create it.
                    // We also fallback on parent layers to make sure that we have data at the location even if it's fallback.
                    while (!layerHF.valid() && actualKey.valid())
                    {
                        layerHF = layer->createHeightField(actualKey, progress);
                        if (!layerHF.valid())
                        {
                            actualKey = actualKey.createParentKey();
                        }
                    }

                    // Mark this layer as fallback if necessary.
                    if (layerHF.valid())
                    {
                        heightFallback[i] = actualKey != contenders[i].second;
                        numHeightFieldsInCache++;
                    }
                    else
                    {
                        heightFailed[i] = true;
                        continue;
                    }
                }

                if (layerHF.valid())
                {
                    bool isFallback = heightFallback[i];

                    // We only have real data if this is not a fallback heightfield.
                    if (!isFallback)
                    {
                        realData = true;
                    }

                    float elevation;
                    if (layerHF.getElevation(keySRS, x, y, interpolation, keySRS, elevation))
                    {
                        if ( elevation != NO_DATA_VALUE )
                        {
                            resolved = true;                    
                            hf->setHeight(c, r, elevation);
                        }
                        else
                        {
                            ++nodataCount;
                        }
                    }
                }


                // Clear the heightfield cache if we have too many heightfields in the cache.
                if (numHeightFieldsInCache >= maxHeightFields)
                {
                    //OE_NOTICE << "Clearing cache" << std::endl;
                    for (unsigned int k = 0; k < heightFields.size(); k++)
                    {
                        heightFields[k] = GeoHeightField::INVALID;
                        heightFallback[k] = false;
                    }
                    numHeightFieldsInCache = 0;
                }
            }

            for(int i=offsets.size()-1; i>=0; --i)
            {
                if ( offsetFailed[i] )
                    continue;

                GeoHeightField& layerHF = offsetFields[i];
                if ( !layerHF.valid() )
                {
                    ElevationLayer* offset = offsets[i].first.get();

                    layerHF = offset->createHeightField(offsets[i].second, progress);
                    if ( !layerHF.valid() )
                    {
                        offsetFailed[i] = true;
                        continue;
                    }
                }

                // If we actually got a layer then we have real data
                realData = true;

                float elevation = 0.0f;
                if (layerHF.getElevation(keySRS, x, y, interpolation, keySRS, elevation) &&
                    elevation != NO_DATA_VALUE)
                {                    
                    hf->getHeight(c, r) += elevation;
                }
            }
        }
    }

    // Return whether or not we actually read any real data
    return realData;
}
