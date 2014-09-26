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
#include <osgEarth/TerrainLayer>
#include <osgEarth/TileSource>
#include <osgEarth/Registry>
#include <osgEarth/StringUtils>
#include <osgEarth/TimeControl>
#include <osgEarth/URI>
#include <osgEarth/MemCache>
#include <osgEarth/CacheBin>
#include <osgDB/WriteFile>
#include <osg/Version>
#include <OpenThreads/ScopedLock>
#include <memory.h>

using namespace osgEarth;
using namespace OpenThreads;

#define LC "[TerrainLayer] \"" << getName() << "\": "

//------------------------------------------------------------------------

TerrainLayerOptions::TerrainLayerOptions( const ConfigOptions& options ) :
ConfigOptions       ( options ),
_minLevel           ( 0 ),
_maxLevel           ( 23 ),
_cachePolicy        ( CachePolicy::DEFAULT ),
_loadingWeight      ( 1.0f ),
_exactCropping      ( false ),
_enabled            ( true ),
_visible            ( true ),
_reprojectedTileSize( 256 ),
_maxDataLevel       ( 99 )
{
    setDefaults();
    fromConfig( _conf ); 
}

TerrainLayerOptions::TerrainLayerOptions( const std::string& name, const TileSourceOptions& driverOptions ) :
ConfigOptions()
{
    setDefaults();
    fromConfig( _conf );
    _name = name;
    _driver = driverOptions;
}

void
TerrainLayerOptions::setDefaults()
{
    _enabled.init( true );
    _visible.init( true );
    _exactCropping.init( false );
    _reprojectedTileSize.init( 256 );
    _cachePolicy.init( CachePolicy() );
    _loadingWeight.init( 1.0f );
    _minLevel.init( 0 );
    _maxLevel.init( 23 );
    _maxDataLevel.init( 99 );
}

Config
TerrainLayerOptions::getConfig( bool isolate ) const
{
    Config conf = isolate ? ConfigOptions::newConfig() : ConfigOptions::getConfig();

    conf.set("name", _name);
    conf.updateIfSet( "min_level", _minLevel );
    conf.updateIfSet( "max_level", _maxLevel );
    conf.updateIfSet( "min_resolution", _minResolution );
    conf.updateIfSet( "max_resolution", _maxResolution );
    conf.updateIfSet( "loading_weight", _loadingWeight );
    conf.updateIfSet( "enabled", _enabled );
    conf.updateIfSet( "visible", _visible );
    conf.updateIfSet( "edge_buffer_ratio", _edgeBufferRatio);
    conf.updateIfSet( "reprojected_tilesize", _reprojectedTileSize);
    conf.updateIfSet( "max_data_level", _maxDataLevel );

    conf.updateIfSet( "vdatum", _vertDatum );

    conf.updateIfSet   ( "cacheid",      _cacheId );
    conf.updateIfSet   ( "cache_format", _cacheFormat );
    conf.updateObjIfSet( "cache_policy", _cachePolicy );
    conf.updateObjIfSet( "proxy",        _proxySettings );

    // Merge the TileSource options
    if ( !isolate && driver().isSet() )
        conf.merge( driver()->getConfig() );

    return conf;
}

void
TerrainLayerOptions::fromConfig( const Config& conf )
{
    _name = conf.value("name");
    conf.getIfSet( "min_level", _minLevel );
    conf.getIfSet( "max_level", _maxLevel );        
    conf.getIfSet( "min_resolution", _minResolution );
    conf.getIfSet( "max_resolution", _maxResolution );
    conf.getIfSet( "loading_weight", _loadingWeight );
    conf.getIfSet( "enabled", _enabled );
    conf.getIfSet( "visible", _visible );
    conf.getIfSet( "edge_buffer_ratio", _edgeBufferRatio);    
    conf.getIfSet( "reprojected_tilesize", _reprojectedTileSize);
    conf.getIfSet( "max_data_level", _maxDataLevel );

    conf.getIfSet( "vdatum", _vertDatum );
    conf.getIfSet( "vsrs", _vertDatum );    // back compat

    conf.getIfSet   ( "cacheid",      _cacheId );
    conf.getIfSet   ( "cache_format", _cacheFormat );
    conf.getObjIfSet( "cache_policy", _cachePolicy );
    conf.getObjIfSet( "proxy",        _proxySettings );

    // legacy support:
    if ( conf.value<bool>( "cache_only", false ) == true )
        _cachePolicy->usage() = CachePolicy::USAGE_CACHE_ONLY;
    if ( conf.value<bool>( "cache_enabled", true ) == false )
        _cachePolicy->usage() = CachePolicy::USAGE_NO_CACHE;

    if ( conf.hasValue("driver") )
        driver() = TileSourceOptions(conf);
}

void
TerrainLayerOptions::mergeConfig( const Config& conf )
{
    ConfigOptions::mergeConfig( conf );
    fromConfig( conf );
}

//------------------------------------------------------------------------

TerrainLayer::TerrainLayer(const TerrainLayerOptions& initOptions,
                           TerrainLayerOptions*       runtimeOptions ) :
_initOptions   ( initOptions ),
_runtimeOptions( runtimeOptions )
{
    init();
}

TerrainLayer::TerrainLayer(const TerrainLayerOptions& initOptions,
                           TerrainLayerOptions*       runtimeOptions,
                           TileSource*                tileSource ) :
_initOptions   ( initOptions ),
_runtimeOptions( runtimeOptions ),
_tileSource    ( tileSource )
{
    init();
}

TerrainLayer::~TerrainLayer()
{
    if ( _cache.valid() )
    {
        Threading::ScopedWriteLock exclusive( _cacheBinsMutex );
        for( CacheBinInfoMap::iterator i = _cacheBins.begin(); i != _cacheBins.end(); ++i )
        {
            CacheBinInfo& info = i->second;
            if ( info._bin.valid() )
            {
                _cache->removeBin( info._bin.get() );
            }
        }
    }
}

void
TerrainLayer::init()
{
    _tileSourceInitAttempted = false;
    _tileSourceInitFailed    = false;
    _tileSize                = 256;
    _dbOptions               = Registry::instance()->cloneOrCreateOptions();
    
    initializeCachePolicy( _dbOptions.get() );
    storeProxySettings( _dbOptions.get() );

    // Create an L2 mem cache that sits atop the main cache, if necessary.
    // For now: use the same L2 cache size at the driver.
    int memCacheSize = _initOptions.driver()->L2CacheSize().get();
    if ( memCacheSize > 0 )
    {
        _memCache = new MemCache(memCacheSize);
    }
}

void
TerrainLayer::setCache( Cache* cache )
{
    if (_cache.get() != cache && getCachePolicy() != CachePolicy::NO_CACHE )
    {
        _cache = cache;

        // Initialize a cache bin for this layer.
        if ( _cache.valid() )
        {
            // create the unique cache ID for the cache bin.
            std::string cacheId;

            if ( _runtimeOptions->cacheId().isSet() && !_runtimeOptions->cacheId()->empty() )
            {
                // user expliticy set a cacheId in the terrain layer options.
                // this appears to be a NOP; review for removal -gw
                cacheId = *_runtimeOptions->cacheId();
            }
            else
            {
                // system will generate a cacheId.
                // technically, this is not quite right, we need to remove everything that's
                // an image layer property and just use the tilesource properties.
                Config layerConf  = _runtimeOptions->getConfig( true );
                Config driverConf = _runtimeOptions->driver()->getConfig();
                Config hashConf   = driverConf - layerConf;

                OE_DEBUG << LC << "Hash JSON is: " << hashConf.toJSON(false) << std::endl;

                // remove cache-control properties before hashing.
                hashConf.remove( "cache_only" );
                hashConf.remove( "cache_enabled" );
                hashConf.remove( "cache_policy" );
                hashConf.remove( "cacheid" );
                
                // need this, b/c data is vdatum-transformed before caching.
                if ( layerConf.hasValue("vdatum") )
                    hashConf.add("vdatum", layerConf.value("vdatum"));

                cacheId = Stringify() << std::hex << osgEarth::hashString(hashConf.toJSON());

                _runtimeOptions->cacheId().init( cacheId ); // set as default value
            }
        }
    }

    if ( !_cache.valid() )
    {
        _cache = 0L; 
        setCachePolicy( CachePolicy::NO_CACHE );
    }
}

void
TerrainLayer::setCachePolicy( const CachePolicy& cp )
{
    _runtimeOptions->cachePolicy() = cp;
    _runtimeOptions->cachePolicy()->apply( _dbOptions.get() );
}

const CachePolicy&
TerrainLayer::getCachePolicy() const
{
    return _runtimeOptions->cachePolicy().value();
}

void
TerrainLayer::setTargetProfileHint( const Profile* profile )
{
    _targetProfileHint = profile;

    // This will attempt to open and access a cache bin if there
    // is one. This is important in cache-only mode since we cannot
    // establish a profile from the tile source.
    if ( getCachePolicy() != CachePolicy::NO_CACHE )
    {
        CacheBinMetadata meta;
        getCacheBinMetadata( profile, meta );
    }

    // If the tilesource was already initialized, re-read the 
    // cache policy hint since it may change due to the target
    // profile change.
    refreshTileSourceCachePolicyHint();
}

void
TerrainLayer::refreshTileSourceCachePolicyHint()
{
    if ( _tileSource.valid() && !_initOptions.cachePolicy().isSet() )
    {
        CachePolicy hint = _tileSource->getCachePolicyHint( _targetProfileHint.get() );

        if ( hint.usage().isSetTo(CachePolicy::USAGE_NO_CACHE) )
        {
            setCachePolicy( hint );
            OE_INFO << LC << "Caching disabled (by policy hint)" << std::endl;
        }
    }
}

TileSource* 
TerrainLayer::getTileSource() const
{
    if ( _tileSourceInitFailed )
        return 0L;

    if ((_tileSource.valid() && !_tileSourceInitAttempted) ||
        (!_tileSource.valid() && !isCacheOnly()))
    {
        Threading::ScopedMutexLock lock(_initTileSourceMutex);
        
        // double-check pattern
        if ((_tileSource.valid() && !_tileSourceInitAttempted) ||
            (!_tileSource.valid() && !isCacheOnly()))
        {
            // Initialize the tile source once.
            const_cast<TerrainLayer*>(this)->initTileSource();

            // read the cache policy hint from the tile source unless user expressly set 
            // a policy in the initialization options. In other words, the hint takes
            // ultimate priority (even over the Registry override) unless expressly
            // overridden in the layer options!
            const_cast<TerrainLayer*>(this)->refreshTileSourceCachePolicyHint();

            // Unless the user has already configured an expiration policy, use the "last modified"
            // timestamp of the TileSource to set a minimum valid cache entry timestamp.
            if ( _tileSource.valid() )
            {
                CachePolicy& cp = _runtimeOptions->cachePolicy().mutable_value();
                if ( !cp.minTime().isSet() && !cp.maxAge().isSet() )
                {
                    cp.minTime() = _tileSource->getLastModifiedTime();
                }
                OE_INFO << LC << "cache policy = " << getCachePolicy().usageString() << std::endl;
            }
        }
    }

    return _tileSource.get();
}

const Profile*
TerrainLayer::getProfile() const
{
    // NB: in cache-only mode, there IS NO layer profile.
    if ( !_profile.valid() && !isCacheOnly() )
    {
        if ( !_tileSourceInitAttempted )
        {
            // Call getTileSource to make sure the TileSource is initialized
            getTileSource();
        }
    }
    
    return _profile.get();
}

unsigned
TerrainLayer::getTileSize() const
{
    // force tile source initialization (which sets _tileSize)
    getTileSource();
    return _tileSize; //ts ? ts->getPixelsPerTile() : _tileSize;
}

bool
TerrainLayer::isDynamic() const
{
    TileSource* ts = getTileSource();
    return ts ? ts->isDynamic() : false;
}

CacheBin*
TerrainLayer::getCacheBin(const Profile* profile)
{
    // make sure we've initialized the tile source first.
    getTileSource();

    if ( getCachePolicy() == CachePolicy::NO_CACHE )
    {
        return 0L;
    }

    // the cache bin ID is the cache ID concatenated with the FULL profile signature.
    std::string binId = *_runtimeOptions->cacheId() + std::string("_") + profile->getFullSignature();

    return getCacheBin( profile, binId );
}

CacheBin*
TerrainLayer::getCacheBin(const Profile* profile, const std::string& binId)
{
    // make sure we've initialized the tile source first.
    TileSource* tileSource = getTileSource();

    // in no-cache mode, there are no cache bins.
    if ( getCachePolicy() == CachePolicy::NO_CACHE )
    {
        return 0L;
    }

    // if cache is not setted, return NULL
    if (_cache == NULL)
    {
        return 0L;
    }

    // see if the cache bin already exists and return it if so
    {
        Threading::ScopedReadLock shared(_cacheBinsMutex);
        CacheBinInfoMap::iterator i = _cacheBins.find( binId );
        if ( i != _cacheBins.end() )
            return i->second._bin.get();
    }

    // create/open the cache bin.
    {
        Threading::ScopedWriteLock exclusive(_cacheBinsMutex);

        // double-check:
        CacheBinInfoMap::iterator i = _cacheBins.find( binId );
        if ( i != _cacheBins.end() )
            return i->second._bin.get();

        // add the new bin:
        osg::ref_ptr<CacheBin> newBin = _cache->addBin( binId );

        // and configure:
        if ( newBin.valid() )
        {
            // attempt to read the cache metadata:
            CacheBinMetadata meta( newBin->readMetadata() );

            if ( meta.isValid() ) // cache exists and is valid.
            {
                // verify that the cache if compatible with the tile source:
                if ( tileSource && getProfile() )
                {
                    //todo: check the profile too
                    if ( *meta._sourceDriver != tileSource->getOptions().getDriver() )
                    {
                        OE_WARN << LC << "Cache has an incompatible driver or profile... disabling"
                            << std::endl;
                        setCachePolicy( CachePolicy::NO_CACHE );
                        return 0L;
                    }
                }   

                else if ( isCacheOnly() && !_profile.valid() )
                {
                    // in cacheonly mode, create a profile from the first cache bin accessed
                    // (they SHOULD all be the same...)
                    _profile = Profile::create( *meta._sourceProfile );
                    _tileSize = *meta._sourceTileSize;
                }
            }

            else
            {
                // cache does not exist, so try to create it. A valid TileSource is necessary
                // for this.
                if ( tileSource && getProfile() )
                {
                    // no existing metadata; create some.
                    meta._cacheBinId      = binId;
                    meta._sourceName      = this->getName();
                    meta._sourceDriver    = tileSource->getOptions().getDriver();
                    meta._sourceTileSize  = getTileSize();
                    meta._sourceProfile   = getProfile()->toProfileOptions();
                    meta._cacheProfile    = profile->toProfileOptions();
                    meta._cacheCreateTime = DateTime().asTimeStamp();

                    // store it in the cache bin.
                    newBin->writeMetadata( meta.getConfig() );
                }
                else if ( isCacheOnly() )
                {
                    OE_WARN << LC <<
                        "Failed to open a cache for layer "
                        "because cache_only policy is in effect and bin [" << binId << "] "
                        "could not be located."
                        << std::endl;
                    return 0L;
                }
                else
                {
                    OE_WARN << LC <<
                        "Failed to create cache bin [" << binId << "] "
                        "because there is no valid tile source."
                        << std::endl;
                    return 0L;
                }
            }

            // store the bin.
            CacheBinInfo& newInfo = _cacheBins[binId];
            newInfo._metadata = meta;
            newInfo._bin      = newBin.get();

            OE_INFO << LC <<
                "Opened cache bin [" << binId << "]" << std::endl;

            // If we loaded a profile from the cache metadata, apply the overrides:
            applyProfileOverrides();
        }
        else
        {
            // bin creation failed, so disable caching for this layer.
            setCachePolicy( CachePolicy::NO_CACHE );
            OE_WARN << LC << "Failed to create a cache bin; cache disabled." << std::endl;
        }

        return newBin.get(); // not release()
    }
}

bool
TerrainLayer::getCacheBinMetadata( const Profile* profile, CacheBinMetadata& output )
{
    // the cache bin ID is the cache IF concatenated with the profile signature.
    std::string binId = *_runtimeOptions->cacheId() + std::string("_") + profile->getFullSignature();
    CacheBin* bin = getCacheBin( profile );
    if ( bin )
    {
        Threading::ScopedReadLock shared(_cacheBinsMutex);
        CacheBinInfoMap::iterator i = _cacheBins.find( binId );
        if ( i != _cacheBins.end() )
        {
            output = i->second._metadata.value();
            return true;
        }
    }
    return false;
}

void
TerrainLayer::initTileSource()
{
    _tileSourceInitAttempted = true;

    OE_DEBUG << LC << "Initializing tile source ..." << std::endl;

    // Instantiate it from driver options if it has not already been created.
    // This will also set a manual "override" profile if the user provided one.
    if ( !_tileSource.valid() )
    {
        if ( _runtimeOptions->driver().isSet() )
        {
            _tileSource = TileSourceFactory::create(*_runtimeOptions->driver());
        }
    }

    // Initialize the profile with the context information:
    if ( _tileSource.valid() )
    {
        // set up the URI options.
        if ( !_dbOptions.valid() )
        {
            _dbOptions = Registry::instance()->cloneOrCreateOptions();
            if ( _cache.valid() ) _cache->apply( _dbOptions.get() );
            _initOptions.cachePolicy()->apply( _dbOptions.get() );
            URIContext( _runtimeOptions->referrer() ).apply( _dbOptions.get() );
        }


        // report on a manual override profile:
        if ( _tileSource->getProfile() )
        {
            OE_INFO << LC << "set profile to: " 
                << _tileSource->getProfile()->toString() << std::endl;
        }

        // Open the tile source (if it hasn't already been started)
        TileSource::Status status = _tileSource->getStatus();
        if ( status != TileSource::STATUS_OK )
        {
            status = _tileSource->open(TileSource::MODE_READ, _dbOptions.get());
        }

        if ( status == TileSource::STATUS_OK )
        {
            _tileSize = _tileSource->getPixelsPerTile();

#if 0 //debugging 
            // dump out data extents:
            if ( _tileSource->getDataExtents().size() > 0 )
            {
                OE_INFO << LC << "Data extents reported:" << std::endl;
                for(DataExtentList::const_iterator i = _tileSource->getDataExtents().begin();
                    i != _tileSource->getDataExtents().end(); ++i)
                {
                    const DataExtent& de = *i;
                    OE_INFO << "    "
                        << "X(" << i->xMin() << ", " << i->xMax() << ") "
                        << "Y(" << i->yMin() << ", " << i->yMax() << ") "
                        << "Z(" << i->minLevel().get() << ", " << i->maxLevel().get() << ")"
                        << std::endl;
                }                
            }
#endif
        }
        else
        {
            OE_WARN << LC << "Could not initialize driver" << std::endl;
            _tileSource = NULL;
            _tileSourceInitFailed = true;
            _runtimeOptions->enabled() = true;
        }
    }

    // Set the profile from the TileSource if possible:
    if ( _tileSource.valid() )
    {
        if ( !_profile.valid() && !_tileSourceInitFailed )
        {
            _profile = _tileSource->getProfile();
        }


        if ( _profile.valid() )
        {
            // create the final profile from any overrides:
            applyProfileOverrides();

            OE_INFO << LC << "Profile=" << _profile->toString() << std::endl;
        }
    }

    // Otherwise, force cache-only mode (since there is no tilesource). The layer will try to 
    // establish a profile from the metadata in the cache instead.
    else if (_cache.valid())
    {
        OE_NOTICE << LC << "Could not initialize TileSource " << _name << ", but a cache exists. Setting layer to cache-only mode." << std::endl;
        setCachePolicy( CachePolicy::CACHE_ONLY );
    }
}

void
TerrainLayer::applyProfileOverrides()
{
    // Check for a vertical datum override.
    bool changed = false;
    if ( _profile.valid() && _runtimeOptions->verticalDatum().isSet() )
    {
        std::string vdatum = *_runtimeOptions->verticalDatum();
        OE_INFO << "override vdatum = " << vdatum << ", profile vdatum = " << _profile->getSRS()->getVertInitString() << std::endl;
        if ( !ciEquals(_profile->getSRS()->getVertInitString(), vdatum) )
        {
            ProfileOptions po = _profile->toProfileOptions();
            po.vsrsString() = vdatum;
            _profile = Profile::create(po);
            changed = true;
        }
    }

    if (changed && _profile.valid())
    {
        OE_INFO << LC << "Override profile: " << _profile->toString() << std::endl;
    }
}

bool
TerrainLayer::isKeyInRange(const TileKey& key) const
{    
    if ( !key.valid() )
    {
        return false;
    }

    // First check the key against the min/max level limits, it they are set.
    if ((_runtimeOptions->maxLevel().isSet() && key.getLOD() > _runtimeOptions->maxLevel().value()) ||
        (_runtimeOptions->minLevel().isSet() && key.getLOD() < _runtimeOptions->minLevel().value()))
    {
        return false;
    }

    // Next, check against resolution limits (based on the source tile size).
    if (_runtimeOptions->minResolution().isSet() ||
        _runtimeOptions->maxResolution().isSet())
    {
        const Profile* profile = getProfile();
        if ( profile )
        {
            // calculate the resolution in the layer's profile, which can
            // be different that the key's profile.
            double resKey   = key.getExtent().width() / (double)getTileSize();
            double resLayer = key.getProfile()->getSRS()->transformUnits(resKey, profile->getSRS());

            if (_runtimeOptions->maxResolution().isSet() &&
                _runtimeOptions->maxResolution().value() > resLayer)
            {
                return false;
            }

            if (_runtimeOptions->minResolution().isSet() &&
                _runtimeOptions->minResolution().value() < resLayer)
            {
                return false;
            }
        }
    }

	return true;
}

bool
TerrainLayer::isCached(const TileKey& key) const
{
    // first consult the policy:
    if ( getCachePolicy() == CachePolicy::NO_CACHE )
        return false;
    else if ( getCachePolicy() == CachePolicy::CACHE_ONLY )
        return true;

    // next check for a bin:
    CacheBin* bin = const_cast<TerrainLayer*>(this)->getCacheBin( key.getProfile() );
    if ( !bin )
        return false;
    
    return bin->getRecordStatus( key.str() ) == CacheBin::STATUS_OK;
}

void
TerrainLayer::setVisible( bool value )
{
    _runtimeOptions->visible() = value;
    fireCallback( &TerrainLayerCallback::onVisibleChanged );
}

void
TerrainLayer::setDBOptions( const osgDB::Options* dbOptions )
{
    _dbOptions = Registry::instance()->cloneOrCreateOptions( dbOptions );
    initializeCachePolicy( dbOptions );
    storeProxySettings( _dbOptions );
}

void
TerrainLayer::initializeCachePolicy(const osgDB::Options* options)
{
    // Start with the cache policy passed in by the Map.
    optional<CachePolicy> cp;
    CachePolicy::fromOptions(options, cp);

    // if this layer specifies cache policy info, that will override 
    // whatever the map passed in:
    if ( _initOptions.cachePolicy().isSet() )
        cp->mergeAndOverride( _initOptions.cachePolicy() );

    // finally resolve with global overrides:
    Registry::instance()->resolveCachePolicy( cp );

    setCachePolicy( cp.get() );
}

void
TerrainLayer::storeProxySettings(osgDB::Options* opt)
{
    //Store the proxy settings in the options structure.
    if (_initOptions.proxySettings().isSet())
    {        
        _initOptions.proxySettings().get().apply( opt );
    }
}

SequenceControl*
TerrainLayer::getSequenceControl()
{
    return dynamic_cast<SequenceControl*>( getTileSource() );
}
