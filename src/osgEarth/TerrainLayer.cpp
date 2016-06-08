/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2015 Pelican Mapping
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
    conf.updateObjIfSet( "proxy",        _proxySettings );

    if ( _cachePolicy.isSet() && !_cachePolicy->empty() )
        conf.updateObjIfSet( "cache_policy", _cachePolicy );

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
_runtimeOptions( runtimeOptions ),
_openCalled( false )
{
    init();
}

TerrainLayer::TerrainLayer(const TerrainLayerOptions& initOptions,
                           TerrainLayerOptions*       runtimeOptions,
                           TileSource*                tileSource ) :
_initOptions   ( initOptions ),
_runtimeOptions( runtimeOptions ),
_tileSource    ( tileSource ),
_openCalled( false )
{
    init();
}

TerrainLayer::~TerrainLayer()
{
    if ( _cache.valid() )
    {
        //Threading::ScopedWriteLock exclusive( _cacheBinsMutex );
        //for( CacheBinInfoMap::iterator i = _cacheBins.begin(); i != _cacheBins.end(); ++i )
        //{
        //    CacheBinInfo& info = i->second;
        //    if ( info._bin.valid() )
        //    {
        //        _cache->removeBin( info._bin.get() );
        //    }
        //}
    }
}

void
TerrainLayer::init()
{
    _tileSourceInitAttempted = false;
    _tileSize                = 256;
    
    // intiailize out read-options, which store caching and IO information.
    setReadOptions(0L);

    // Create an L2 mem cache that sits atop the main cache, if necessary.
    // For now: use the same L2 cache size at the driver.
    int l2CacheSize = _initOptions.driver()->L2CacheSize().get();
    
    // See if it was overridden with an env var.
    char const* l2env = ::getenv( "OSGEARTH_L2_CACHE_SIZE" );
    if ( l2env )
    {
        l2CacheSize = as<int>( std::string(l2env), 0 );
        OE_INFO << LC << "L2 cache size set from environment = " << l2CacheSize << "\n";
    }

    // Env cache-only mode also disables the L2 cache.
    char const* noCacheEnv = ::getenv( "OSGEARTH_MEMORY_PROFILE" );
    if ( noCacheEnv )
    {
        l2CacheSize = 0;
    }

    // Initialize the l2 cache if it's size is > 0
    if ( l2CacheSize > 0 )
    {
        _memCache = new MemCache( l2CacheSize );
    }

    // create the unique cache ID for the cache bin.
    std::string cacheId;

    if (_runtimeOptions->cacheId().isSet() && !_runtimeOptions->cacheId()->empty())
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
        Config layerConf = _runtimeOptions->getConfig(true);
        Config driverConf = _runtimeOptions->driver()->getConfig();
        Config hashConf = driverConf - layerConf;

        // remove cache-control properties before hashing.
        hashConf.remove("cache_only");
        hashConf.remove("cache_enabled");
        hashConf.remove("cache_policy");
        hashConf.remove("cacheid");
        hashConf.remove("l2_cache_size");

        // need this, b/c data is vdatum-transformed before caching.
        if (layerConf.hasValue("vdatum"))
            hashConf.add("vdatum", layerConf.value("vdatum"));

        unsigned hash = osgEarth::hashString(hashConf.toJSON());
        cacheId = Stringify() << std::hex << std::setw(8) << std::setfill('0') << hash;

        _runtimeOptions->cacheId().init(cacheId); // set as default value
    }
}

bool
TerrainLayer::open()
{
    if ( !_openCalled )
    {
        Threading::ScopedMutexLock lock(_mutex);
        if (!_openCalled)
        {
            // If you created the layer with a pre-created tile source, it will already by set.
            if (!_tileSource.valid())
            {
                osg::ref_ptr<TileSource> ts;

                // as long as we're not in cache-only mode, try to create the TileSource.
                if (!_runtimeOptions->cachePolicy().isSetTo(CachePolicy::CACHE_ONLY))
                {
                    // Initialize the tile source once and only once.
                    ts = createTileSource();
                }

                if ( ts.valid() )
                {
                    // read the cache policy hint from the tile source unless user expressly set 
                    // a policy in the initialization options. In other words, the hint takes
                    // ultimate priority (even over the Registry override) unless expressly
                    // overridden in the layer options!
                    refreshTileSourceCachePolicyHint( ts.get() );

                    // Unless the user has already configured an expiration policy, use the "last modified"
                    // timestamp of the TileSource to set a minimum valid cache entry timestamp.
                    CachePolicy& cp = _runtimeOptions->cachePolicy().mutable_value();
                    if ( !cp.minTime().isSet() && !cp.maxAge().isSet() && ts->getLastModifiedTime() > 0)
                    {
                        // The "effective" policy overrides the runtime policy, but it does not get serialized.
                        _effectiveCachePolicy = cp;
                        _effectiveCachePolicy->minTime() = ts->getLastModifiedTime();
                        OE_INFO << LC << "cache min valid time reported by driver = " << DateTime(*cp.minTime()).asRFC1123() << "\n";
                    }
                    OE_INFO << LC << "cache policy = " << getCachePolicy().usageString() << std::endl;

                    // All is well - set the tile source.
                    if ( !_tileSource.valid() )
                    {
                        _tileSource = ts.release();
                    }
                }
            }

            _openCalled = true;
        }

    }

    return _tileSource.valid();
}

#if 0
namespace
{
    struct CreateSettingsCallback : public CacheSettings::CreateCallback
    {
        optional<CachePolicy> _effectivePolicy, _layerPolicy;

        CreateSettingsCallback(const optional<CachePolicy>& effective, const optional<CachePolicy>& layer) :
            _effectivePolicy(effective), _layerPolicy(layer) { }

        void onCreate(CacheSettings* settings)
        {
            // if we calculated an effective policy based on the tile source, apply it:
            settings->cachePolicy()->mergeAndOverride( _effectivePolicy );
            
            // merge in any cache policy specified for this layer in particular:
            settings->cachePolicy()->mergeAndOverride( _layerPolicy );
        }
    };
}

CacheSettings*
TerrainLayer::getOrCreateCacheSettings(const std::string& id)
{
    CacheSettings* settings = 0L;

    CacheManager* cm = CacheManager::get(_readOptions.get());
    if ( cm )
    {
        bool isNew = false;
        settings = cm->getOrCreateSettings(
            id, 
            new CreateSettingsCallback(_effectiveCachePolicy, _runtimeOptions->cachePolicy()));
    }

    return settings;
}
#endif

CacheSettings*
TerrainLayer::getCacheSettings()
{
    if (!_cacheSettings.valid())
    {
        Threading::ScopedMutexLock lock(_mutex);
        if (!_cacheSettings.valid())
        {
            CacheManager* cm = CacheManager::get(_readOptions.get());
            if (cm)
            {
                // create a new cache settings object for this layer:
                _cacheSettings = cm->getOrCreateSettings(this->getUID());
                if (_cacheSettings.valid())
                {
                    // if we calculated an effective policy based on the tile source, apply it:
                    _cacheSettings->cachePolicy()->mergeAndOverride(_effectiveCachePolicy);

                    // merge in any cache policy specified for this layer in particular:
                    _cacheSettings->cachePolicy()->mergeAndOverride(_runtimeOptions->cachePolicy());
                }
            }
        }
    }
    return _cacheSettings.get();
}

const CachePolicy&
TerrainLayer::getCachePolicy() const
{
    if (_cacheSettings.valid())
        return _cacheSettings->cachePolicy().get();
    else
        return _initOptions.cachePolicy().get();
}

bool
TerrainLayer::isCacheOnly() const
{
    return getCachePolicy().usage() == CachePolicy::USAGE_CACHE_ONLY;
}

bool
TerrainLayer::isNoCache() const
{
    return getCachePolicy().usage() == CachePolicy::USAGE_NO_CACHE;
}

void
TerrainLayer::setTargetProfileHint( const Profile* profile )
{
    _targetProfileHint = profile;

    // This will attempt to open and access a cache bin if there
    // is one. This is important in cache-only mode since we cannot
    // establish a profile from the tile source.
    CacheBin* bin = getCacheBin(profile);
    if ( bin )
    {
        getCacheBinMetadata( profile );
    }

    // If the tilesource was already initialized, re-read the 
    // cache policy hint since it may change due to the target
    // profile change.
    refreshTileSourceCachePolicyHint( getTileSource() );
}

void
TerrainLayer::refreshTileSourceCachePolicyHint(TileSource* ts)
{
    if ( ts && !_initOptions.cachePolicy().isSet() )
    {
        CachePolicy hint = ts->getCachePolicyHint( _targetProfileHint.get() );

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
    return _tileSource.get();
}

#if 0
TileSource* 
TerrainLayer::getTileSource() const
{
    if ( !_tileSourceInitAttempted )
    {
        // Lock and double check:
        Threading::ScopedMutexLock lock( _initTileSourceMutex );
        if ( !_tileSourceInitAttempted )
        {
            // Continue with thread-safe initialization.
            TerrainLayer* this_nc = const_cast<TerrainLayer*>(this);

            osg::ref_ptr<TileSource> ts;
            if ( !isCacheOnly() )
            {
                // Initialize the tile source once and only once.
                ts = this_nc->createTileSource();
            }

            if ( ts.valid() )
            {
                // read the cache policy hint from the tile source unless user expressly set 
                // a policy in the initialization options. In other words, the hint takes
                // ultimate priority (even over the Registry override) unless expressly
                // overridden in the layer options!
                this_nc->refreshTileSourceCachePolicyHint( ts.get() );

                // Unless the user has already configured an expiration policy, use the "last modified"
                // timestamp of the TileSource to set a minimum valid cache entry timestamp.
                if ( ts )
                {
                    CachePolicy& cp = _runtimeOptions->cachePolicy().mutable_value();
                    if ( !cp.minTime().isSet() && !cp.maxAge().isSet() && ts->getLastModifiedTime() > 0)
                    {
                        // The "effective" policy overrides the runtime policy, but it does not
                        // get serialized.
                        this_nc->_effectiveCachePolicy = cp;
                        this_nc->_effectiveCachePolicy->minTime() = ts->getLastModifiedTime();
                        OE_INFO << LC << "cache min valid time reported by driver = " << DateTime(*cp.minTime()).asRFC1123() << "\n";
                    }
                    OE_INFO << LC << "cache policy = " << getCachePolicy().usageString() << std::endl;
                }

                if ( !_tileSource.valid() )
                    this_nc->_tileSource = ts.release();
            }

            // finally, set this whether we succeeded or failed
            _tileSourceInitAttempted = true;
        }
    }

    return _tileSource.get();
}
#endif

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
    if ( !_openCalled )
    {
        OE_WARN << LC << "Illegal- called getCacheBin() before calling open()\n";
        return 0L;
    }

    CacheSettings* cacheSettings = getCacheSettings();
    if (!cacheSettings)
        return 0L;

    if (cacheSettings->cachePolicy()->isCacheDisabled())
        return 0L;
    
    std::string cacheId = _runtimeOptions->cacheId().get();

    CacheBin* bin = cacheSettings->getCache()->getBin(cacheId);
    if (bin)
        return bin;


    Threading::ScopedMutexLock lock(_mutex);

    bin = cacheSettings->getCache()->getBin(cacheId);
    if (bin)
        return bin;

    bin = cacheSettings->getCache()->addBin(cacheId);
    if (!bin)
    {
        // cache bin creation failed, so disable the caching for this layer:
        cacheSettings->cachePolicy() = CachePolicy::NO_CACHE;
        return 0L;
    }

    // read the metadata record from the cache bin:
    std::string metaKey = Stringify() << profile->getHorizSignature() << "_metadata";
    ReadResult rr = bin->readString(metaKey, _readOptions.get());

    bool metadataOK = false;
    if (rr.succeeded())
    {
        // Try to parse the metadata record:
        Config conf;
        conf.fromJSON(rr.getString());
        osg::ref_ptr<CacheBinMetadata> meta = new CacheBinMetadata(conf);

        if (meta.valid() && meta->isOK())
        {
            metadataOK = true;

            // verify that the cache if compatible with the open tile source:
            if ( getTileSource() && getProfile() )
            {
                //todo: check the profile too
                if ( meta->_sourceDriver.get() != getTileSource()->getOptions().getDriver() )
                {                     
                    OE_WARN << LC 
                        << "Layer \"" << getName() << "\" is requesting a \""
                        << getTileSource()->getOptions().getDriver() << " cache, but a \""
                        << meta->_sourceDriver.get() << "\" cache exists at the specified location. "
                        << "The cache will ignored for this layer.\n";

                    cacheSettings->cachePolicy() = CachePolicy::NO_CACHE;
                    return 0L;
                }
            }   

            // if not, see if we're in cache-only mode and still need a profile:
            else if (cacheSettings->cachePolicy()->isCacheOnly() && !_profile.valid())
            {
                // in cacheonly mode, create a profile from the first cache bin accessed
                // (they SHOULD all be the same...)
                _profile = Profile::create( meta->_sourceProfile.get() );
                _tileSize = meta->_sourceTileSize.get();
            }

            bin->setMetadata(meta.get());
        }
    }

    if (!metadataOK)
    {
        // cache metadata does not exist, so try to create it. A valid TileSource is necessary for this.
        if ( getTileSource() && getProfile() )
        {
            osg::ref_ptr<CacheBinMetadata> meta = new CacheBinMetadata();

            // no existing metadata; create some.
            meta->_cacheBinId      = cacheId;
            meta->_sourceName      = this->getName();
            meta->_sourceDriver    = getTileSource()->getOptions().getDriver();
            meta->_sourceTileSize  = getTileSize();
            meta->_sourceProfile   = getProfile()->toProfileOptions();
            meta->_cacheProfile    = profile->toProfileOptions();
            meta->_cacheCreateTime = DateTime().asTimeStamp();

            // store it in the cache bin.
            std::string data = meta->getConfig().toJSON(false);
            bin->write(data, new StringObject(data), _readOptions.get());                   

            bin->setMetadata(meta.get());
        }

        else if ( cacheSettings->cachePolicy()->isCacheOnly() )
        {
            OE_WARN << LC <<
                "Failed to open a cache for layer "
                "because cache_only policy is in effect and bin [" << cacheId << "] "
                "could not be located."
                << std::endl;
            return 0L;
        }

        else
        {
            OE_WARN << LC <<
                "Failed to create cache bin [" << cacheId << "] "
                "because there is no valid tile source."
                << std::endl;

            cacheSettings->cachePolicy() = CachePolicy::NO_CACHE;
            return 0L;
        }
    }

    OE_INFO << LC << "Opened cache bin [" << cacheId << "]" << std::endl;

    // If we loaded a profile from the cache metadata, apply the overrides:
    applyProfileOverrides();

    return bin;
}

#if 0
CacheBin*
TerrainLayer::getCacheBin(const Profile* profile, const std::string& binId)
{
    if ( !_openCalled )
    {
        OE_WARN << LC << "Illegal- called getCacheBin() before calling open()\n";
        return 0L;
    }

    // access the cache setting for this layer.
    CacheSettings* cacheSettings = getOrCreateCacheSettings();
    if ( !cacheSettings )
        return 0L;

    // if there's no caching, bail
    if (cacheSettings->cachePolicy()->isCacheDisabled())
        return 0L;

    CacheBin* bin = 0L;
    {
        Threading::ScopedMutexLock lock(_mutex);

        // find the cache bin for the provided binID. There can be multiple bins (one per profile).
        
        bin = cacheSettings->getOrCreateCacheBin( _runtimeOptions->cacheId().get() );

        // if we can't create a bin, drop back into NO_CACHE mode and bail.
        if ( !bin )
        {
            OE_INFO << LC << "Unable to open a cache bin; falling back on NO_CACHE mode\n";
            cacheSettings->cachePolicy() = CachePolicy::NO_CACHE;
            return 0L;
        }
        
        // attempt to read the cache metadata:
        osg::ref_ptr<CacheBinMetadata> meta = new CacheBinMetadata( bin->readMetadata() );

        // if the metadata exists and is valid, use it:
        if ( meta.valid() && meta->isOK() )
        {
            // verify that the cache if compatible with the open tile source:
            if ( getTileSource() && getProfile() )
            {
                //todo: check the profile too
                if ( *meta->_sourceDriver != getTileSource()->getOptions().getDriver() )
                {                     
                    OE_WARN << LC 
                        << "Layer \"" << getName() << "\" is requesting a \""
                        << getTileSource()->getOptions().getDriver() << " cache, but a \""
                        << *meta->_sourceDriver << "\" cache exists at the specified location. "
                        << "The cache will ignored for this layer.\n";

                    cacheSettings->cachePolicy() = CachePolicy::NO_CACHE;
                    return 0L;
                }
            }   

            // if not, see if we're in cache-only mode and still need a profile:
            else if (cacheSettings->cachePolicy()->isCacheOnly() && !_profile.valid())
            {
                // in cacheonly mode, create a profile from the first cache bin accessed
                // (they SHOULD all be the same...)
                _profile = Profile::create( *meta->_sourceProfile );
                _tileSize = *meta->_sourceTileSize;
            }
        }

        else
        {
            // cache metadata does not exist, so try to create it. A valid TileSource is necessary for this.
            if ( getTileSource() && getProfile() )
            {
                // no existing metadata; create some.
                meta->_cacheBinId      = binId;
                meta->_sourceName      = this->getName();
                meta->_sourceDriver    = getTileSource()->getOptions().getDriver();
                meta->_sourceTileSize  = getTileSize();
                meta->_sourceProfile   = getProfile()->toProfileOptions();
                meta->_cacheProfile    = profile->toProfileOptions();
                meta->_cacheCreateTime = DateTime().asTimeStamp();

                // store it in the cache bin.
                bin->writeMetadata( meta->getConfig() );
            }

            else if ( cacheSettings->cachePolicy()->isCacheOnly() )
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

                cacheSettings->cachePolicy() = CachePolicy::NO_CACHE;
                return 0L;
            }
        }

        // store out metadata in the cache settings for this bin as well.
        if (meta.valid())
        {
            bin->setMetadata(meta.get());
        }

        OE_INFO << LC << "Opened cache bin [" << binId << "]" << std::endl;

        // If we loaded a profile from the cache metadata, apply the overrides:
        applyProfileOverrides();
    }

    return bin;
}
#endif

TerrainLayer::CacheBinMetadata*
TerrainLayer::getCacheBinMetadata(const Profile* profile)
{
    // the cache bin ID is the cache IF concatenated with the profile signature.
    //std::string binId = *_runtimeOptions->cacheId() + std::string("_") + profile->getFullSignature();

    CacheSettings* settings = getCacheSettings();
    if (settings && settings->getCacheBin())
    {
        return dynamic_cast<CacheBinMetadata*>( settings->getCacheBin()->getMetadata() );
    }
    return 0L;
}

TileSource*
TerrainLayer::createTileSource()
{
    osg::ref_ptr<TileSource> ts;

    if ( _tileSource.valid() )
    {
        // this will happen if the layer was created with an explicit TileSource instance.
        ts = _tileSource.get();
    }

    else
    {
        // Instantiate it from driver options if it has not already been created.
        // This will also set a manual "override" profile if the user provided one.
        if ( _runtimeOptions->driver().isSet() )
        {
            OE_INFO << LC << "Creating TileSource, driver = \"" << _runtimeOptions->driver()->getDriver() << "\"\n";
            ts = TileSourceFactory::create( *_runtimeOptions->driver() );
            if ( !ts.valid() )
            {
                OE_WARN << LC << "Failed to create TileSource for driver \"" << _runtimeOptions->driver()->getDriver() << "\"\n";
            }
        }
    }

    // Initialize the profile with the context information:
    if ( ts.valid() )
    {
        //// set up the URI options.
        //if ( !_readOptions.valid() )
        //{
        //    _readOptions = Registry::instance()->cloneOrCreateOptions();

        //    if ( _cache.valid() )
        //        _cache->store( _readOptions.get() );

        //    _initOptions.cachePolicy()->store( _readOptions.get() );

        //    URIContext( _runtimeOptions->referrer() ).store( _readOptions.get() );
        //}

        // add the osgDB options string if it's set.
        const optional<std::string>& osgOptions = ts->getOptions().osgOptionString();
        if ( osgOptions.isSet() && !osgOptions->empty() )
        {
            std::string s = _readOptions->getOptionString();
            if ( !s.empty() )
                s = Stringify() << osgOptions.get() << " " << s;
            else
                s = osgOptions.get();
            _readOptions->setOptionString( s );
        }

        // report on a manual override profile:
        if ( ts->getProfile() )
        {
            OE_INFO << LC << "Override profile: "  << ts->getProfile()->toString() << std::endl;
        }

        // Open the tile source (if it hasn't already been started)
        TileSource::Status status = ts->getStatus();
        if ( status != TileSource::STATUS_OK )
        {
            status = ts->open(TileSource::MODE_READ, _readOptions.get());
        }

        if ( status == TileSource::STATUS_OK )
        {
            _tileSize = ts->getPixelsPerTile();

#if 0 //debugging 
            // dump out data extents:
            if ( ts->getDataExtents().size() > 0 )
            {
                OE_INFO << LC << "Data extents reported:" << std::endl;
                for(DataExtentList::const_iterator i = ts->getDataExtents().begin();
                    i != ts->getDataExtents().end(); ++i)
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
            ts = NULL;
            //_tileSourceInitFailed = true;
            _runtimeOptions->enabled() = true;
        }
    }

    // Set the profile from the TileSource if possible:
    if ( ts.valid() )
    {
        if ( !_profile.valid() )
        {
            OE_DEBUG << LC << "Get Profile from tile source" << std::endl;
            _profile = ts->getProfile();
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

    return ts.release();
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
TerrainLayer::setReadOptions(const osgDB::Options* readOptions)
{
    // clone the options, or create it not set
    _readOptions = Registry::instance()->cloneOrCreateOptions(readOptions);

//    _cacheSettings = 0L;

#if 0 // done elsewhere (in getOrCreateCacheSettings)
    // if there's a layer-specific cache policy, integrate it now:
    if (_initOptions.cachePolicy().isSet())
    {
        CacheSettings* settings = getOrCreateCacheSettings();
        if (settings)
        {
            if (_initOptions.cachePolicy().isSet())
            {
                settings->cachePolicy()->mergeAndOverride(_initOptions.cachePolicy());
            }

            Registry::instance()->resolveCachePolicy(settings->cachePolicy());
        }
    }
#endif

    // store HTTP proxy settings in the options:
    storeProxySettings( _readOptions );
    
    // store the referrer for relative-path resolution
    URIContext( _runtimeOptions->referrer() ).store( _readOptions.get() );
}

#if 0
void
TerrainLayer::initializeCachePolicy(const osgDB::Options* options)
{
    // Start with the cache policy passed in by the Map.
    optional<CachePolicy> cp = CachePolicy::get(options);

    // if this layer specifies cache policy info, that will override 
    // whatever the map passed in:
    if ( _initOptions.cachePolicy().isSet() )
        cp->mergeAndOverride( _initOptions.cachePolicy() );

    // finally resolve with global overrides:
    Registry::instance()->resolveCachePolicy( cp );

    setCachePolicy( cp.get() );
}
#endif

void
TerrainLayer::storeProxySettings(osgDB::Options* readOptions)
{
    //Store the proxy settings in the options structure.
    if (_initOptions.proxySettings().isSet())
    {        
        _initOptions.proxySettings()->apply( readOptions );
    }
}

SequenceControl*
TerrainLayer::getSequenceControl()
{
    return dynamic_cast<SequenceControl*>( getTileSource() );
}
