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

#define LC "[TerrainLayer] Layer \"" << getName() << "\" "

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

TerrainLayer::CacheBinMetadata::CacheBinMetadata() :
_valid(false)
{
    //nop
}

TerrainLayer::CacheBinMetadata::CacheBinMetadata(const TerrainLayer::CacheBinMetadata& rhs) :
_valid          ( rhs._valid ),
_cacheBinId     ( rhs._cacheBinId ),
_sourceName     ( rhs._sourceName ),
_sourceDriver   ( rhs._sourceDriver ),
_sourceTileSize ( rhs._sourceTileSize ),
_sourceProfile  ( rhs._sourceProfile ),
_cacheProfile   ( rhs._cacheProfile ),   
_cacheCreateTime( rhs._cacheCreateTime )
{
    //nop
}

TerrainLayer::CacheBinMetadata::CacheBinMetadata(const Config& conf)
{
    _valid = !conf.empty();

    conf.getIfSet("cachebin_id", _cacheBinId);
    conf.getIfSet("source_name", _sourceName);
    conf.getIfSet("source_driver", _sourceDriver);
    conf.getIfSet("source_tile_size", _sourceTileSize);
    conf.getObjIfSet("source_profile", _sourceProfile);
    conf.getObjIfSet("cache_profile", _cacheProfile);
    conf.getIfSet("cache_create_time", _cacheCreateTime);

    const Config* extentsRoot = conf.child_ptr("extents");
    if ( extentsRoot )
    {
        const ConfigSet& extents = extentsRoot->children();

        for (ConfigSet::const_iterator i = extents.begin(); i != extents.end(); ++i)
        {
            std::string srsString;
            double xmin, ymin, xmax, ymax;
            optional<unsigned> minLevel, maxLevel;
        
            srsString = i->value("srs");
            xmin = i->value("xmin", 0.0f);
            ymin = i->value("ymin", 0.0f);
            xmax = i->value("xmax", 0.0f);
            ymax = i->value("ymax", 0.0f);
            i->getIfSet("minlevel", minLevel);
            i->getIfSet("maxlevel", maxLevel);

            const SpatialReference* srs = SpatialReference::get(srsString);
            DataExtent e( GeoExtent(srs, xmin,  ymin, xmax, ymax) );
            if (minLevel.isSet())
                e.minLevel() = minLevel.get();
            if (maxLevel.isSet())
                e.maxLevel() = maxLevel.get();

            _dataExtents.push_back(e);
        }
    }

    // check for validity. This will reject older caches that don't have
    // sufficient attribution.
    if (_valid)
    {
        if (!conf.hasValue("source_tile_size") ||
            !conf.hasChild("source_profile") ||
            !conf.hasChild("cache_profile"))
        {
            _valid = false;
        }
    }
}

Config 
TerrainLayer::CacheBinMetadata::getConfig() const
{
    Config conf("osgearth_terrainlayer_cachebin");
    conf.addIfSet("cachebin_id", _cacheBinId);
    conf.addIfSet("source_name", _sourceName);
    conf.addIfSet("source_driver", _sourceDriver);
    conf.addIfSet("source_tile_size", _sourceTileSize);
    conf.addObjIfSet("source_profile", _sourceProfile);
    conf.addObjIfSet("cache_profile", _cacheProfile);
    conf.addIfSet("cache_create_time", _cacheCreateTime);

    if (!_dataExtents.empty())
    {
        Config extents;
        for (DataExtentList::const_iterator i = _dataExtents.begin(); i != _dataExtents.end(); ++i)
        {
            Config extent;
            extent.set("srs", i->getSRS()->getHorizInitString());
            extent.set("xmin", i->xMin());
            extent.set("ymin", i->yMin());
            extent.set("xmax", i->xMax());
            extent.set("ymax", i->yMax());
            extent.addIfSet("minlevel", i->minLevel());
            extent.addIfSet("maxlevel", i->maxLevel());
            extents.add("extent", extent);
        }
        conf.add("extents", extents);
    }

    return conf;
}

//------------------------------------------------------------------------

TerrainLayer::TerrainLayer(const TerrainLayerOptions& initOptions,
                           TerrainLayerOptions*       runtimeOptions ) :
_initOptions   ( initOptions ),
_runtimeOptions( runtimeOptions ),
_openCalled( false ),
_tileSize( 256 )
{
    // nop
}

TerrainLayer::TerrainLayer(const TerrainLayerOptions& initOptions,
                           TerrainLayerOptions*       runtimeOptions,
                           TileSource*                tileSource ) :
_initOptions   ( initOptions ),
_runtimeOptions( runtimeOptions ),
_tileSource    ( tileSource ),
_openCalled( false ),
_tileSize( 256 )
{
    // nop
}

TerrainLayer::~TerrainLayer()
{
    //nop
}

void
TerrainLayer::init()
{    
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

const Status&
TerrainLayer::open()
{
    if ( !_openCalled )
    {
        CacheSettings* cacheSettings = getCacheSettings(); // guaranteed to return non-null

        Threading::ScopedMutexLock lock(_mutex);
        if (!_openCalled)
        {
            // If you created the layer with a pre-created tile source, it will already by set.
            if (!_tileSource.valid())
            {
                osg::ref_ptr<TileSource> ts;

                // as long as we're not in cache-only mode, try to create the TileSource.
                if (cacheSettings->cachePolicy()->isCacheOnly())
                {
                    OE_INFO << LC << "Opening in cache-only mode\n";
                }
                else
                {
                    // Initialize the tile source once and only once.
                    ts = createTileSource();
                }

                if ( ts.valid() )
                {
                    if (cacheSettings->isCacheEnabled())
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
                            OE_INFO << LC << "driver says min valid timestamp = " << DateTime(*cp.minTime()).asRFC1123() << "\n";
                        }
                    }

                    // All is well - set the tile source.
                    if ( !_tileSource.valid() )
                    {
                        _tileSource = ts.release();
                    }
                }
            }
            else
            {
                // User supplied the tile source, so attempt to get its profile:
                _profile = _tileSource->getProfile();
                if (!_profile.valid())
                {
                    setStatus( Status::Error(getName(), "Cannot establish profile") );
                }
            }

            _openCalled = true;
                        
            OE_INFO << LC << cacheSettings->toString() << "\n";
        }

    }

    return _status;
    //return _runtimeOptions->enabled() == true;
}

CacheSettings*
TerrainLayer::getCacheSettings() const
{
    if (!_cacheSettings.valid())
    {
        Threading::ScopedMutexLock lock(_mutex);
        if (!_cacheSettings.valid())
        {
            // clone the existing one if it exists:
            CacheSettings* oldSettings = CacheSettings::get(_readOptions.get());
            _cacheSettings = oldSettings ? new CacheSettings(*oldSettings) : new CacheSettings();

            // install the effective policy (which comes from the tile source). We don't call
            // integrateCachePolicy here because we want this to take precedend over the registry's
            // global overrides.
            _cacheSettings->cachePolicy()->mergeAndOverride(_effectiveCachePolicy);

            // install the layer policy
            _cacheSettings->integrateCachePolicy(_runtimeOptions->cachePolicy());

            // if all it well, open and activate a caching bin for this layer.
            if (_cacheSettings->isCacheEnabled())
            {
                CacheBin* bin = _cacheSettings->getCache()->addBin(_runtimeOptions->cacheId().get());
                if (bin)
                {
                    _cacheSettings->setCacheBin(bin);
                    OE_INFO << LC << "Cache bin is [" << bin->getID() << "]\n";
                }
            }

            _cacheSettings->store(_readOptions.get());
        }
    }
    return _cacheSettings.get();
}

void
TerrainLayer::setTargetProfileHint( const Profile* profile )
{
    _targetProfileHint = profile;

    // Re-read the  cache policy hint since it may change due to the target profile change.
    refreshTileSourceCachePolicyHint( getTileSource() );
}

void
TerrainLayer::refreshTileSourceCachePolicyHint(TileSource* ts)
{
    if ( ts && getCacheSettings() && !_initOptions.cachePolicy().isSet() )
    {
        CachePolicy hint = ts->getCachePolicyHint( _targetProfileHint.get() );

        if ( hint.usage().isSetTo(CachePolicy::USAGE_NO_CACHE) )
        {
            getCacheSettings()->cachePolicy() = hint;
            OE_INFO << LC << "Caching disabled (by policy hint)" << std::endl;
        }
    }
}

TileSource*
TerrainLayer::getTileSource() const
{
    return _tileSource.get();
}

const Profile*
TerrainLayer::getProfile() const
{
    return _profile.get();
}

unsigned
TerrainLayer::getTileSize() const
{
    // force tile source initialization (which sets _tileSize)
    //getTileSource();
    return _tileSize; //ts ? ts->getPixelsPerTile() : _tileSize;
}

bool
TerrainLayer::isDynamic() const
{
    TileSource* ts = getTileSource();
    return ts ? ts->isDynamic() : false;
}

std::string
TerrainLayer::getMetadataKey(const Profile* profile) const
{
    if (profile)
        return Stringify() << profile->getHorizSignature() << "_metadata";
    else
        return "_metadata";
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

    CacheBin* bin = cacheSettings->getCacheBin();
    if (!bin)
        return 0L;

    // does the metadata need initializing?
    std::string metaKey = getMetadataKey(profile);

    Threading::ScopedMutexLock lock(_mutex);

    CacheBinMetadataMap::iterator i = _cacheBinMetadata.find(metaKey);
    if (i == _cacheBinMetadata.end())
    {
        std::string cacheId = _runtimeOptions->cacheId().get();

        // read the metadata record from the cache bin:
        ReadResult rr = bin->readString(metaKey, _readOptions.get());
            
        osg::ref_ptr<CacheBinMetadata> meta;
        bool metadataOK = false;

        if (rr.succeeded())
        {
            // Try to parse the metadata record:
            Config conf;
            conf.fromJSON(rr.getString());
            meta = new CacheBinMetadata(conf);

            if (meta->isOK())
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
                            << getTileSource()->getOptions().getDriver() << "\" cache, but a \""
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
            else
            {
                OE_WARN << LC << "Metadata appears to be corrupt.\n";
            }
        }

        if (!metadataOK)
        {
            // cache metadata does not exist, so try to create it. A valid TileSource is necessary for this.
            if ( getTileSource() && getProfile() )
            {
                meta = new CacheBinMetadata();

                // no existing metadata; create some.
                meta->_cacheBinId      = cacheId;
                meta->_sourceName      = this->getName();
                meta->_sourceDriver    = getTileSource()->getOptions().getDriver();
                meta->_sourceTileSize  = getTileSize();
                meta->_sourceProfile   = getProfile()->toProfileOptions();
                meta->_cacheProfile    = profile->toProfileOptions();
                meta->_cacheCreateTime = DateTime().asTimeStamp();
                meta->_dataExtents     = getTileSource()->getDataExtents();

                // store it in the cache bin.
                std::string data = meta->getConfig().toJSON(false);
                bin->write(metaKey, new StringObject(data), _readOptions.get());                   

                bin->setMetadata(meta.get());
            }

            else if ( cacheSettings->cachePolicy()->isCacheOnly() )
            {
                disable(Stringify() <<
                    "Failed to open a cache for layer "
                    "because cache_only policy is in effect and bin [" << cacheId << "] "
                    "could not be located.");

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

        // If we loaded a profile from the cache metadata, apply the overrides:
        applyProfileOverrides();

        if (meta.valid())
        {
            _cacheBinMetadata[metaKey] = meta.get();
            OE_DEBUG << LC << "Established metadata for cache bin [" << cacheId << "]" << std::endl;
        }
    }

    return bin;
}

void
TerrainLayer::disable(const std::string& msg)
{
    if (!_runtimeOptions->enabled().isSetTo(false))
    {
        _runtimeOptions->enabled() = false;
        _status = Status::Error(msg);
    }
}

TerrainLayer::CacheBinMetadata*
TerrainLayer::getCacheBinMetadata(const Profile* profile)
{
    if (!profile)
        return 0L;

    Threading::ScopedMutexLock lock(_mutex);

    CacheBinMetadataMap::iterator i = _cacheBinMetadata.find(getMetadataKey(profile));
    return i != _cacheBinMetadata.end() ? i->second.get() : 0L;
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
            OE_INFO << LC << "Creating \"" << _runtimeOptions->driver()->getDriver() << "\" driver\n";
            ts = TileSourceFactory::create( *_runtimeOptions->driver() );
            if ( !ts.valid() )
            {
                setStatus(Status::Error(Status::ServiceUnavailable, Stringify()<<"Failed to find driver \"" << _runtimeOptions->driver()->getDriver() << "\"\n"));
                return 0L;
            }
        }
    }

    Status tileSourceStatus;

    // Initialize the profile with the context information:
    if ( ts.valid() )
    {
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
        tileSourceStatus = ts->getStatus();
        if (!tileSourceStatus.isOK())
        {
            tileSourceStatus = ts->open(TileSource::MODE_READ, _readOptions.get());
        }

        if ( tileSourceStatus.isOK() )
        {
            _tileSize = ts->getPixelsPerTile();
        }
        else
        {
            //OE_WARN << LC << "Driver initialization failed: " << tileSourceStatus.message() << std::endl;
            ts = NULL;
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
    else if (getCacheSettings()->isCacheEnabled())
    {
        OE_NOTICE << LC << "Failed to create \"" << _runtimeOptions->driver()->getDriver() << "\" driver, but a cache may exist, so falling back on cache-only mode." << std::endl;
        getCacheSettings()->cachePolicy() = CachePolicy::CACHE_ONLY;
    }

    // Finally: if we could not open a TileSource, and there's no cache available, 
    // just disable the layer.
    else
    {
        disable(tileSourceStatus.message());
        setStatus(tileSourceStatus);
    }
    //if (!ts.valid() && getCacheSettings()->isCacheDisabled())
    //{
    //    disable("Could not initialize TileSource and no cache is available");
    //}

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
    if (getCacheSettings()->isCacheDisabled())
        return false;

    else if (getCacheSettings()->cachePolicy()->isCacheOnly())
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
    _readOptions = Registry::cloneOrCreateOptions(readOptions);

    // store HTTP proxy settings in the options:
    storeProxySettings( _readOptions );
    
    // store the referrer for relative-path resolution
    URIContext( _runtimeOptions->referrer() ).store( _readOptions.get() );

    Threading::ScopedMutexLock lock(_mutex);
    _cacheSettings = 0L;
    _cacheBinMetadata.clear();
}

bool
TerrainLayer::getDataExtents(DataExtentList& output) const
{
    output.clear();

    // if a tile source is available, get the extents directly from it:
    if (getTileSource())
        output = getTileSource()->getDataExtents();

    // otherwise, try the cache. Extents are the same regardless of
    // profile so just use the first one available:
    else if (!_cacheBinMetadata.empty())
        output = _cacheBinMetadata.begin()->second->_dataExtents;

    return !output.empty();
}

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
