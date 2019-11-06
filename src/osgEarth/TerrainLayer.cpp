/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarth/Registry>
#include <osgEarth/TimeControl>
#include <osgEarth/URI>
#include <osgEarth/Map>

using namespace osgEarth;
using namespace OpenThreads;

#define LC "[TerrainLayer] Layer \"" << getName() << "\" "

//------------------------------------------------------------------------

Config
TerrainLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();

    conf.set( "min_level", _minLevel );
    conf.set( "max_level", _maxLevel );
    conf.set( "min_resolution", _minResolution );
    conf.set( "max_resolution", _maxResolution );
    conf.set( "max_data_level", _maxDataLevel );
    conf.set( "proxy", _proxySettings );
    conf.set( "no_data_value", _noDataValue);
    conf.set( "min_valid_value", _minValidValue);
    conf.set( "max_valid_value", _maxValidValue);
    conf.set( "tile_size", _tileSize);
    conf.set( "profile", _profile);

    return conf;
}

void
TerrainLayer::Options::fromConfig(const Config& conf)
{
    _minLevel.init( 0 );
    _maxLevel.init( 23 );
    _maxDataLevel.init( 99 );
    _tileSize.init( 256 );
    _noDataValue.init( -32768.0f );
    _minValidValue.init(-32767.0f );
    _maxValidValue.init( 32768.0f );

    conf.get( "min_level", _minLevel );
    conf.get( "max_level", _maxLevel );
    conf.get( "min_resolution", _minResolution );
    conf.get( "max_resolution", _maxResolution );
    conf.get( "max_data_level", _maxDataLevel );
    conf.get( "proxy",        _proxySettings );
    conf.get( "no_data_value", _noDataValue);
    conf.get( "nodata_value", _noDataValue); // back compat
    conf.get( "min_valid_value", _minValidValue);
    conf.get( "max_valid_value", _maxValidValue);
    conf.get( "tile_size", _tileSize);
    conf.get( "profile", _profile);

    if (conf.hasValue("driver"))
        driver() = TileSourceOptions(conf);
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

    conf.get("cachebin_id", _cacheBinId);
    conf.get("source_name", _sourceName);
    conf.get("source_driver", _sourceDriver);
    conf.get("source_tile_size", _sourceTileSize);
    conf.get("source_profile", _sourceProfile);
    conf.get("cache_profile", _cacheProfile);
    conf.get("cache_create_time", _cacheCreateTime);

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
            i->get("minlevel", minLevel);
            i->get("maxlevel", maxLevel);

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
    conf.set("cachebin_id", _cacheBinId);
    conf.set("source_name", _sourceName);
    conf.set("source_driver", _sourceDriver);
    conf.set("source_tile_size", _sourceTileSize);
    conf.set("source_profile", _sourceProfile);
    conf.set("cache_profile", _cacheProfile);
    conf.set("cache_create_time", _cacheCreateTime);

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
            extent.set("minlevel", i->minLevel());
            extent.set("maxlevel", i->maxLevel());

            extents.add("extent", extent);
        }
        conf.add("extents", extents);
    }

    return conf;
}

//------------------------------------------------------------------------

OE_LAYER_PROPERTY_IMPL(TerrainLayer, unsigned, MinLevel, minLevel);
OE_LAYER_PROPERTY_IMPL(TerrainLayer, double, MinResolution, minResolution);
OE_LAYER_PROPERTY_IMPL(TerrainLayer, unsigned, MaxLevel, maxLevel);
OE_LAYER_PROPERTY_IMPL(TerrainLayer, double, MaxResolution, maxResolution);
OE_LAYER_PROPERTY_IMPL(TerrainLayer, unsigned, MaxDataLevel, maxDataLevel);


TerrainLayer::~TerrainLayer()
{
    //nop
}

void
TerrainLayer::init()
{
    Layer::init();

    _openCalled = false;
    _writingRequested = false;
    _tileSourceExpected = true;
    _profileMatchesMapProfile = true;

    // intiailize our read-options, which store caching and IO information.
    setReadOptions(0L);

    // Custom tile size?
    if (options().tileSize().isSet())
        _tileSize = options().tileSize().get();
    else
        _tileSize = 256;

    // If the user asked for a custom profile, install it now
    if (options().profile().isSet())
    {
        _profile = Profile::create(options().profile().get());
    }
}

void
TerrainLayer::addedToMap(const Map* map)
{
    VisibleLayer::addedToMap(map);

    unsigned l2CacheSize = 0u;

    // If the profiles don't match, mosaicing will be likely so set up a 
    // small L2 cache for this layer.
    if (map &&
        map->getProfile() &&
        getProfile() &&
        !map->getProfile()->getSRS()->isHorizEquivalentTo(getProfile()->getSRS()))
    {
        _profileMatchesMapProfile = false;
        l2CacheSize = 16u;
        OE_INFO << LC << "Map/Layer profiles differ; requesting L2 cache" << std::endl;
    }

    setUpL2Cache(l2CacheSize);
}

void
TerrainLayer::removedFromMap(const Map* map)
{
    VisibleLayer::removedFromMap(map);
    _profileMatchesMapProfile.unset();
}

void
TerrainLayer::setUpL2Cache(unsigned minSize)
{
    // Check the layer hints
    unsigned l2CacheSize = layerHints().L2CacheSize().getOrUse(minSize);

        // Create an L2 mem cache that sits atop the main cache, if necessary.
        // For now: use the same L2 cache size at the driver.
    if (l2CacheSize == 0u && options().driver()->L2CacheSize().isSet())
        l2CacheSize = options().driver()->L2CacheSize().get();

        // See if it was overridden with an env var.
    char const* l2env = ::getenv("OSGEARTH_L2_CACHE_SIZE");
    if (l2env)
        {
        l2CacheSize = as<int>(std::string(l2env), 0);
            OE_INFO << LC << "L2 cache size set from environment = " << l2CacheSize << "\n";
        }

        // Env cache-only mode also disables the L2 cache.
    char const* noCacheEnv = ::getenv("OSGEARTH_MEMORY_PROFILE");
    if (noCacheEnv)
        {
            l2CacheSize = 0;
        }

        // Initialize the l2 cache if it's size is > 0
    if (l2CacheSize > 0)
        {
        _memCache = new MemCache(l2CacheSize);
        OE_INFO << LC << "L2 cache size = " << l2CacheSize << std::endl;
        }
}

Status
TerrainLayer::openImplementation()
{
    if ( !_openCalled )
    {
        _openCalled = true;

        Status parent = VisibleLayer::openImplementation();
        if (parent.isError())
            return parent;

        // create the unique cache ID for the cache bin.
        if (options().cacheId().isSet() && !options().cacheId()->empty())
        {
            // user expliticy set a cacheId in the terrain layer options.
            // this appears to be a NOP; review for removal -gw
            _runtimeCacheId = options().cacheId().get();
        }
        else
        {
            // system will generate a cacheId from the layer configuration.
            Config hashConf = options().getConfig();

            // remove non-data properties.
            hashConf.remove("name");
            hashConf.remove("enabled");
            hashConf.remove("cacheid");
            hashConf.remove("cache_only");
            hashConf.remove("cache_enabled");
            hashConf.remove("cache_policy");
            hashConf.remove("visible");
            hashConf.remove("l2_cache_size");

            OE_DEBUG << "hashConfFinal = " << hashConf.toJSON(true) << std::endl;

            unsigned hash = osgEarth::hashString(hashConf.toJSON());
            _runtimeCacheId = Stringify() << std::hex << std::setw(8) << std::setfill('0') << hash;
        }

        // Now that we know the cache ID, establish the cache settings for this Layer.
        // Start by cloning whatever CacheSettings were inherited in the read options
        // (typically from the Map).
        CacheSettings* oldSettings = CacheSettings::get(_readOptions.get());
        _cacheSettings = oldSettings ? new CacheSettings(*oldSettings) : new CacheSettings();

        // Store for further propagation!
        _cacheSettings->store(_readOptions.get());

        // Integrate a cache policy from this Layer's options:
        _cacheSettings->integrateCachePolicy(options().cachePolicy());

        // If the layer hints are set, integrate that cache policy last.
        _cacheSettings->integrateCachePolicy(layerHints().cachePolicy());

        // If you created the layer with a pre-created tile source, it will already by set.
        if (!_tileSource.valid())
        {
            osg::ref_ptr<TileSource> ts;

            // as long as we're not in cache-only mode, try to create the TileSource.
            if (_cacheSettings->cachePolicy()->isCacheOnly())
            {
                OE_INFO << LC << "Opening in cache-only mode\n";
            }
            else if (isTileSourceExpected())
            {
                // Initialize the tile source once and only once.
                ts = createAndOpenTileSource();
            }

            // All good
            if (ts.valid() && !_tileSource.valid())
            {
                _tileSource = ts.release();
            }
        }
        else
        {
            // User supplied the tile source, so attempt to initialize it:
            _tileSource = createAndOpenTileSource();
        }

        // Finally, open and activate a caching bin for this layer if it
        // hasn't already been created.
        if (_cacheSettings->isCacheEnabled() && _cacheSettings->getCacheBin() == 0L)
        {
            CacheBin* bin = _cacheSettings->getCache()->addBin(_runtimeCacheId);
            if (bin)
            {
                _cacheSettings->setCacheBin(bin);
                OE_INFO << LC << "Cache bin is [" << bin->getID() << "]\n";
            }
        }
    }

    return getStatus();
}


const Status&
TerrainLayer::openForWriting()
{
    if (isWritingSupported())
    {
        _writingRequested = true;
        return open();
    }
    return setStatus(Status::ServiceUnavailable, "Layer does not support writing");
}

Status
TerrainLayer::closeImplementation()
{
    setProfile(0L);
    _tileSource = 0L;
    _openCalled = false;
    setStatus(Status());
    _readOptions = 0L;
    _cacheSettings = new CacheSettings();
    return Layer::closeImplementation();
}

void
TerrainLayer::establishCacheSettings()
{
    //nop
}

CacheSettings*
TerrainLayer::getCacheSettings() const
{
    return _cacheSettings.get();
}

void
TerrainLayer::setTileSource(TileSource* value)
{
    if (_openCalled)
    {
        OE_WARN << LC << "Illegal: cannot call setTileSource after Layer is open" << std::endl;
        return;
        }
    _tileSource = value;
    setTileSourceExpected(true);    
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

void
TerrainLayer::setProfile(const Profile* profile)
{
    _profile = profile;

    if (getProfile())
    {
        // augment the final profile with any overrides:
        applyProfileOverrides();

        OE_INFO << LC
            << (getProfile()? getProfile()->toString() : "[no profile]") << " "
            << (_cacheSettings.valid()? _cacheSettings->toString() : "[no cache settings]")
            << std::endl;
    }
}

bool
TerrainLayer::isDynamic() const
{
    if (getHints().dynamic().isSetTo(true))
        return true;

    TileSource* ts = getTileSource();
    return ts ? ts->isDynamic() : false;
}

std::string
TerrainLayer::getAttribution() const
{
    // Get the attribution from the layer if it's set.
    if (_options->attribution().isSet())
    {
        return *_options->attribution();
    }

    // Get it from the tilesource if it's not set on the layer.
    TileSource* ts = getTileSource();
    if (ts)
    {
        return ts->getAttribution();
    }
    else
    {
        return "";
    }
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
        OE_WARN << LC << "Illegal- called getCacheBin() before layer is open.. did you call open()?\n";
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
        //std::string cacheId = _runtimeOptions->cacheId().get();

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
                    setProfile( Profile::create(meta->_sourceProfile.get()) );
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
            // cache metadata does not exist, so try to create it.
            if ( getProfile() )
            {
                meta = new CacheBinMetadata();

                // no existing metadata; create some.
                meta->_cacheBinId      = _runtimeCacheId;
                meta->_sourceName      = this->getName();
                meta->_sourceTileSize  = getTileSize();
                meta->_sourceProfile   = getProfile()->toProfileOptions();
                meta->_cacheProfile    = profile->toProfileOptions();
                meta->_cacheCreateTime = DateTime().asTimeStamp();
                meta->_dataExtents     = getDataExtents();

                if (getTileSource())
                {
                    meta->_sourceDriver = getTileSource()->getOptions().getDriver();
                }

                // store it in the cache bin.
                std::string data = meta->getConfig().toJSON(false);
                osg::ref_ptr<StringObject> temp = new StringObject(data);
                bin->write(metaKey, temp.get(), _readOptions.get());

                bin->setMetadata(meta.get());
            }

            else if ( cacheSettings->cachePolicy()->isCacheOnly() )
            {
                disable(Stringify() <<
                    "Failed to open a cache for layer "
                    "because cache_only policy is in effect and bin [" << _runtimeCacheId << "] "
                    "could not be located.");

                return 0L;
            }

            else
            {
                OE_WARN << LC <<
                    "Failed to create cache bin [" << _runtimeCacheId << "] "
                    "because there is no valid profile."
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
            OE_DEBUG << LC << "Established metadata for cache bin [" << _runtimeCacheId << "]" << std::endl;
        }
    }

    return bin;
}

void
TerrainLayer::disable(const std::string& msg)
{
    setStatus(Status::Error(msg));
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
    if (options().driver().isSet())
    {
        OE_INFO << LC << "Creating \"" << options().driver()->getDriver() << "\" driver\n";

        return TileSourceFactory::create(options().driver().get());
    }
    else
    {
        return 0L;
    }
}

TileSource*
TerrainLayer::createAndOpenTileSource()
{
    osg::ref_ptr<TileSource> ts;

    if ( _tileSource.valid() )
    {
        // this will happen if the layer was created with an explicit TileSource instance.
        ts = _tileSource.get();
    }

    else
    {
        ts = createTileSource();

        if (!ts.valid())
        {
            setStatus(Status::Error(Status::ServiceUnavailable, "Failed to load tile source plugin"));
            return 0L;
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

        // If we're setting any custom options, do so now before opening:
        if (options().tileSize().isSet())
            ts->setPixelsPerTile(options().tileSize().get());

        if (options().noDataValue().isSet())
            ts->setNoDataValue(options().noDataValue().get());

        if (options().minValidValue().isSet())
            ts->setMinValidValue(options().minValidValue().get());

        if (options().maxValidValue().isSet())
            ts->setMaxValidValue(options().maxValidValue().get());


        // report on a manual override profile:
        if ( ts->getProfile() )
        {
            OE_INFO << LC << "Override profile: "  << ts->getProfile()->toString() << std::endl;
        }

        // Now that the tile source exists, set up the cache.
        if (_cacheSettings->isCacheEnabled())
        {
            // Unless the user has already configured an expiration policy, use the "last modified"
            // timestamp of the TileSource to set a minimum valid cache entry timestamp.
            const CachePolicy& cp = _cacheSettings->cachePolicy().get();

            if ( !cp.minTime().isSet() && !cp.maxAge().isSet() && ts->getLastModifiedTime() > 0)
            {
                // The "effective" policy overrides the runtime policy, but it does not get serialized.
                _cacheSettings->cachePolicy()->mergeAndOverride( cp );
                _cacheSettings->cachePolicy()->minTime() = ts->getLastModifiedTime();
                OE_INFO << LC << "driver says min valid timestamp = " << DateTime(*cp.minTime()).asRFC1123() << "\n";
            }

            CacheBin* bin = _cacheSettings->getCache()->addBin(_runtimeCacheId);
            if (bin)
            {
                _cacheSettings->setCacheBin(bin);
                OE_INFO << LC << "Cache bin is [" << bin->getID() << "]\n";
            }
        }

        // Open the tile source (if it hasn't already been started)
        tileSourceStatus = ts->getStatus();
        if (!tileSourceStatus.isOK())
        {
            tileSourceStatus = ts->open(TileSource::MODE_READ, _readOptions.get());
        }

        // Now that the tile source is open and ready, propagate any user-set
        // properties to and fro.
        if ( tileSourceStatus.isOK() )
        {
            if (!ts->getDataExtents().empty())
                _dataExtents = ts->getDataExtents();
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
        if (!_profile.valid())
        {
            OE_DEBUG << LC << "Get Profile from tile source" << std::endl;
            setProfile(ts->getProfile());
        }
    }

    // Otherwise, force cache-only mode (since there is no tilesource). The layer will try to
    // establish a profile from the metadata in the cache instead.
    else if (getCacheSettings()->isCacheEnabled() && options().cacheId().isSet())
    {
        OE_WARN << LC << tileSourceStatus.message() << std::endl;
        OE_WARN << LC << "will attempt to use the cache as a fallback data source" << std::endl;
        getCacheSettings()->cachePolicy() = CachePolicy::CACHE_ONLY;
    }

    // Finally: if we could not open a TileSource, and there's no cache available,
    // just disable the layer.
    else
    {
        disable(tileSourceStatus.message());
        setStatus(tileSourceStatus);
    }

    return ts.release();
}

#if 0
bool
TerrainLayer::mayHaveDataInExtent(const GeoExtent& ex) const
{
    if (!ex.isValid())
    {
        // bad extent; no data
        return false;
    }

    const DataExtentList& de = getDataExtents();
    if (de.empty())
    {
        // not enough info, assume yes
        return true;
    }

    // Get extent in local profile:
    GeoExtent localExtent = ex;
    if (getProfile() && !getProfile()->getSRS()->isHorizEquivalentTo(ex.getSRS()))
    {
        localExtent = getProfile()->clampAndTransformExtent(ex);
    }

    // Check union:
    if (getDataExtentsUnion().intersects(localExtent))
    {
        // possible yes
        return true;
    }

    // Check each extent in turn:
    for (DataExtentList::const_iterator i = de.begin(); i != de.end(); ++i)
    {
        if (i->intersects(localExtent))
        {
            // possible yes
            return true;
        }
    }

    // definite no.
    return false;
}
#endif

bool
TerrainLayer::isKeyInLegalRange(const TileKey& key) const
{
    if ( !key.valid() )
    {
        return false;
    }

    // We must use the equivalent lod b/c the input key can be in any profile.
    unsigned localLOD = getProfile() ?
        getProfile()->getEquivalentLOD(key.getProfile(), key.getLOD()) :
        key.getLOD();


    // First check the key against the min/max level limits, it they are set.
    if ((options().maxLevel().isSet() && localLOD > options().maxLevel().value()) ||
        (options().minLevel().isSet() && localLOD < options().minLevel().value()))
    {
        return false;
    }

    // Next check the maxDataLevel if that is set.
    if (options().maxDataLevel().isSet() && localLOD > options().maxDataLevel().get())
    {
        return false;
    }

    // Next, check against resolution limits (based on the source tile size).
    if (options().minResolution().isSet() || options().maxResolution().isSet())
    {
        const Profile* profile = getProfile();
        if ( profile )
        {
            // calculate the resolution in the layer's profile, which can
            // be different that the key's profile.
            double resKey   = key.getExtent().width() / (double)getTileSize();
            double resLayer = key.getProfile()->getSRS()->transformUnits(resKey, profile->getSRS());

            if (options().maxResolution().isSet() &&
                options().maxResolution().value() > resLayer)
            {
                return false;
            }

            if (options().minResolution().isSet() &&
                options().minResolution().value() < resLayer)
            {
                return false;
            }
        }
    }

	return true;
}

bool
TerrainLayer::isKeyInVisualRange(const TileKey& key) const
{
    if (!key.valid())
    {
        return false;
    }

    // We must use the equivalent lod b/c the input key can be in any profile.
    unsigned localLOD = getProfile() ?
        getProfile()->getEquivalentLOD(key.getProfile(), key.getLOD()) :
        key.getLOD();


    // First check the key against the min/max level limits, it they are set.
    if ((options().maxLevel().isSet() && localLOD > options().maxLevel().value()) ||
        (options().minLevel().isSet() && localLOD < options().minLevel().value()))
    {
        return false;
    }

    // Next, check against resolution limits (based on the source tile size).
    if (options().minResolution().isSet() || options().maxResolution().isSet())
    {
        const Profile* profile = getProfile();
        if (profile)
        {
            // calculate the resolution in the layer's profile, which can
            // be different that the key's profile.
            double resKey = key.getExtent().width() / (double)getTileSize();
            double resLayer = key.getProfile()->getSRS()->transformUnits(resKey, profile->getSRS());

            if (options().maxResolution().isSet() &&
                options().maxResolution().value() > resLayer)
            {
                return false;
            }

            if (options().minResolution().isSet() &&
                options().minResolution().value() < resLayer)
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
TerrainLayer::setReadOptions(const osgDB::Options* readOptions)
{
    // clone the options, or create it not set
    _readOptions = Registry::cloneOrCreateOptions(readOptions);
    //Layer::setReadOptions(readOptions);

    // store HTTP proxy settings in the options:
    storeProxySettings( _readOptions.get() );

    // store the referrer for relative-path resolution
    URIContext( options().referrer() ).store( _readOptions.get() );

    Threading::ScopedMutexLock lock(_mutex);
    _cacheSettings = new CacheSettings();
    _cacheBinMetadata.clear();
}

std::string
TerrainLayer::getCacheID() const
{
    return _runtimeCacheId;
}

const DataExtentList&
TerrainLayer::getDataExtents() const
{
    if (!_dataExtents.empty())
    {
        return _dataExtents;
    }

    else if (!_cacheBinMetadata.empty())
    {
        // There are extents in the cache bin, so use those.
        // The DE's are the same regardless of profile so just use the first one in there.
        return _cacheBinMetadata.begin()->second->_dataExtents;
    }

    else
    {
        return _dataExtents;
    }
}

DataExtentList&
TerrainLayer::dataExtents()
{
    return const_cast<DataExtentList&>(getDataExtents());
}

void
TerrainLayer::dirtyDataExtents()
{
    Threading::ScopedMutexLock lock(_mutex);
    _dataExtentsUnion = GeoExtent::INVALID;
}

const GeoExtent&
TerrainLayer::getDataExtentsUnion() const
{
    const DataExtentList& de = getDataExtents();

    if (_dataExtentsUnion.isInvalid() && !de.empty())
    {
        Threading::ScopedMutexLock lock(_mutex);
        {
            if (_dataExtentsUnion.isInvalid() && !de.empty()) // double-check
            {
                GeoExtent e(de[0]);
                for (unsigned int i = 1; i < de.size(); i++)
                {
                    e.expandToInclude(de[i]);
                }
                _dataExtentsUnion = e;
            }
        }
    }
    return _dataExtentsUnion;
}

const GeoExtent&
TerrainLayer::getExtent() const
{
    return getDataExtentsUnion();
}

void
TerrainLayer::storeProxySettings(osgDB::Options* readOptions)
{
    //Store the proxy settings in the options structure.
    if (options().proxySettings().isSet())
    {
        options().proxySettings()->apply( readOptions );
    }
}

SequenceControl*
TerrainLayer::getSequenceControl()
{
    return dynamic_cast<SequenceControl*>( getTileSource() );
}

TileKey
TerrainLayer::getBestAvailableTileKey(const TileKey& key) const
{
    // trivial reject
    if ( !key.valid() )
        return TileKey::INVALID;

    unsigned MDL = options().maxDataLevel().get();

    // We must use the equivalent lod b/c the input key can be in any profile.
    unsigned localLOD = getProfile() ?
        getProfile()->getEquivalentLOD(key.getProfile(), key.getLOD()) :
        key.getLOD();

    // Check against level extrema:
    if (localLOD < options().minLevel().get() || localLOD > options().maxLevel().get())
    {
        return TileKey::INVALID;
    }

    // Next, check against resolution limits (based on the source tile size).
    if (options().minResolution().isSet() || options().maxResolution().isSet())
    {
        const Profile* profile = getProfile();
        if ( profile )
        {
            // calculate the resolution in the layer's profile, which can
            // be different that the key's profile.
            double resKey   = key.getExtent().width() / (double)getTileSize();
            double resLayer = key.getProfile()->getSRS()->transformUnits(resKey, profile->getSRS());

            if (options().maxResolution().isSet() &&
                options().maxResolution().value() > resLayer)
            {
                return TileKey::INVALID;
            }

            if (options().minResolution().isSet() &&
                options().minResolution().value() < resLayer)
            {
                return TileKey::INVALID;
            }
        }
    }

    // Next check against the data extents.
    const DataExtentList& de = getDataExtents();

    // If we have mo data extents available, just return the MDL-limited input key.
    if (de.empty())
    {
        return localLOD > MDL ? key.createAncestorKey(MDL) : key;
    }

    // Transform the key's extent to the layer's extent
    GeoExtent localKeyExtent = getProfile()->clampAndTransformExtent(key.getExtent());

    // Reject if the extents don't overlap at all.
    if (!getDataExtentsUnion().intersects(localKeyExtent))
    {
        return TileKey::INVALID;
    }

    bool     intersects = false;
    unsigned highestLOD = 0;

    // Check each data extent in turn:
    for (DataExtentList::const_iterator itr = de.begin(); itr != de.end(); ++itr)
    {
        // check for 2D intersection:
        if (itr->intersects(localKeyExtent))
        {
            // check that the extent isn't higher-resolution than our key:
            if ( !itr->minLevel().isSet() || localLOD >= (int)itr->minLevel().get() )
            {
                // Got an intersetion; now test the LODs:
                intersects = true;

                // Is the high-LOD set? If not, there's not enough information
                // so just assume our key might be good.
                if ( itr->maxLevel().isSet() == false )
                {
                    return localLOD > MDL ? key.createAncestorKey(MDL) : key;
                }

                // Is our key at a lower or equal LOD than the max key in this extent?
                // If so, our key is good.
                else if ( localLOD <= (int)itr->maxLevel().get() )
                {
                    return localLOD > MDL ? key.createAncestorKey(MDL) : key;
                }

                // otherwise, record the highest encountered LOD that
                // intersects our key.
                else if ( itr->maxLevel().get() > highestLOD )
                {
                    highestLOD = itr->maxLevel().get();
                }
            }
        }
    }

    if ( intersects )
    {
        return key.createAncestorKey(osg::minimum(key.getLOD(), osg::minimum(highestLOD, MDL)));
    }

    return TileKey::INVALID;
}

bool
TerrainLayer::mayHaveData(const TileKey& key) const
{
    return key == getBestAvailableTileKey(key);
}

void
TerrainLayer::setTileSize(const unsigned& value)
{
    options().tileSize() = value;
}

unsigned
TerrainLayer::getTileSize() const
{
    return getTileSource() ? getTileSource()->getPixelsPerTile() : options().tileSize().get();
}

void
TerrainLayer::setNoDataValue(const float& value)
{
    options().noDataValue() = value;
}

float
TerrainLayer::getNoDataValue() const
{
    return getTileSource() ? getTileSource()->getNoDataValue() : options().noDataValue().get();
}

void
TerrainLayer::setMinValidValue(const float& value)
{
    options().minValidValue() = value;
}

float
TerrainLayer::getMinValidValue() const
{
    return getTileSource() ? getTileSource()->getMinValidValue() : options().minValidValue().get();
}

void
TerrainLayer::setMaxValidValue(const float& value)
{
    options().maxValidValue() = value;
}

float
TerrainLayer::getMaxValidValue() const
{
    return getTileSource() ? getTileSource()->getMaxValidValue() : options().maxValidValue().get();
}
