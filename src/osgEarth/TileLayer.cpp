/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <osgEarth/TileLayer>
#include <osgEarth/Registry>
#include <osgEarth/TimeControl>
#include <osgEarth/URI>
#include <osgEarth/Map>
#include <osgEarth/MemCache>

using namespace osgEarth;
using namespace OpenThreads;

#define LC "[TileLayer] Layer \"" << getName() << "\" "

//------------------------------------------------------------------------

Config
TileLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();

    conf.set( "min_level", _minLevel );
    conf.set( "max_level", _maxLevel );
    conf.set( "min_resolution", _minResolution );
    conf.set( "max_resolution", _maxResolution );
    conf.set( "max_data_level", _maxDataLevel );
    conf.set( "no_data_value", _noDataValue);
    conf.set( "min_valid_value", _minValidValue);
    conf.set( "max_valid_value", _maxValidValue);
    conf.set( "tile_size", _tileSize);
    conf.set( "profile", _profile);

    return conf;
}

void
TileLayer::Options::fromConfig(const Config& conf)
{
    _minLevel.init( 0 );
    _maxLevel.init( 23 );
    _maxDataLevel.init( 99 );
    _tileSize.init( 256 );
    _noDataValue.init( -32767.0f ); // SHRT_MIN
    _minValidValue.init( -32766.0f ); // -(2^15 - 2)
    _maxValidValue.init( 32767.0f );

    conf.get( "min_level", _minLevel );
    conf.get( "max_level", _maxLevel );
    conf.get( "min_resolution", _minResolution );
    conf.get( "max_resolution", _maxResolution );
    conf.get( "max_data_level", _maxDataLevel );
    conf.get( "no_data_value", _noDataValue);
    conf.get( "nodata_value", _noDataValue); // back compat
    conf.get( "min_valid_value", _minValidValue);
    conf.get( "max_valid_value", _maxValidValue);
    conf.get( "tile_size", _tileSize);
    conf.get( "profile", _profile);
}

//------------------------------------------------------------------------

TileLayer::CacheBinMetadata::CacheBinMetadata() :
_valid(false)
{
    //nop
}

TileLayer::CacheBinMetadata::CacheBinMetadata(const TileLayer::CacheBinMetadata& rhs) :
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

TileLayer::CacheBinMetadata::CacheBinMetadata(const Config& conf)
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
TileLayer::CacheBinMetadata::getConfig() const
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

OE_LAYER_PROPERTY_IMPL(TileLayer, unsigned, MinLevel, minLevel);
OE_LAYER_PROPERTY_IMPL(TileLayer, double, MinResolution, minResolution);
OE_LAYER_PROPERTY_IMPL(TileLayer, unsigned, MaxLevel, maxLevel);
OE_LAYER_PROPERTY_IMPL(TileLayer, double, MaxResolution, maxResolution);
OE_LAYER_PROPERTY_IMPL(TileLayer, unsigned, MaxDataLevel, maxDataLevel);


TileLayer::~TileLayer()
{
    //nop
}

void
TileLayer::init()
{
    Layer::init();

    _openCalled = false;
    _writingRequested = false;
    _profileMatchesMapProfile = true;

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
TileLayer::addedToMap(const Map* map)
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
TileLayer::removedFromMap(const Map* map)
{
    VisibleLayer::removedFromMap(map);
    _profileMatchesMapProfile.unset();
}

void
TileLayer::setUpL2Cache(unsigned minSize)
{
    // Check the layer hints
    unsigned l2CacheSize = layerHints().L2CacheSize().getOrUse(minSize);

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
TileLayer::openImplementation()
{
    if ( !_openCalled )
    {
        _openCalled = true;

        Status parent = VisibleLayer::openImplementation();
        if (parent.isError())
            return parent;

        _cacheBinMetadata.clear();
    }

    return getStatus();
}


const Status&
TileLayer::openForWriting()
{
    if (isWritingSupported())
    {
        _writingRequested = true;
        return open();
    }
    return setStatus(Status::ServiceUnavailable, "Layer does not support writing");
}

Status
TileLayer::closeImplementation()
{
    setProfile(0L);
    _openCalled = false;
    setStatus(Status());
    return Layer::closeImplementation();
}

void
TileLayer::establishCacheSettings()
{
    //nop
}

const Profile*
TileLayer::getProfile() const
{
    return _profile.get();
}

void
TileLayer::setProfile(const Profile* profile)
{
    _profile = profile;

    if (getProfile())
    {
        // augment the final profile with any overrides:
        applyProfileOverrides();

        OE_INFO << LC
            << (getProfile()? getProfile()->toString() : "[no profile]") << " "
            << (getCacheSettings()? getCacheSettings()->toString() : "[no cache settings]")
            << std::endl;
    }
}

bool
TileLayer::isDynamic() const
{
    if (getHints().dynamic().isSetTo(true))
        return true;
    else
        return false;
}

std::string
TileLayer::getMetadataKey(const Profile* profile) const
{
    if (profile)
        return Stringify() << profile->getHorizSignature() << "_metadata";
    else
        return "_metadata";
}

CacheBin*
TileLayer::getCacheBin(const Profile* profile)
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
        // read the metadata record from the cache bin:
        ReadResult rr = bin->readString(metaKey, getReadOptions());

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
                
                if (cacheSettings->cachePolicy()->isCacheOnly() && !_profile.valid())
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

                // store it in the cache bin.
                std::string data = meta->getConfig().toJSON(false);
                osg::ref_ptr<StringObject> temp = new StringObject(data);
                bin->write(metaKey, temp.get(), getReadOptions());

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
TileLayer::disable(const std::string& msg)
{
    setStatus(Status::Error(msg));
}

TileLayer::CacheBinMetadata*
TileLayer::getCacheBinMetadata(const Profile* profile)
{
    if (!profile)
        return 0L;

    Threading::ScopedMutexLock lock(_mutex);

    CacheBinMetadataMap::iterator i = _cacheBinMetadata.find(getMetadataKey(profile));
    return i != _cacheBinMetadata.end() ? i->second.get() : 0L;
}

bool
TileLayer::isKeyInLegalRange(const TileKey& key) const
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
TileLayer::isKeyInVisualRange(const TileKey& key) const
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
TileLayer::isCached(const TileKey& key) const
{
    // first consult the policy:
    if (getCacheSettings()->isCacheDisabled())
        return false;

    else if (getCacheSettings()->cachePolicy()->isCacheOnly())
        return true;

    // next check for a bin:
    CacheBin* bin = const_cast<TileLayer*>(this)->getCacheBin( key.getProfile() );
    if ( !bin )
        return false;

    return bin->getRecordStatus( key.str() ) == CacheBin::STATUS_OK;
}

const DataExtentList&
TileLayer::getDataExtents() const
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
TileLayer::dataExtents()
{
    return const_cast<DataExtentList&>(getDataExtents());
}

void
TileLayer::dirtyDataExtents()
{
    Threading::ScopedMutexLock lock(_mutex);
    _dataExtentsUnion = GeoExtent::INVALID;
}

const GeoExtent&
TileLayer::getDataExtentsUnion() const
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
TileLayer::getExtent() const
{
    return getDataExtentsUnion();
}

TileKey
TileLayer::getBestAvailableTileKey(const TileKey& key) const
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

    // Reject if the extents don't overlap at all.
    if (!getDataExtentsUnion().intersects(key.getExtent()))
    {
        return TileKey::INVALID;
    }

    bool     intersects = false;
    unsigned highestLOD = 0;

    // Check each data extent in turn:
    for (DataExtentList::const_iterator itr = de.begin(); itr != de.end(); ++itr)
    {
        // check for 2D intersection:
        if (key.getExtent().intersects(*itr))
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
TileLayer::mayHaveData(const TileKey& key) const
{
    return key == getBestAvailableTileKey(key);
}

void
TileLayer::setTileSize(const unsigned& value)
{
    options().tileSize() = value;
}

unsigned
TileLayer::getTileSize() const
{
    return options().tileSize().get();
}

void
TileLayer::setNoDataValue(const float& value)
{
    options().noDataValue() = value;
}

float
TileLayer::getNoDataValue() const
{
    return options().noDataValue().get();
}

void
TileLayer::setMinValidValue(const float& value)
{
    options().minValidValue() = value;
}

float
TileLayer::getMinValidValue() const
{
    return options().minValidValue().get();
}

void
TileLayer::setMaxValidValue(const float& value)
{
    options().maxValidValue() = value;
}

float
TileLayer::getMaxValidValue() const
{
    return options().maxValidValue().get();
}
