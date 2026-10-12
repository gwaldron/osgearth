/* osgEarth
 * Copyright 2008-2026 Pelican Mapping
 * MIT License
 */
#include <osgEarth/ElevationPool>
#include <osgEarth/Map>
#include <osgEarth/Metrics>
#include <osgEarth/rtree.h>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Progress>
#include <condition_variable>
#include <chrono>
#include <thread>
#include <atomic>
#include <list>

using namespace osgEarth;

namespace
{
    using MaxLevelIndex = RTree<unsigned, double, 2>;
    using RasterKey = Internal::RevElevationKey;
    constexpr std::size_t quickCacheSize = 64u;
    std::atomic<std::size_t> nextGeneration{1u};
    thread_local std::vector<RasterKey> creatingKeys;

    struct CreationScope
    {
        //! Records nested creation to detect cycles, including builds that bypass another thread's flight.
        explicit CreationScope(const RasterKey& key) { creatingKeys.push_back(key); }

        //! Restores the thread's creation stack even if a layer callback throws.
        ~CreationScope() { creatingKeys.pop_back(); }
    };

    //! Limits tile dimensions to representable unsigned counts, avoiding invalid shifts.
    unsigned maximumLOD(const Profile* profile)
    {
        unsigned wide, high;
        profile->getNumTiles(0u, wide, high);
        unsigned lod = 0u;
        while (lod < 30u && wide <= UINT_MAX / 2u && high <= UINT_MAX / 2u)
        {
            wide *= 2u;
            high *= 2u;
            ++lod;
        }
        return lod;
    }

    //! Converts resolution using geographic latitude, even for projected map coordinates.
    int resolutionLOD(const Profile* profile, const Distance& resolution, double x, double y, unsigned maxLOD)
    {
        const auto* srs = profile->getSRS();
        double latitude = y;
        const bool latitudeDependent = resolution.getUnits().isAngular() != srs->getUnits().isAngular();
        if (latitudeDependent && !srs->isGeographic())
        {
            GeoPoint geographic;
            if (!GeoPoint(srs, x, y).transform(srs->getGeographicSRS(), geographic))
                return -1;
            latitude = geographic.y();
        }
        double units = srs->transformDistance(resolution, srs->getUnits(), latitude);
        if (!std::isfinite(units) || units < 0.0)
            return -1;
        return int(std::min(maxLOD,
            profile->getLevelOfDetailForHorizResolution(units, ELEVATION_TILE_SIZE)));
    }

    //! Leaves ordinary XYZ points eligible for sampling.
    bool skipPoint(const osg::Vec3d&) { return false; }

    //! Preserves the existing per-point resolution sentinel, including its unchanged Z value.
    bool skipPoint(const osg::Vec4d& point) { return point.w() == FLT_MAX; }

    //! Samples a batch with bounded tile retention; callbacks select LOD and acquire revisioned tiles.
    //! Coordinates must be in profile SRS. Returns -1 on cancellation, preserving completed samples.
    template<typename Iterator, typename SelectLOD, typename Acquire>
    int sampleBatch(Iterator begin, Iterator end, const Profile* profile, std::size_t hash,
        ElevationPool::Envelope::QuickCache& cache, SelectLOD selectLOD, Acquire acquire,
        ProgressCallback* progress, float failValue)
    {
        if (begin == end || !profile)
            return -1;
        const auto& extent = profile->getExtent();
        const double inverseWidth = 1.0 / extent.width(), inverseHeight = 1.0 / extent.height();
        unsigned tw = 0u, th = 0u, previousX = UINT_MAX, previousY = UINT_MAX;
        int previousLOD = -1;
        osg::ref_ptr<ElevationTile> raster;
        double xmin = 0.0, ymin = 0.0, inverseTileWidth = 0.0, inverseTileHeight = 0.0;
        int count = 0;
        unsigned processed = 0u;
        for (auto iter = begin; iter != end; ++iter)
        {
            if ((processed++ & 63u) == 0u && progress && progress->isCanceled())
                return -1;
            auto& p = *iter;
            if (skipPoint(p))
                continue;
            const double rx = (p.x() - extent.xMin()) * inverseWidth;
            const double ry = (p.y() - extent.yMin()) * inverseHeight;
            if (!(rx >= 0.0 && rx <= 1.0 && ry >= 0.0 && ry <= 1.0))
            {
                p.z() = failValue;
                continue;
            }
            const int lod = selectLOD(p);
            if (lod < 0)
            {
                p.z() = failValue;
                continue;
            }
            if (lod != previousLOD)
                profile->getNumTiles(unsigned(lod), tw, th);
            const unsigned tx = std::min(unsigned(rx * double(tw)), tw - 1u);
            const unsigned ty = std::min(unsigned((1.0 - ry) * double(th)), th - 1u);
            if (lod != previousLOD || tx != previousX || ty != previousY)
            {
                RasterKey key{TileKey(unsigned(lod), tx, ty, profile), hash};
                auto found = cache.find(key);
                if (found != cache.end())
                    raster = found->second;
                else
                {
                    raster = acquire(key);
                    if (progress && progress->isCanceled())
                        return -1;
                    // Batches can span millions of points; never pin every visited tile.
                    if (cache.size() >= quickCacheSize)
                        cache.clear();
                    cache.emplace(key, raster);
                }
                previousLOD = lod;
                previousX = tx;
                previousY = ty;
                if (raster.valid())
                {
                    xmin = raster->getExtent().xMin();
                    ymin = raster->getExtent().yMin();
                    inverseTileWidth = 1.0 / raster->getExtent().width();
                    inverseTileHeight = 1.0 / raster->getExtent().height();
                }
            }
            if (raster.valid())
            {
                const double u = clamp((p.x() - xmin) * inverseTileWidth, 0.0, 1.0);
                const double v = clamp((p.y() - ymin) * inverseTileHeight, 0.0, 1.0);
                p.z() = raster->getRawElevationUV(u, v);
                if (p.z() != failValue)
                    ++count;
            }
            else
                p.z() = failValue;
        }
        return count;
    }
}

struct ElevationPool::MapData
{
    struct LayerIndex
    {
        int revision = 0;
        std::shared_ptr<MaxLevelIndex> tree;
    };
    osg::observer_ptr<const Map> map;
    ElevationLayerVector layers;
    osg::ref_ptr<const Profile> mapProfile, mapProfileNoVDatum;
    RasterInterpolation interpolation = INTERP_BILINEAR;
    std::vector<LayerIndex> indexes;
    std::size_t generation = 0u, hash = 0u;
    unsigned maxLOD = 0u;

    //! Captures map configuration without an owning Map reference, avoiding a Map/Pool cycle.
    explicit MapData(const Map* source) : map(source), generation(nextGeneration.fetch_add(1u))
    {
        if (source)
        {
            source->getOpenLayers<ElevationLayer>(layers);
            mapProfile = source->getProfile();
            mapProfileNoVDatum = source->getProfileNoVDatum();
            interpolation = source->getElevationInterpolation();
        }
        buildIndexes();
    }

    //! Rebuilds metadata for a revision change or custom selection, preserving map configuration identity.
    MapData(const MapData& source, const ElevationLayerVector& selection) :
        map(source.map), layers(selection), mapProfile(source.mapProfile),
        mapProfileNoVDatum(source.mapProfileNoVDatum), interpolation(source.interpolation),
        generation(source.generation)
    {
        buildIndexes();
    }

    //! Builds owned read-only indexes outside pool locks; unknown coverage uses the layer profile extent.
    void buildIndexes()
    {
        hash = hash_value_unsigned(generation, unsigned(interpolation));
        if (!mapProfile.valid())
            return;
        maxLOD = maximumLOD(mapProfile.get());
        indexes.reserve(layers.size());
        for (const auto& layer : layers)
        {
            LayerIndex index;
            index.revision = layer->getRevision();
            index.tree = std::make_shared<MaxLevelIndex>();
            hash = hash_value_unsigned(hash, layer->getUID(), index.revision);
            DataExtentList extents;
            layer->getDataExtents(extents);
            const Profile* profile = layer->getProfile();
            if (extents.empty() && profile)
                extents.emplace_back(profile->getExtent(), 0u, layer->getMaxDataLevel());
            for (const auto& dataExtent : extents)
            {
                GeoExtent extent = mapProfile->clampAndTransformExtent(dataExtent);
                if (extent.isInvalid() || !profile)
                    continue;
                unsigned level = std::min(layer->getMaxDataLevel(), layer->getMaxLevel());
                if (dataExtent.maxLevel().isSet())
                    level = std::min(level, dataExtent.maxLevel().get());
                level = std::min(maxLOD, mapProfile->getEquivalentLOD(profile, std::min(level, 30u)));
                // Insert each non-wrapping part so point searches work at the antimeridian.
                auto insert = [&](const GeoExtent& part)
                {
                    double minimum[2] = {part.xMin(), part.yMin()};
                    double maximum[2] = {part.xMax(), part.yMax()};
                    index.tree->Insert(minimum, maximum, level);
                };
                if (extent.crossesAntimeridian())
                {
                    GeoExtent left, right;
                    extent.splitAcrossAntimeridian(left, right);
                    insert(left);
                    insert(right);
                }
                else
                    insert(extent);
            }
            indexes.emplace_back(std::move(index));
        }
    }

    //! Checks atomic layer revisions without copying metadata or taking an exclusive pool lock.
    bool current() const
    {
        if (indexes.size() != layers.size())
            return layers.empty();
        for (std::size_t i = 0; i < layers.size(); ++i)
            if (indexes[i].revision != layers[i]->getRevision())
                return false;
        return true;
    }
};

struct ElevationPool::Cache
{
    struct Entry
    {
        RasterKey key;
        Pointer tile;
        std::size_t bytes;
    };
    struct WeakEntry
    {
        WeakPointer tile;
        std::list<RasterKey>::iterator position;
    };
    struct Flight
    {
        std::condition_variable changed;
        std::thread::id owner;
        Pointer tile;
        bool complete = false;
        bool retry = false;
    };
    mutable std::mutex mutex;
    std::list<Entry> retained;
    std::unordered_map<RasterKey, std::list<Entry>::iterator> strong;
    std::list<RasterKey> weakOrder;
    std::unordered_map<RasterKey, WeakEntry> weak;
    std::unordered_map<RasterKey, std::shared_ptr<Flight>> flights;
    std::size_t budget = 48u * 1024u * 1024u;
    std::size_t weakLimit = 4096u;
    CacheStats stats;

    //! Honors the existing environment switch for shared strong retention.
    Cache()
    {
        if (::getenv("OSGEARTH_NO_L2_CACHE"))
            budget = 0u;
    }

    //! Removes least-recently-used strong references until payload fits; caller holds mutex.
    void trim()
    {
        while (!retained.empty() && stats.retainedBytes > budget)
        {
            stats.retainedBytes -= retained.front().bytes;
            strong.erase(retained.front().key);
            retained.pop_front();
        }
    }

    //! Retains one canonical tile, charging aliases only once; caller holds mutex.
    void retain(const RasterKey& key, const Pointer& tile)
    {
        auto found = strong.find(key);
        if (found != strong.end())
        {
            retained.splice(retained.end(), retained, found->second);
            return;
        }
        std::size_t bytes = tile->getResolutions().size() * sizeof(float);
        if (tile->getHeightField())
            bytes += tile->getHeightField()->getHeightList().size() * sizeof(float);
        if (tile->getElevationTile() && tile->getElevationTile()->getImage())
            bytes += tile->getElevationTile()->getImage()->getTotalSizeInBytes();
        if (bytes > budget || budget == 0u)
            return;
        retained.push_back(Entry{key, tile, bytes});
        strong.emplace(key, std::prev(retained.end()));
        stats.retainedBytes += bytes;
        trim();
    }

    //! Performs a bounded expired-entry sweep and enforces the metadata cap; caller holds mutex.
    void sweep()
    {
        for (unsigned i = 0u; i < 4u && !weakOrder.empty(); ++i)
        {
            auto position = weakOrder.begin();
            auto found = weak.find(*position);
            Pointer tile;
            if (!found->second.tile.lock(tile))
            {
                weak.erase(found);
                weakOrder.erase(position);
            }
            else
                weakOrder.splice(weakOrder.end(), weakOrder, position);
        }
        while (weak.size() > weakLimit)
        {
            weak.erase(weakOrder.front());
            weakOrder.pop_front();
        }
    }

    //! Publishes a weak alias, retaining the actual tile key separately; caller holds mutex.
    void insert(const RasterKey& key, const Pointer& tile)
    {
        auto found = weak.find(key);
        if (found == weak.end())
        {
            weakOrder.push_back(key);
            weak.emplace(key, WeakEntry{WeakPointer(tile.get()), std::prev(weakOrder.end())});
        }
        else
            found->second.tile = tile.get();
        retain(RasterKey{tile->getTileKey(), key._hash}, tile);
        sweep();
    }

    //! Looks up strong references before promoting weak ones; caller holds mutex.
    Pointer find(const RasterKey& key)
    {
        auto cached = strong.find(key);
        if (cached != strong.end())
        {
            retained.splice(retained.end(), retained, cached->second);
            ++stats.hits;
            return cached->second->tile;
        }
        auto found = weak.find(key);
        Pointer tile;
        if (found != weak.end())
        {
            if (found->second.tile.lock(tile))
            {
                retain(RasterKey{tile->getTileKey(), key._hash}, tile);
                ++stats.hits;
                return tile;
            }
            weakOrder.erase(found->second.position);
            weak.erase(found);
        }
        ++stats.misses;
        return {};
    }
};

ElevationPool::ElevationPool() : _cache(new Cache()), _tileSize(ELEVATION_TILE_SIZE),
    _mapData(std::make_shared<MapData>(nullptr))
{
}

ElevationPool::~ElevationPool() = default;

const SpatialReference*
ElevationPool::getMapSRS() const
{
    auto snapshot = std::atomic_load(&_mapData);
    osg::ref_ptr<const Map> map;
    return snapshot->map.lock(map) ? map->getSRS() : nullptr;
}

void
ElevationPool::setMap(const Map* map)
{
    Snapshot snapshot = std::make_shared<MapData>(map);
    std::lock_guard<std::mutex> lock(_cache->mutex);
    std::atomic_store(&_mapData, snapshot);
    _cache->strong.clear();
    _cache->retained.clear();
    _cache->weak.clear();
    _cache->weakOrder.clear();
    _cache->stats.retainedBytes = 0u;
    // Existing flights finish against their owned snapshots and wake their current callers.
}

void
ElevationPool::setCacheBudget(std::size_t bytes)
{
    std::lock_guard<std::mutex> lock(_cache->mutex);
    _cache->budget = bytes;
    const std::size_t tileBytes = std::size_t(_tileSize) * _tileSize * 3u * sizeof(float);
    _cache->weakLimit = std::max(std::size_t(64u), std::min(std::size_t(16384u), bytes / tileBytes * 64u));
    _cache->trim();
    _cache->sweep();
}

ElevationPool::CacheStats
ElevationPool::getCacheStats() const
{
    std::lock_guard<std::mutex> lock(_cache->mutex);
    CacheStats stats = _cache->stats;
    stats.retainedTiles = _cache->retained.size();
    stats.weakEntries = _cache->weak.size();
    stats.inFlight = _cache->flights.size();
    return stats;
}

ElevationPool::WorkingSet::WorkingSet(unsigned size) : _lru(size) { }

void
ElevationPool::WorkingSet::setElevationLayers(const ElevationLayerVector& layers)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _elevationLayers = layers;
    _snapshot.reset();
    _lru.clear();
}

void
ElevationPool::WorkingSet::clear()
{
    std::lock_guard<std::mutex> lock(_mutex);
    _snapshot.reset();
    _lru.clear();
}

ElevationPool::Snapshot
ElevationPool::snapshotMapData(WorkingSet* ws)
{
    auto snapshot = std::atomic_load(&_mapData);
    while (!snapshot->current())
    {
        Snapshot updated = std::make_shared<MapData>(*snapshot, snapshot->layers);
        if (std::atomic_compare_exchange_strong(&_mapData, &snapshot, updated))
        {
            snapshot = updated;
            break;
        }
    }
    if (ws)
    {
        std::unique_lock<std::mutex> lock(ws->_mutex);
        if (!ws->_elevationLayers.empty())
        {
            if (ws->_snapshot && ws->_snapshot->generation == snapshot->generation && ws->_snapshot->current())
                return ws->_snapshot;
            ElevationLayerVector selection = ws->_elevationLayers;
            lock.unlock();
            Snapshot custom = std::make_shared<MapData>(*snapshot, selection);
            lock.lock();
            // Do not replace a newer selection installed while virtual extent queries were running.
            if (selection.size() == ws->_elevationLayers.size() &&
                std::equal(selection.begin(), selection.end(), ws->_elevationLayers.begin()))
                ws->_snapshot = custom;
            return custom;
        }
    }
    return snapshot;
}

int
ElevationPool::getLOD(const Snapshot& snapshot, double x, double y, int maxLOD) const
{
    if (maxLOD < 0 || !std::isfinite(x) || !std::isfinite(y))
        return -1;
    double point[2] = {x, y};
    int best = -1;
    for (const auto& index : snapshot->indexes)
    {
        index.tree->Search(point, point, [&](const unsigned& level)
        {
            best = std::max(best, int(level));
            return best < maxLOD;
        });
        if (best >= maxLOD)
            return maxLOD;
    }
    return best;
}

osg::ref_ptr<ElevationTile>
ElevationPool::createRaster(const Snapshot& snapshot, const RasterKey& key, ProgressCallback* progress)
{
    if (std::find(creatingKeys.begin(), creatingKeys.end(), key) != creatingKeys.end())
        return {};
    std::shared_ptr<Cache::Flight> flight;
    bool bypass = false;
    for (;;)
    {
        if (progress && progress->isCanceled())
            return {};
        std::unique_lock<std::mutex> lock(_cache->mutex);
        Pointer cached = _cache->find(key);
        if (cached.valid())
            return cached->getTileKey() == key._tilekey ? cached : Pointer();
        auto found = _cache->flights.find(key);
        if (found == _cache->flights.end())
        {
            flight = std::make_shared<Cache::Flight>();
            flight->owner = std::this_thread::get_id();
            _cache->flights.emplace(key, flight);
            ++_cache->stats.builds;
            break;
        }
        flight = found->second;
        if (flight->owner == std::this_thread::get_id())
            return {}; // A procedural layer must not wait on its own unfinished tile.
        if (!creatingKeys.empty())
        {
            // Nested layer queries can form cross-thread dependency cycles. Duplicate this
            // exceptional build instead of waiting while another build depends on this one.
            bypass = true;
            ++_cache->stats.builds;
            break;
        }
        ++_cache->stats.waits;
        while (!flight->complete)
        {
            flight->changed.wait_for(lock, std::chrono::milliseconds(10));
            lock.unlock();
            const bool canceled = progress && progress->isCanceled();
            lock.lock();
            if (canceled)
                return {};
        }
        if (!flight->retry)
            return flight->tile;
    }

    Pointer result;
    bool retry = false;
    CreationScope scope(key);
    try
    {
        // Match populateHeightField's native-data test before allocating a full raster.
        // This avoids allocating and clearing 66K samples at every unavailable ancestor.
        TileKey haeKey(key._tilekey);
        if (snapshot->mapProfileNoVDatum.valid())
            haeKey = TileKey(haeKey.getLOD(), haeKey.getTileX(), haeKey.getTileY(), snapshot->mapProfileNoVDatum);
        bool native = false;
        for (const auto& layer : snapshot->layers)
        {
            if (layer->isOpen() && key._tilekey.getLOD() >= layer->getMinLevel())
            {
                const TileKey mapped = haeKey.mapResolution(_tileSize, layer->getTileSize());
                if (mapped == layer->getBestAvailableTileKey(mapped))
                {
                    native = true;
                    break;
                }
            }
        }
        if (native)
        {
            osg::ref_ptr<osg::HeightField> hf = HeightFieldUtils::createReferenceHeightField(
                key._tilekey.getExtent(), _tileSize, _tileSize, 0u, true);
            std::vector<float> resolutions(std::size_t(_tileSize) * _tileSize, FLT_MAX);
            const bool populated = snapshot->layers.populateHeightField(hf.get(), &resolutions,
                key._tilekey, snapshot->mapProfileNoVDatum.get(), snapshot->interpolation, progress);
            retry = progress && progress->isCanceled();
            if (populated && !retry)
            {
                HeightFieldUtils::resolveInvalidHeights(hf.get(), key._tilekey.getExtent(), NO_DATA_VALUE, nullptr);
                result = new ElevationTile(key._tilekey, GeoHeightField(hf.get(), key._tilekey.getExtent()),
                    std::move(resolutions));
            }
        }
        else
            retry = progress && progress->isCanceled();
    }
    catch (...)
    {
        if (!bypass)
        {
            std::lock_guard<std::mutex> lock(_cache->mutex);
            flight->retry = true;
            flight->complete = true;
            _cache->flights.erase(key);
            flight->changed.notify_all();
        }
        throw;
    }
    {
        std::lock_guard<std::mutex> lock(_cache->mutex);
        if (result.valid() && snapshot->generation == std::atomic_load(&_mapData)->generation)
            _cache->insert(key, result);
        if (!bypass)
        {
            flight->tile = result;
            flight->retry = retry;
            flight->complete = true;
            _cache->flights.erase(key);
            flight->changed.notify_all();
        }
    }
    return result;
}

osg::ref_ptr<ElevationTile>
ElevationPool::getOrCreateRaster(const Snapshot& snapshot, const RasterKey& key, bool acceptLowerRes,
    WorkingSet* ws, ProgressCallback* progress)
{
    OE_PROFILING_ZONE;
    if (!key._tilekey.valid() || !snapshot->mapProfile.valid() || snapshot->layers.empty() ||
        (progress && progress->isCanceled()))
        return {};
    if (ws)
    {
        auto cached = ws->_lru.get(key);
        if (cached.has_value() && cached.value().valid())
            return acceptLowerRes || cached.value()->getTileKey() == key._tilekey ? cached.value() : Pointer();
    }
    Pointer result;
    {
        std::lock_guard<std::mutex> lock(_cache->mutex);
        result = _cache->find(key);
    }
    if (result.valid() && !acceptLowerRes && result->getTileKey() != key._tilekey)
        return {};
    for (TileKey candidate = key._tilekey; !result.valid() && candidate.valid(); candidate.makeParent())
    {
        result = createRaster(snapshot, RasterKey{candidate, key._hash}, progress);
        if (!acceptLowerRes || (progress && progress->isCanceled()))
            break;
    }
    if (progress && progress->isCanceled())
        return {};
    if (result.valid())
    {
        if (result->getTileKey() != key._tilekey)
        {
            std::lock_guard<std::mutex> lock(_cache->mutex);
            if (snapshot->generation == std::atomic_load(&_mapData)->generation)
                _cache->insert(key, result);
        }
        if (ws)
            ws->_lru.insert(key, result);
    }
    return result;
}

ElevationPool::Envelope::Envelope() : _lod(-1), _ws(nullptr)
{
}

bool
ElevationPool::prepareEnvelope(Envelope& env, const GeoPoint& refPoint, const Distance& resolution, WorkingSet* ws)
{
    env._lod = -1;
    env._cache.clear();
    env._default_ws.clear();
    env._mapDataSnapshot = snapshotMapData(ws);
    env._pool = this;
    env._profile = env._mapDataSnapshot->mapProfile;
    env._ws = ws ? ws : &env._default_ws;
    if (!refPoint.isValid() || !env._profile.valid() || !env._mapDataSnapshot->map.lock(env._map))
        return false;
    GeoPoint point;
    if (!refPoint.transform(env._profile->getSRS(), point))
        return false;
    const int cap = resolutionLOD(env._profile.get(), resolution, point.x(), point.y(), env._mapDataSnapshot->maxLOD);
    env._lod = getLOD(env._mapDataSnapshot, point.x(), point.y(), cap);
    // A reference point can lie between disjoint data extents that the envelope will sample.
    if (env._lod < 0 && !env._mapDataSnapshot->layers.empty())
        env._lod = cap;
    return true;
}

int
ElevationPool::Envelope::sampleMapCoords(std::vector<osg::Vec3d>::iterator begin,
    std::vector<osg::Vec3d>::iterator end, ProgressCallback* progress, float failValue)
{
    if (!_mapDataSnapshot || !_pool.valid())
        return -1;
    return sampleBatch(begin, end, _profile.get(), _mapDataSnapshot->hash, _cache,
        [&](const osg::Vec3d&) { return _lod; },
        [&](const RasterKey& key) { return _pool->getOrCreateRaster(_mapDataSnapshot, key, true, _ws, progress); },
        progress, failValue);
}

int
ElevationPool::sampleMapCoords(std::vector<osg::Vec4d>::iterator begin, std::vector<osg::Vec4d>::iterator end,
    WorkingSet* ws, ProgressCallback* progress, float failValue)
{
    auto snapshot = snapshotMapData(ws);
    if (!snapshot->mapProfile.valid())
        return -1;
    const Profile* profile = snapshot->mapProfile.get();
    double previousResolution = -1.0;
    int cap = -1;
    Envelope::QuickCache cache;
    return sampleBatch(begin, end, profile, snapshot->hash, cache,
        [&](const osg::Vec4d& p)
        {
            if (!std::isfinite(p.w()) || p.w() < 0.0)
                return -1;
            if (p.w() != previousResolution)
            {
                cap = int(std::min(snapshot->maxLOD,
                    profile->getLevelOfDetailForHorizResolution(p.w(), ELEVATION_TILE_SIZE)));
                previousResolution = p.w();
            }
            return getLOD(snapshot, p.x(), p.y(), cap);
        },
        [&](const RasterKey& key) { return getOrCreateRaster(snapshot, key, true, ws, progress); },
        progress, failValue);
}

int
ElevationPool::sampleMapCoords(std::vector<osg::Vec3d>::iterator begin, std::vector<osg::Vec3d>::iterator end,
    const Distance& resolution, WorkingSet* ws, ProgressCallback* progress, float failValue)
{
    auto snapshot = snapshotMapData(ws);
    if (!snapshot->mapProfile.valid())
        return -1;
    const Profile* profile = snapshot->mapProfile.get();
    const bool variable = resolution.getUnits().isAngular() != profile->getSRS()->getUnits().isAngular();
    const int fixedCap = variable ? -1 : resolutionLOD(profile, resolution, 0.0, 0.0, snapshot->maxLOD);
    const bool geographic = profile->getSRS()->isGeographic();
    double previousX = std::numeric_limits<double>::quiet_NaN(), previousY = previousX;
    int variableCap = -1;
    Envelope::QuickCache cache;
    return sampleBatch(begin, end, profile, snapshot->hash, cache,
        [&](const osg::Vec3d& p)
        {
            // Geographic row samples share a latitude conversion. Projected coordinates require
            // both X and Y to match since geographic latitude may depend on either coordinate.
            if (variable && (p.y() != previousY || (!geographic && p.x() != previousX)))
            {
                variableCap = resolutionLOD(profile, resolution, p.x(), p.y(), snapshot->maxLOD);
                previousX = p.x();
                previousY = p.y();
            }
            int cap = variable ? variableCap : fixedCap;
            return getLOD(snapshot, p.x(), p.y(), cap);
        },
        [&](const RasterKey& key) { return getOrCreateRaster(snapshot, key, true, ws, progress); },
        progress, failValue);
}

ElevationSample
ElevationPool::getSample(const Snapshot& snapshot, const GeoPoint& point, unsigned maxLOD,
    WorkingSet* ws, ProgressCallback* progress)
{
    if (!point.isValid() || !snapshot->mapProfile.valid() || (progress && progress->isCanceled()))
        return {};
    const int lod = getLOD(snapshot, point.x(), point.y(), int(std::min(maxLOD, snapshot->maxLOD)));
    if (lod < 0)
        return {};
    RasterKey key{snapshot->mapProfile->createTileKey(point.x(), point.y(), unsigned(lod)), snapshot->hash};
    auto raster = getOrCreateRaster(snapshot, key, true, ws, progress);
    return raster.valid() ? raster->getElevation(point.x(), point.y()) : ElevationSample();
}

ElevationSample
ElevationPool::getSample(const GeoPoint& point, WorkingSet* ws, ProgressCallback* progress)
{
    auto snapshot = snapshotMapData(ws);
    if (!point.isValid() || !snapshot->mapProfile.valid())
        return {};
    if (point.getSRS()->isHorizEquivalentTo(snapshot->mapProfile->getSRS()))
        return getSample(snapshot, point, snapshot->maxLOD, ws, progress);
    GeoPoint transformed;
    if (!point.transform(snapshot->mapProfile->getSRS(), transformed))
        return {};
    return getSample(snapshot, transformed, snapshot->maxLOD, ws, progress);
}

ElevationSample
ElevationPool::getSample(const GeoPoint& point, const Distance& resolution, WorkingSet* ws,
    ProgressCallback* progress)
{
    auto snapshot = snapshotMapData(ws);
    if (!point.isValid() || !snapshot->mapProfile.valid())
        return {};
    const GeoPoint* mapPoint = &point;
    GeoPoint transformed;
    if (!point.getSRS()->isHorizEquivalentTo(snapshot->mapProfile->getSRS()))
    {
        if (!point.transform(snapshot->mapProfile->getSRS(), transformed))
            return {};
        mapPoint = &transformed;
    }
    const int cap = resolutionLOD(snapshot->mapProfile.get(), resolution, mapPoint->x(), mapPoint->y(), snapshot->maxLOD);
    return cap >= 0 ? getSample(snapshot, *mapPoint, unsigned(cap), ws, progress) : ElevationSample();
}

bool
ElevationPool::getTile(const TileKey& key, bool acceptLowerRes, osg::ref_ptr<ElevationTexture>& output,
    WorkingSet* ws, ProgressCallback* progress)
{
    auto snapshot = snapshotMapData(ws);
    output = getOrCreateRaster(snapshot, RasterKey{key, snapshot->hash}, acceptLowerRes, ws, progress);
    return output.valid();
}

AsyncElevationSampler::AsyncElevationSampler(const Map* map, unsigned numThreads) :
    _map(map), _ws(std::make_shared<ElevationPool::WorkingSet>()), _arena(jobs::get_pool("oe.asyncelevation"))
{
    _arena->set_can_steal_work(false);
    _arena->set_concurrency(numThreads > 0u ? numThreads : _arena->concurrency());
}

Future<ElevationSample>
AsyncElevationSampler::getSample(const GeoPoint& point)
{
    return getSample(point, Distance(0.0, point.isValid() ? point.getXYUnits() : Units::METERS));
}

Future<ElevationSample>
AsyncElevationSampler::getSample(const GeoPoint& point, const Distance& resolution)
{
    jobs::context context;
    context.pool = _arena;
    auto mapObserver = _map;
    auto workingSet = _ws;
    auto task = [mapObserver, workingSet, point, resolution](Cancelable& cancelable)
    {
        osg::ref_ptr<const Map> map;
        if (cancelable.canceled() || !mapObserver.lock(map))
            return ElevationSample();
        osg::ref_ptr<ProgressCallback> progress = new ProgressCallback(&cancelable);
        return map->getElevationPool()->getSample(point, resolution, workingSet.get(), progress.get());
    };
    return jobs::dispatch(task, context);
}
