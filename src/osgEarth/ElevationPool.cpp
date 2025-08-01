
/* osgEarth
 * Copyright 2008-2016 Pelican Mapping
 * MIT License
 */
#include <osgEarth/ElevationPool>
#include <osgEarth/Map>
#include <osgEarth/Metrics>
#include <osgEarth/rtree.h>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Containers>
#include <osgEarth/Progress>
#include <osgEarth/Notify>

#include <thread>

using namespace osgEarth;

#define LC "[ElevationPool] "

ElevationPool::ElevationPool() :
    _tileSize(257),
    _L2(64u),
    _mapRevision(-1),
    _elevationHash(0)
{
}

const SpatialReference*
ElevationPool::getMapSRS() const
{
    osg::ref_ptr<const Map> map;
    if (_map.lock(map))
        return map->getSRS();
    else
        return nullptr;
}

namespace
{
#if 1
    using MaxLevelIndex = RTree<unsigned, double, 2>;
#else
    struct MaxLevelIndex
    {
        std::vector<osg::BoundingBox> _boxes;

        void Insert(const double a_min[2], const double a_max[2], unsigned maxLevel)
        {
            osg::BoundingBox box(a_min[0], a_min[1], 0.0, a_max[0], a_max[1], 0.0);
            _boxes.push_back(box);
        }

        template<class FUNC>
        bool Search(const double a_min[2], const double a_max[2], FUNC&& callback)
        {
            osg::BoundingBox query(a_min[0], a_min[1], 0.0, a_max[0], a_max[1], 0.0);
            for (unsigned i = 0; i < _boxes.size(); ++i)
            {
                if (query.intersects(_boxes[i]))
                {
                    if (callback(i) == false)
                        return false;
                }
            }
            return true;
        }
    };
#endif
}

ElevationPool::~ElevationPool()
{
    setMap(nullptr);

    for (auto& itr : _layerIndex)
    {
        if (itr.second)
        {
            delete static_cast<MaxLevelIndex*>(itr.second);
        }
    }
    _layerIndex.clear();
}

void
ElevationPool::setMap(const Map* map)
{
    _map = map;

    if (map)
    {
        refresh(map);
    }
}

size_t
ElevationPool::getElevationHash(WorkingSet* ws) const
{
    // yes, must do this every time because individual
    // layers can "bump" their revisions (dynamic layers)    
    size_t hash = hash_value_unsigned(_mapRevision);

    // using the working set or the baseline?
    auto& layers =
        (ws && ws->_elevationLayers.size() > 0) ? ws->_elevationLayers :
        this->_elevationLayers;

    for (auto& layer : layers)
        if (layer->isOpen())
            hash = hash_value_unsigned(hash, layer->getRevision());
        else
            hash = hash_value_unsigned(hash, 0u);

    return hash;
}

void
ElevationPool::sync(const Map* map, WorkingSet* ws)
{
    if (needsRefresh())
    {
        OE_PROFILING_ZONE;

        refresh(map);

        if (ws)
            ws->_lru.clear();
    }
}

void
ElevationPool::refresh(const Map* map)
{
    ScopedWriteLock lk(_mutex);

    _elevationLayers.clear();

    for (auto& itr : _layerIndex)
    {
        if (itr.second)
        {
            delete static_cast<MaxLevelIndex*>(itr.second);
        }
    }
    _layerIndex.clear();

    _mapRevision = _map->getOpenLayers(_elevationLayers);
    _elevationHash = getElevationHash(nullptr);

    double a_min[2], a_max[2];

    for (auto i : _elevationLayers)
    {
        const ElevationLayer* layer = i.get();
        DataExtentList dataExtents;
        layer->getDataExtents(dataExtents);

        MaxLevelIndex* layerIndex = new MaxLevelIndex();

        for (auto de = dataExtents.begin(); de != dataExtents.end(); ++de)
        {
            GeoExtent extentInMapSRS = map->getProfile()->clampAndTransformExtent(*de);

            // Convert the max level so it's relative to the map profile:
            unsigned maxLevel = std::min(de->maxLevel().get(), layer->getMaxDataLevel());
            maxLevel = map->getProfile()->getEquivalentLOD(layer->getProfile(), maxLevel);

            if (extentInMapSRS.crossesAntimeridian())
            {
                GeoExtent a, b;
                extentInMapSRS.splitAcrossAntimeridian(a, b);

                for (auto& ex : { a, b })
                {
                    a_min[0] = ex.xMin(), a_min[1] = ex.yMin();
                    a_max[0] = ex.xMax(), a_max[1] = ex.yMax();
                    layerIndex->Insert(a_min, a_max, maxLevel);
                }
            }
            else
            {
                a_min[0] = extentInMapSRS.xMin(), a_min[1] = extentInMapSRS.yMin();
                a_max[0] = extentInMapSRS.xMax(), a_max[1] = extentInMapSRS.yMax();
                layerIndex->Insert(a_min, a_max, maxLevel);
            }
        }
        _layerIndex[layer] = layerIndex;
    }

    _L2.clear();

    ScopedWriteLock lock(_globalLUTMutex);
    _globalLUT.clear();
}

int
ElevationPool::getLOD(double x, double y, WorkingSet* ws)
{
    double point[2] = { x, y };
    int maxiestMaxLevel = -1;

    auto& layers =
        (ws && ws->_elevationLayers.size() > 0) ? ws->_elevationLayers :
        this->_elevationLayers;

    for (auto& layerItr : layers)
    {
        auto itr = _layerIndex.find(layerItr.get());
        if (itr != _layerIndex.end())
        {
            MaxLevelIndex* index = static_cast<MaxLevelIndex*>(itr->second);
            index->Search(point, point, [&](const unsigned& level)
                {
                    maxiestMaxLevel = std::max(maxiestMaxLevel, (int)level);
                    return RTREE_KEEP_SEARCHING;
                });
        }
    }

    return maxiestMaxLevel;
}

bool
ElevationPool::needsRefresh()
{
    ScopedReadLock lk(_mutex);

    // Check to see if the overall data model has changed in the map
    int mapRevision = _map.valid() ? static_cast<int>(_map->getDataModelRevision()) : 0;
    if (mapRevision != _mapRevision)
    {
        return true;
    }

    // Check to see if any of the elevation layers in our list have changed.
    return getElevationHash(nullptr) != _elevationHash;
}

ElevationPool::WorkingSet::WorkingSet(unsigned size) :
    _lru(size)
{
    //nop
}

void
ElevationPool::WorkingSet::clear()
{
    _lru.clear();
    // No need to clear the elevation layers; only invalidate the cache.
}

bool
ElevationPool::findExistingRaster(
    const Internal::RevElevationKey& key,
    osg::ref_ptr<ElevationTile>& output,
    bool* fromLUT)
{
    OE_PROFILING_ZONE;

    *fromLUT = false;

    // Next check the system LUT -- see if someone somewhere else
    // already has it (the terrain or another WorkingSet)
    optional<Internal::RevElevationKey> orphanedKey;
    {
        ScopedReadLock lock(_globalLUTMutex);

        auto i = _globalLUT.find(key);
        if (i != _globalLUT.end())
        {
            i->second.lock(output);
            if (output.valid())
            {
                *fromLUT = true;
            }
            else
            {
                // observer was orphaned..remove it
                orphanedKey = key;
            }
        }
    }

    if (orphanedKey.isSet())
    {
        ScopedWriteLock lock(_globalLUTMutex);
        _globalLUT.erase(orphanedKey.get());
    }

    return output.valid();
}

osg::ref_ptr<ElevationTile>
ElevationPool::getOrCreateRaster(
    const Internal::RevElevationKey& key,
    const Map* map,
    bool acceptLowerRes,
    WorkingSet* ws,
    ProgressCallback* progress)
{
    OE_PROFILING_ZONE;

    // first check for pre-existing data for this key:
    osg::ref_ptr<ElevationTile> result;
    bool fromLUT;

    findExistingRaster(key, result, &fromLUT);

    if (!result.valid())
    {
        // need to build NEW data for this key
        osg::ref_ptr<osg::HeightField> hf = HeightFieldUtils::createReferenceHeightField(
            key._tilekey.getExtent(),
            _tileSize, _tileSize,
            false,      // no border
            true);      // initialize to HAE (0.0) heights

        std::vector<float> resolutions;
        resolutions.assign(_tileSize * _tileSize, FLT_MAX);

        TileKey keyToUse;
        bool populated = false;

        const ElevationLayerVector& layersToSample =
            ws && !ws->_elevationLayers.empty() ? ws->_elevationLayers :
            _elevationLayers;

        for (keyToUse = key._tilekey;
            keyToUse.valid();
            keyToUse.makeParent())
        {
            populated = layersToSample.populateHeightField(
                hf.get(),
                &resolutions,
                keyToUse,
                map->getProfileNoVDatum(),
                map->getElevationInterpolation(),
                progress);

            // Resolve any invalid heights in the output heightfield.
            HeightFieldUtils::resolveInvalidHeights(hf.get(), keyToUse.getExtent(), NO_DATA_VALUE, 0L);

            if ((populated == true) ||
                (acceptLowerRes == false) ||
                (progress && progress->isCanceled()))
            {
                break;
            }
        }

        // check for cancelation/deferral
        if (progress && progress->isCanceled())
        {
            return NULL;
        }

        if (populated)
        {
            result = new ElevationTile(
                keyToUse,
                GeoHeightField(hf.get(), keyToUse.getExtent()),
                std::move(resolutions));
        }
        else
        {
            return NULL;
        }
    }

    else
    {
        // found it ... but if it's a lower res tile and we aren't accepting
        // those, discard it.
        if (acceptLowerRes == false &&
            result->getTileKey() != key._tilekey)
        {
            return NULL;
        }
    }

    // update WorkingSet:
    if (ws)
        ws->_lru.insert(key, result);

    // update the L2 cache:
    _L2.insert(key, result);

    // update system weak-LUT:
    if (!fromLUT)
    {
        ScopedWriteLock lock(_globalLUTMutex);
        _globalLUT[key] = result.get();
    }

    return result;
}


bool
ElevationPool::prepareEnvelope(
    ElevationPool::Envelope& env,
    const GeoPoint& refPoint,
    const Distance& resolution,
    WorkingSet* ws)
{
    env._pool = this;
    env._map = nullptr;
    env._profile = nullptr;

    if (_map.lock(env._map) == false || env._map->getProfile() == nullptr)
        return false;

    env._profile = env._map->getProfile();

    sync(env._map.get(), ws);

    env._key._revision = getElevationHash(ws);

    env._raster = nullptr;
    env._cache.clear();

    env._pw = env._profile->getExtent().width();
    env._ph = env._profile->getExtent().height();
    env._pxmin = env._profile->getExtent().xMin();
    env._pymin = env._profile->getExtent().yMin();

    auto& units = env._map->getSRS()->getUnits();
    Distance pointRes(0.0, units);

    GeoPoint refPointMap = refPoint.transform(env._map->getSRS());

    double resolutionInMapUnits = env._map->getSRS()->transformDistance(resolution, units, refPointMap.y());

    int maxLOD = env._profile->getLevelOfDetailForHorizResolution(
        resolutionInMapUnits,
        ELEVATION_TILE_SIZE);

    env._lod = std::min(getLOD(refPointMap.x(), refPointMap.y(), ws), (int)maxLOD);

    // This can happen if the elevation data publishes no data extents
    if (env._lod < 0 && !_elevationLayers.empty())
    {
        env._lod = maxLOD;
    }

    env._profile->getNumTiles(env._lod, env._tw, env._th);

    env._ws = ws;

    if (env._ws == nullptr)
        env._ws = &env._default_ws;

    return true;
}

int
ElevationPool::Envelope::sampleMapCoords(
    std::vector<osg::Vec3d>::iterator begin,
    std::vector<osg::Vec3d>::iterator end,
    ProgressCallback* progress,
    float failValue)
{
    OE_PROFILING_ZONE;

    if (begin == end)
        return -1;

    if (_lod < 0)
    {
        for(auto i = begin; i != end; ++i)
            i->z() = failValue;
        return 0;
    }

    ScopedReadLock lk(_pool->_mutex);

    double u, v;
    double rx, ry;
    int tx, ty;
    int tx_prev = INT_MAX, ty_prev = INT_MAX;
    float lastRes = -1.0f;
    int lod = _lod;
    int lod_prev = INT_MAX;
    osg::Vec4f elev;
    int count = 0;

    for (auto iter = begin; iter != end; ++iter)
    {
        auto& p = *iter;

        bool tileKeyChanged = false;

        rx = (p.x() - _pxmin) / _pw, ry = (p.y() - _pymin) / _ph;
        tx = osg::clampBelow((unsigned)(rx * (double)_tw), _tw - 1u); // TODO: wrap around for geo
        ty = osg::clampBelow((unsigned)((1.0 - ry) * (double)_th), _th - 1u);

        if (lod != lod_prev || tx != tx_prev || ty != ty_prev)
        {
            _key._tilekey = TileKey(lod, tx, ty, _profile.get());
            lod_prev = lod;
            tx_prev = tx;
            ty_prev = ty;
            tileKeyChanged = true;
        }

        if (_key._tilekey.valid())
        {
            if (tileKeyChanged || !_raster.valid())
            {
                auto iter = _cache.find(_key);

                if (iter == _cache.end())
                {
                    _raster = _pool->getOrCreateRaster(
                        _key,   // key to query
                        _map.get(), // map to query
                        true,  // fall back on lower resolution data if necessary
                        _ws,    // user's workingset
                        progress);

                    // bail on cancelation before using the quickcache
                    if (progress && progress->isCanceled())
                    {
                        return -1;
                    }

                    _cache[_key] = _raster.get();
                }
                else
                {
                    _raster = iter->second;
                }
            }

            if (_raster.valid())
            {
                u = (p.x() - _raster->getExtent().xMin()) / _raster->getExtent().width();
                v = (p.y() - _raster->getExtent().yMin()) / _raster->getExtent().height();

                // Note: This can happen on the map edges..
                // TODO: consider looping around for geo and clamping for projected
                u = clamp(u, 0.0, 1.0);
                v = clamp(v, 0.0, 1.0);

                p.z() = _raster->getRawElevationUV(u, v);
            }
            else
            {
                p.z() = failValue;
            }
        }
        else
        {
            p.z() = failValue;
        }

        if (p.z() != failValue)
            ++count;
    }

    return count;
}

int
ElevationPool::sampleMapCoords(
    std::vector<osg::Vec4d>::iterator begin,
    std::vector<osg::Vec4d>::iterator end,
    WorkingSet* ws,
    ProgressCallback* progress,
    float failValue)
{
    OE_PROFILING_ZONE;

    if (begin == end)
        return -1;

    osg::ref_ptr<const Map> map;
    if (_map.lock(map) == false || map->getProfile() == NULL)
        return -1;

    sync(map.get(), ws);

    if (_elevationLayers.empty())
    {
        for (auto i = begin; i != end; ++i)
            i->z() = failValue;
        return 0;
    }

    ScopedReadLock lk(_mutex);

    Internal::RevElevationKey key;
    key._revision = getElevationHash(ws);

    osg::ref_ptr<ElevationTile> raster;
    osg::Vec4 elev;
    double u, v;

    const Profile* profile = map->getProfile();
    double pw = profile->getExtent().width();
    double ph = profile->getExtent().height();
    double pxmin = profile->getExtent().xMin();
    double pymin = profile->getExtent().yMin();

    int count = 0;

    Envelope::QuickCache quickCache;

    unsigned tw, th;
    double rx, ry;
    int tx, ty;
    int tx_prev = INT_MAX, ty_prev = INT_MAX;

    int lod;
    int lod_prev = INT_MAX;
    auto* srs = map->getSRS();
    auto& units = srs->getUnits();
    Distance pointRes(0.0, units);

    for (auto iter = begin; iter != end; ++iter)
    {
        auto& p = *iter;

        if (p.w() == FLT_MAX)
            continue;

        bool tileKeyChanged = false;

        pointRes.set(p.w(), units);

        double resolutionInMapUnits = srs->transformDistance(pointRes, units, p.y());

        lod = profile->getLevelOfDetailForHorizResolution(
            resolutionInMapUnits,
            ELEVATION_TILE_SIZE);

        profile->getNumTiles(lod, tw, th);

        rx = (p.x() - pxmin) / pw, ry = (p.y() - pymin) / ph;
        tx = osg::clampBelow((unsigned)(rx * (double)tw), tw - 1u); // TODO: wrap around for geo
        ty = osg::clampBelow((unsigned)((1.0 - ry) * (double)th), th - 1u);

        if (lod != lod_prev || tx != tx_prev || ty != ty_prev)
        {
            key._tilekey = TileKey(lod, tx, ty, profile);
            lod_prev = lod;
            tx_prev = tx;
            ty_prev = ty;
            tileKeyChanged = true;
        }

        if (key._tilekey.valid())
        {
            if (tileKeyChanged || !raster.valid())
            {
                auto iter = quickCache.find(key);

                if (iter == quickCache.end())
                {
                    raster = getOrCreateRaster(
                        key,   // key to query
                        map.get(), // map to query
                        true,  // fall back on lower resolution data if necessary
                        ws,    // user's workingset
                        progress);

                    // bail on cancelation before using the quickcache
                    if (progress && progress->isCanceled())
                    {
                        return -1;
                    }

                    quickCache[key] = raster.get();
                }
                else
                {
                    raster = iter->second;
                }
            }

            if (raster.valid())
            {
                u = (p.x() - raster->getExtent().xMin()) / raster->getExtent().width();
                v = (p.y() - raster->getExtent().yMin()) / raster->getExtent().height();

                // Note: This can happen on the map edges..
                // TODO: consider looping around for geo and clamping for projected
                u = clamp(u, 0.0, 1.0);
                v = clamp(v, 0.0, 1.0);

                p.z() = raster->getRawElevationUV(u, v);
            }
            else
            {
                p.z() = failValue;
            }
        }
        else
        {
            p.z() = failValue;
        }

        if (p.z() != failValue)
            ++count;
    }

    return count;
}

int
ElevationPool::sampleMapCoords(
    std::vector<osg::Vec3d>::iterator begin,
    std::vector<osg::Vec3d>::iterator end,
    const Distance& resolution,
    WorkingSet* ws,
    ProgressCallback* progress,
    float failValue)
{    
    OE_PROFILING_ZONE;

    if (begin == end)
        return -1;

    osg::ref_ptr<const Map> map;
    if (_map.lock(map) == false || map->getProfile() == NULL)
        return -1;

    sync(map.get(), ws);

    if (_elevationLayers.empty())
    {
        for (auto i = begin; i != end; ++i)
            i->z() = failValue;
        return 0;
    }

    ScopedReadLock lk(_mutex);

    Internal::RevElevationKey key;
    key._revision = getElevationHash(ws);

    osg::ref_ptr<ElevationTile> raster;
    double u, v;

    const Profile* profile = map->getProfile();
    double pw = profile->getExtent().width();
    double ph = profile->getExtent().height();
    double pxmin = profile->getExtent().xMin();
    double pymin = profile->getExtent().yMin();

    int count = 0;

    Envelope::QuickCache quickCache;

    unsigned tw, th;
    double rx, ry;
    int tx, ty;
    int tx_prev = INT_MAX, ty_prev = INT_MAX;

    int lod;
    int lod_prev = INT_MAX;
    auto* srs = map->getSRS();
    auto& units = srs->getUnits();

    for (auto iter = begin; iter != end; ++iter)
    {
        auto& p = *iter;

        bool tileKeyChanged = false;

        double resolutionInMapUnits = srs->transformDistance(resolution, units, p.y());

        int computedLOD = profile->getLevelOfDetailForHorizResolution(
            resolutionInMapUnits,
            ELEVATION_TILE_SIZE);

        lod = std::min(getLOD(p.x(), p.y(), ws), (int)computedLOD);

        if (lod < 0)
        {
            p.z() = failValue;
            continue;
        }

        profile->getNumTiles(lod, tw, th);
        rx = (p.x() - pxmin) / pw, ry = (p.y() - pymin) / ph;
        tx = osg::clampBelow((unsigned)(rx * (double)tw), tw - 1u);
        ty = osg::clampBelow((unsigned)((1.0 - ry) * (double)th), th - 1u);

        if (lod != lod_prev || tx != tx_prev || ty != ty_prev)
        {
            key._tilekey = TileKey(lod, tx, ty, profile);
            lod_prev = lod;
            tx_prev = tx;
            ty_prev = ty;
            tileKeyChanged = true;
        }

        if (key._tilekey.valid())
        {
            if (tileKeyChanged || !raster.valid())
            {
                auto iter = quickCache.find(key);

                if (iter == quickCache.end())
                {
                    raster = getOrCreateRaster(
                        key,   // key to query
                        map.get(), // map to query
                        true,  // fall back on lower resolution data if necessary
                        ws,    // user's workingset
                        progress);

                    // bail on cancelation before using the quickcache
                    if (progress && progress->isCanceled())
                    {
                        return -1;
                    }

                    quickCache[key] = raster.get();
                }
                else
                {
                    raster = iter->second;
                }
            }

            if (raster.valid())
            {
                u = (p.x() - raster->getExtent().xMin()) / raster->getExtent().width();
                v = (p.y() - raster->getExtent().yMin()) / raster->getExtent().height();

                // Note: This can happen on the map edges..
                // TODO: consider looping around for geo and clamping for projected
                u = clamp(u, 0.0, 1.0);
                v = clamp(v, 0.0, 1.0);

                p.z() = raster->getRawElevationUV(u, v);
            }
            else
            {
                p.z() = failValue;
            }
        }
        else
        {
            p.z() = failValue;
        }

        if (p.z() != failValue)
            ++count;
    }

    return count;
}

ElevationSample
ElevationPool::getSample(
    const GeoPoint& p,
    unsigned maxLOD,
    const Map* map,
    WorkingSet* ws,
    ProgressCallback* progress)
{
    // ensure the Pool is in sync with the map
    sync(map, ws);

    if (_elevationLayers.empty())
        return {};

    ScopedReadLock lk(_mutex);

    Internal::RevElevationKey key;

    // Need to limit maxLOD <= INT_MAX else std::min for lod will return -1 due to cast
    maxLOD = std::min(maxLOD, static_cast<unsigned>(std::numeric_limits<int>::max()));

    // returns the best LOD for the given point, or -1 if there is no data there
    int lod = std::min(getLOD(p.x(), p.y(), ws), (int)maxLOD);

    if (lod >= 0)
    {
        key._tilekey = map->getProfile()->createTileKey(p.x(), p.y(), lod);
        key._revision = getElevationHash(ws);

        osg::ref_ptr<ElevationTile> raster = getOrCreateRaster(
            key,   // key to query
            map,   // map to query
            true,  // fall back on lower resolution data if necessary
            ws,    // user's workingset
            progress);

        if (raster.valid())
        {
            return raster->getElevation(p.x(), p.y());
        }
    }
    return ElevationSample();
}

ElevationSample
ElevationPool::getSample(
    const GeoPoint& p,
    WorkingSet* ws,
    ProgressCallback* progress)
{
    if (!p.isValid())
        return {};

    osg::ref_ptr<const Map> map = _map.get();
    if (!map.valid() || !map->getProfile())
        return {};

    if (_elevationLayers.empty())
        return {};

    if (!p.getSRS()->isHorizEquivalentTo(map->getProfile()->getSRS()))
    {
        GeoPoint xp(p);
        xp.transformInPlace(map->getProfile()->getSRS());
        return getSample(xp, ~0, map.get(), ws, progress);
    }
    else
    {
        return getSample(p, ~0, map.get(), ws, progress);
    }
}

ElevationSample
ElevationPool::getSample(
    const GeoPoint& p,
    const Distance& resolution,
    WorkingSet* ws,
    ProgressCallback* progress)
{
    if (!p.isValid())
        return {};

    if (_elevationLayers.empty())
        return {};

    osg::ref_ptr<const Map> map;
    if (_map.lock(map) == false || map->getProfile() == NULL)
        return {};

    // mostly right. :)
    double resolutionInMapUnits = SpatialReference::transformUnits(
        resolution,
        map->getSRS(),
        p.y());

    unsigned maxLOD = map->getProfile()->getLevelOfDetailForHorizResolution(
        resolutionInMapUnits,
        ELEVATION_TILE_SIZE);

    if (!p.getSRS()->isHorizEquivalentTo(map->getProfile()->getSRS()))
    {
        GeoPoint xp(p);
        xp.transformInPlace(map->getProfile()->getSRS());
        return getSample(xp, maxLOD, map.get(), ws, progress);
    }
    else
    {
        return getSample(p, maxLOD, map.get(), ws, progress);
    }
}

bool
ElevationPool::getTile(
    const TileKey& tilekey,
    bool acceptLowerRes,
    osg::ref_ptr<ElevationTile>& out_tex,
    WorkingSet* ws,
    ProgressCallback* progress)
{
    osg::ref_ptr<const Map> map;
    if (!_map.lock(map))
        return false;

    // ensure we are in sync with the map
    sync(map.get(), ws);

    ScopedReadLock lk(_mutex);

    Internal::RevElevationKey key;
    key._tilekey = tilekey;
    key._revision = getElevationHash(ws);

    out_tex = getOrCreateRaster(
        key,
        _map.get(),
        acceptLowerRes,
        ws,
        progress);

    return out_tex.valid();
}

//...................................................................

AsyncElevationSampler::AsyncElevationSampler(
    const Map* map,
    unsigned numThreads) :

    _map(map),
    _arena(nullptr)
{
    _arena = jobs::get_pool("oe.asyncelevation");
    _arena->set_can_steal_work(false);
    _arena->set_concurrency(numThreads > 0 ? numThreads : _arena->concurrency());
}

Future<ElevationSample>
AsyncElevationSampler::getSample(const GeoPoint& p)
{
    return getSample(p, Distance(0, p.getXYUnits()));
}

Future<ElevationSample>
AsyncElevationSampler::getSample(const GeoPoint& point, const Distance& resolution)
{
    jobs::context c;
    c.pool = _arena;

    auto task = [=](Cancelable& cancelable)
        {
            ElevationSample sample;
            if (!cancelable.canceled())
            {
                osg::ref_ptr<const Map> map(_map);
                if (map.valid())
                {
                    osg::ref_ptr<ProgressCallback> progress = new ProgressCallback(&cancelable);

                    sample = map->getElevationPool()->getSample(point, resolution, &_ws, progress.get());
                }
            }
            return sample;
        };

    return jobs::dispatch(task, c);
}
