
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2008-2016 Pelican Mapping
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
#include <osgEarth/ElevationPool>
#include <osgEarth/Map>
#include <osgEarth/Metrics>

using namespace osgEarth;

#define LC "[ElevationPool] "

#define USE_KEY_MAPPING_MEMORY

#define USE_ENVELOPE_DATA_EXTENTS

#define USE_ENVELOPE_CONTEXT

#define USE_ENVELOPE_TILE_CACHE

#define OE_TEST OE_DEBUG


ElevationPool::ElevationPool() :
_entries(0u),
_maxEntries( 128u ),
_tileSize( 257u )
{
    //nop
    //_opQueue = Registry::instance()->getAsyncOperationQueue();
    if (!_opQueue.valid())
    {
        _opQueue = new osg::OperationQueue();
        for (unsigned i=0; i<2; ++i)
        {
            osg::OperationThread* thread = new osg::OperationThread();
            thread->setOperationQueue(_opQueue.get());
            thread->start();
            _opThreads.push_back(thread);
        }
    }
}

ElevationPool::~ElevationPool()
{
    stopThreading();
}

void
ElevationPool::setMap(const Map* map)
{
    Threading::ScopedMutexLock lock(_tiles.mutex());
    _map = map;
    clearImpl();
}

void
ElevationPool::clear()
{
    Threading::ScopedMutexLock lock(_tiles.mutex());
    clearImpl();
}

void
ElevationPool::stopThreading()
{
    _opQueue->releaseAllOperations();
    
    for (unsigned i = 0; i<_opThreads.size(); ++i)
    _opThreads[i]->setDone(true);
}

void
ElevationPool::setElevationLayers(const ElevationLayerVector& layers)
{
    Threading::ScopedMutexLock lock(_tiles.mutex());
    _layers = layers;
    clearImpl();
}

void
ElevationPool::setTileSize(unsigned value)
{
    Threading::ScopedMutexLock lock(_tiles.mutex());
    _tileSize = value;
    clearImpl();
}

Future<ElevationSample>
ElevationPool::getElevation(const GeoPoint& point, unsigned lod)
{
    GetElevationOp* op = new GetElevationOp(this, point, lod);
    Future<ElevationSample> result = op->_promise.getFuture();
    _opQueue->add(op);
    return result;
}

ElevationPool::GetElevationOp::GetElevationOp(ElevationPool* pool, const GeoPoint& point, unsigned lod) :
_pool(pool), _point(point), _lod(lod)
{
    //nop
}

void
ElevationPool::GetElevationOp::operator()(osg::Object*)
{
    osg::ref_ptr<ElevationPool> pool;
    if (!_promise.isAbandoned() && _pool.lock(pool))
    {
        osg::ref_ptr<ElevationEnvelope> env = pool->createEnvelope(_point.getSRS(), _lod);
        std::pair<float, float> r = env->getElevationAndResolution(_point.x(), _point.y());
        _promise.resolve(new ElevationSample(r.first, r.second));
    }
}

bool
ElevationPool::fetchTileFromMap(
    const TileKey& key, 
    const ElevationLayerVector& layers,
    KeyFetchMemory& memory,
    Tile* out_tile) const
{
    out_tile->_loadTime = osg::Timer::instance()->tick();

    osg::ref_ptr<osg::HeightField> hf = new osg::HeightField();
    hf->allocate( _tileSize, _tileSize );

    // Initialize the heightfield to nodata
    hf->getFloatArray()->assign( hf->getFloatArray()->size(), NO_DATA_VALUE );

    std::vector<TileKey> keysTried;
    TileKey keyToUse = key;

    while( !out_tile->_hf.valid() && keyToUse.valid() )
    {
        keysTried.push_back(keyToUse);

        bool ok;
        if (_layers.empty())
        {
            OE_TEST << LC << "Populating from envelope (" << keyToUse.str() << ")\n";
            ok = layers.populateHeightFieldAndNormalMap(hf.get(), 0L, keyToUse, 0L, INTERP_BILINEAR, 0L);
        }
        else
        {
            OE_TEST << LC << "Populating from layers (" << keyToUse.str() << ")\n";
            ok = _layers.populateHeightFieldAndNormalMap(hf.get(), 0L, keyToUse, 0L, INTERP_BILINEAR, 0L);
        }

        if (ok)
        {
            // store the *actual* key (keyToUse) with the heightfield;
            // may be an ancestor of key in the case of fallback.
            out_tile->_hf = GeoHeightField( hf.get(), keyToUse.getExtent() );
        }
        else
        {
            keyToUse = keyToUse.createParentKey();
        }
    }

#ifdef USE_KEY_MAPPING_MEMORY
    // every key that failed maps to the final key:
    for(std::vector<TileKey>::const_iterator i = keysTried.begin();
        i != keysTried.end();
        ++i)
    {
        memory[*i] = keyToUse;
    }
#endif

    return out_tile->_hf.valid();
}

void
ElevationPool::popMRU()
{
    // first rememeber the key of the item we're about the pop:
    TileKey key = _mru.back()->_key;

    // establish a temporary observer on the item:
    osg::observer_ptr<Tile> temp = _mru.back().get();

    // pop the Tile from the MRU:
    _mru.pop_back();

    // if our observer went to NULL, we know there are no more pointers
    // to that Tile in the MRU, and we can remove it from the main list:
    if (!temp.valid())
    {
        _tiles.erase(key);
    }
}

bool
ElevationPool::tryTile(
    const TileKey& key, 
    const ElevationLayerVector& layers, 
    ElevationPool::KeyFetchMemory& memory,
    osg::ref_ptr<Tile>& out_tile)
{
#ifdef USE_KEY_MAPPING_MEMORY
    TileKey keyToUse;
    KeyFetchMemory::iterator i = memory.find(key);
    keyToUse = (i != memory.end())? i->second : key;

    if (keyToUse.valid() == false)
        return false;
#else
    TileKey keyToUse = key;
#endif

    // first see whether the tile is available
    _tiles.lock();

    // locate the tile in the local tile cache:
    osg::observer_ptr<Tile>& tile_obs = _tiles[keyToUse];

    osg::ref_ptr<Tile> tile;

    // Get a safe pointer to it. If this is NULL, we need to create and
    // fetch a new tile from the Map.
    if (!tile_obs.lock(tile))
    {
        // a new tile; status -> EMPTY
        tile = new Tile();
        tile->_key = key;

        // update the LRU:
        _mru.push_front(tile.get());

        // prune the MRU if necessary:
        if (++_entries > _maxEntries )
        {
            popMRU();
            --_entries;
        }

        // add to the main cache (after putting it on the LRU).
        tile_obs = tile;
    }
       
    // This means the tile object exists but has yet to be populated:
    if ( tile->_status == STATUS_EMPTY )
    {
        OE_TEST << "  getTile(" << key.str() << ") -> fetch from map\n";
        tile->_status.exchange(STATUS_IN_PROGRESS);
        _tiles.unlock();

        bool ok = fetchTileFromMap(keyToUse, layers, memory, tile.get());
        tile->_status.exchange( ok ? STATUS_AVAILABLE : STATUS_FAIL );
        
        out_tile = ok ? tile.get() : 0L;
        return ok;
    }

    // This means the tile object is populated and available for use:
    else if ( tile->_status == STATUS_AVAILABLE )
    {
        OE_TEST << "  getTile(" << key.str() << ") -> available\n";
        out_tile = tile.get();

        // Mark this tile as recently used:
        _mru.push_front(tile.get());

        // prune the MRU if necessary
        if (++_entries > _maxEntries)
        {
            popMRU();
            --_entries;
        }

        _tiles.unlock();
        return true;
    }

    // This means the attempt to populate the tile with data failed.
    else if ( tile->_status == STATUS_FAIL )
    {
        OE_TEST << "  getTile(" << key.str() << ") -> fail\n";
        _tiles.unlock();
        out_tile = 0L;
        return false;
    }

    // This means tile data fetch is still in progress (in another thread)
    // and the caller should check back later.
    else //if ( tile->_status == STATUS_IN_PROGRESS )
    {
        OE_DEBUG << "  getTile(" << key.str() << ") -> in progress...waiting\n";
        _tiles.unlock();
        out_tile = 0L;
        return true;            // out:NULL => check back later please.
    }
}

void
ElevationPool::clearImpl()
{
    // assumes the tiles lock is taken.
    _tiles.clear();
    _mru.clear();
    _entries = 0u;
}

bool
ElevationPool::getTile(
    const TileKey& key, 
    const ElevationLayerVector& layers,
    ElevationPool::KeyFetchMemory& memory,
    osg::ref_ptr<ElevationPool::Tile>& out_tile)
{   
    OE_START_TIMER(get);

#ifdef USE_KEY_MAPPING_MEMORY
    // trivial rejection test
    KeyFetchMemory::iterator i = memory.find(key);
    if (i != memory.end() && i->second.valid() == false)
        return false;
#endif

    const double timeout = 30.0;
    osg::ref_ptr<Tile> tile;
    while( tryTile(key, layers, memory, tile) && !tile.valid() && OE_GET_TIMER(get) < timeout)
    {
        // condition: another thread is working on fetching the tile from the map,
        // so wait and try again later. Do this until we succeed or time out.
        OpenThreads::Thread::YieldCurrentThread();
    }

    if ( !tile.valid() && OE_GET_TIMER(get) >= timeout )
    {
        // this means we timed out trying to fetch the map tile.
        OE_TEST << LC << "Timeout fetching tile " << key.str() << std::endl;
    }

    if ( tile.valid() )
    {
        if ( tile->_hf.valid() )
        {
            // got a valid tile, so push it to the query set.
            out_tile = tile.get();
        }
        else
        {
            OE_WARN << LC << "Got a tile with an invalid HF (" << key.str() << ")\n";
        }
    }

    return tile.valid();
}

ElevationEnvelope*
ElevationPool::createEnvelope(const SpatialReference* srs, unsigned lod)
{
    osg::ref_ptr<ElevationEnvelope> e = new ElevationEnvelope();
    e->_inputSRS = srs; 
    e->_requestedLOD = lod;
    e->_lod = lod;
    e->_pool = this;
    
    osg::ref_ptr<const Map> map;
    if (_map.lock(map))
    {
        if (_layers.size() > 0)
        {
            // user-specified layers
            e->_layers = _layers;
        }
        else
        {
            // all elevation layers
            map->getLayers(e->_layers);
        }

        e->_mapProfile = map->getProfile();

        e->collectDataExtents();
    }
    else
    {
        e = NULL;
    }

    return e.release();
}

//........................................................................

ElevationEnvelope::ElevationEnvelope() :
_pool(0L),
_requestedLOD(0),
_lod(0),
_queries(0u),
_contexthits(0u),
_cachehits(0u),
_newcontexts(0u),
_fails(0u)
{
    //nop
}

ElevationEnvelope::~ElevationEnvelope()
{
    //nop
}

bool
ElevationEnvelope::sample(
    double x, double y,
    Context* context,
    float& out_elevation, float& out_resolution)
{
    out_elevation = NO_DATA_VALUE;

    ++_queries;

    // Keep the envelope in sync with the elevation layers.
    unsigned changes = 0;
    for(unsigned i=0; i<_layers.size(); ++i)
    {
        if (_layers[i]->getRevision() != _revisions[i])
        {
            _revisions[i] = _layers[i]->getRevision();
            ++changes;
        }
    }
    if (changes > 0)
    {
        _memory.clear();
        _tiles.clear();
        context->_tiles.clear();
        collectDataExtents();
    }

    out_elevation = NO_DATA_VALUE;
    out_resolution = 0.0f;
    bool foundTile = false;
    osg::ref_ptr<ElevationPool::Tile> tile;

    GeoPoint p(_inputSRS.get(), x, y, 0.0f, ALTMODE_ABSOLUTE);


    if (p.transformInPlace(_mapProfile->getSRS()))
    {
        unsigned lodToUse = _lod;

#ifdef USE_ENVELOPE_DATA_EXTENTS
        // check the data extents under the point to come up with an LOD.
        bool foundData = false;
        for(SortedDataExtentList::const_iterator i = _dataExtentsSortedHiToLoRes.begin();
            i != _dataExtentsSortedHiToLoRes.end();
            ++i)
        {
            if (i->maxLevel().isSet() && i->contains(p.x(), p.y()))
            {
                lodToUse = osg::minimum(_lod, i->maxLevel().get());
                foundData = true;
                break;
            }
        }

        if (!foundData)
        {
            return false;
        }
#endif

#ifdef USE_ENVELOPE_TILE_CACHE
        // See if we have a cached tile containing the point:
        if (!foundTile && !context)
        {
            for(ElevationPool::QuerySet::const_iterator tile_ref = _tiles.begin();
                tile_ref != _tiles.end();
                ++tile_ref)
            {
                tile = tile_ref->get();

                // Important: test against the bounds of the original key that was used
                // to make the tile request, even if the request fell back on an ancestor
                // key. We cannot assume that points outside the original request bounds
                // would result in the same tile.
                if (lodToUse <= tile->_key.getLOD() &&
                    tile->_key.getExtent().contains(p.x(), p.y()))
                {
                    foundTile = true;
                    ++_cachehits;
                    break;
                }
            }
        }
#endif    

        // If we still don't have a tile, we need to ask the pool for the tile.
        if (!foundTile)
        {
#ifdef USE_ENVELOPE_CONTEXT

            if (context && context->_tiles.empty())
                ++_newcontexts;

            // If the user passed in a context, check that first.
            if (context && context->_tiles.empty() == false)
            {
                for(Context::TileMRU::iterator i = context->_tiles.begin();
                    i != context->_tiles.end();
                    ++i)
                {
                    ElevationPool::Tile* temp = i->get();

                    if (lodToUse <= temp->_key.getLOD() &&
                        temp->_key.getExtent().contains(p.x(), p.y()))
                    {
                        tile = temp;
                        foundTile = true;
                        context->_tiles.erase(i); // will re-push it to front later
                        ++_contexthits;
                        break;
                    }
                }
            }
#endif

            if (!foundTile)
            {
                TileKey key = _mapProfile->createTileKey(p.x(), p.y(), lodToUse);

                osg::ref_ptr<ElevationPool> pool;

                if (_pool.lock(pool) && pool->getTile(key, _layers, _memory, tile))
                {
                    foundTile = true;

#ifdef USE_ENVELOPE_TILE_CACHE
                    // Got the new tile; put it in the query set:
                    _tiles.insert(tile.get());
#endif
                }
            }
        }

        // Finally, so the actual elevation query against out found tile.
        if (foundTile)
        {
            if (tile->_hf.getElevation(0L, p.x(), p.y(), INTERP_BILINEAR, 0L, out_elevation))
            {
                out_resolution = 0.5*(tile->_hf.getXInterval() + tile->_hf.getYInterval());
            }

#ifdef USE_ENVELOPE_CONTEXT
            // If the user passed in a context, store the tile there so the next query
            // for the same context has a chance of being faster.
            if (context)
            {
                context->_tiles.push_front(tile.get());
                if (context->_tiles.size() > 4)
                    context->_tiles.pop_back();
            }
#endif
        }
        else
        {
            ++_fails;
        }

#if 0
        if (_queries % 500 == 0)
        {
            OE_INFO 
                <<": Q="<<_queries
                <<"; Ctx=" << _contexthits << "("<<100.0f*(float)_contexthits/(float)_queries<<")"
                <<"; Cache=" << 100.0f*(float)_cachehits/(float)_queries
                <<"; NewCtx=" << _newcontexts
                <<"; Fail=" << 100.0f*(float)_fails/(float)_queries
                << std::endl;

            _queries = 0, _contexthits = 0, _cachehits = 0, _newcontexts = 0, _fails = 0;
        }
#endif
    }
    else
    {
        OE_WARN << LC << "sample: xform failed" << std::endl;
    }

    // push the result, even if it was not found and it's NO_DATA_VALUE
    return out_elevation != NO_DATA_VALUE;
}

float
ElevationEnvelope::getElevation(double x, double y)
{
    OE_PROFILING_ZONE;

    float elevation, resolution;
    sample(x, y, NULL, elevation, resolution);
    return elevation;
}

float
ElevationEnvelope::getElevation(double x, double y, ElevationEnvelope::Context& context)
{
    OE_PROFILING_ZONE;

    float elevation, resolution;
    sample(x, y, &context, elevation, resolution);
    return elevation;
}

std::pair<float, float>
ElevationEnvelope::getElevationAndResolution(double x, double y)
{
    OE_PROFILING_ZONE;
    float elevation, resolution;
    sample(x, y, NULL, elevation, resolution);
    return std::make_pair(elevation, resolution);
}

unsigned
ElevationEnvelope::getElevations(const std::vector<osg::Vec3d>& input,
                                 std::vector<float>& output)
{
    OE_PROFILING_ZONE;
    OE_PROFILING_ZONE_TEXT(Stringify() << "Count " << input.size());

    unsigned count = 0u;

    output.reserve(input.size());
    output.clear();

    // for each input point:
    for (std::vector<osg::Vec3d>::const_iterator v = input.begin(); v != input.end(); ++v)
    {
        float elevation, resolution;
        sample(v->x(), v->y(), NULL, elevation, resolution);
        output.push_back(elevation);
        if (elevation != NO_DATA_VALUE)
            ++count;
    }

    return count;
}

bool
ElevationEnvelope::getElevationExtrema(const std::vector<osg::Vec3d>& input,
                                       float& min, float& max)
{
    if (input.empty())
        return false;

    min = FLT_MAX, max = -FLT_MAX;

    osg::Vec3d centroid;

    for (std::vector<osg::Vec3d>::const_iterator v = input.begin(); v != input.end(); ++v)
    {
        centroid += *v;

        float elevation, resolution;
        
        if (sample(v->x(), v->y(), NULL, elevation, resolution))
        {
            if (elevation < min) min = elevation;
            if (elevation > max) max = elevation;
        }
    }

    // If none of the feature points clamped, try the feature centroid.
    // It's possible (but improbable) that the feature encloses the envelope
    if (min > max)
    {
        centroid /= input.size();

        float elevation, resolution;
        if (sample(centroid.x(), centroid.y(), NULL, elevation, resolution))
        {
            if (elevation < min) min = elevation;
            if (elevation > max) max = elevation;
        }
    }

    return (min <= max);
}

const SpatialReference*
ElevationEnvelope::getSRS() const
{
    return _inputSRS.get();
}

void
ElevationEnvelope::collectDataExtents()
{
    // Restrict the query resolution to the maximum available elevation data resolution:
    _revisions.clear();

    _lod = _requestedLOD;

    unsigned maxLOD = 0u;
    for(ElevationLayerVector::const_iterator i = _layers.begin();
        i != _layers.end();
        ++i)
    {
        const ElevationLayer* layer = i->get();

        if (layer->getDataExtentsUnion().maxLevel().isSet())
        {
            maxLOD = osg::maximum(maxLOD, layer->getDataExtentsUnion().maxLevel().get());
        }

        // record the revision at the time of creation:
        _revisions.push_back(layer->getRevision());
    }
    if (maxLOD > 0u)
    {
        _lod = osg::minimum(_requestedLOD, maxLOD);
    }

    _dataExtentsSortedHiToLoRes.clear();

    for(ElevationLayerVector::const_iterator i = _layers.begin();
        i != _layers.end();
        ++i)
    {
        const ElevationLayer* layer = i->get();
        const DataExtentList& del = layer->getDataExtents();
        for(DataExtentList::const_iterator k = del.begin();
            k != del.end();
            ++k)
        {
            const DataExtent& de = *k;

            GeoExtent localExtent = _mapProfile->clampAndTransformExtent(de);

            if (!de.maxLevel().isSet())
            {
                _dataExtentsSortedHiToLoRes.push_front(DataExtent(localExtent));
            }
            else
            {
                unsigned localMaxLevel = _mapProfile->getEquivalentLOD(layer->getProfile(), de.maxLevel().get());

                SortedDataExtentList::iterator m;
                for(m = _dataExtentsSortedHiToLoRes.begin();
                    m != _dataExtentsSortedHiToLoRes.end();
                    ++m)
                {
                    const DataExtent& rhs = *m;
                    if (rhs.maxLevel().isSet() && localMaxLevel > rhs.maxLevel().get())
                    {
                        break;
                    }
                }
                _dataExtentsSortedHiToLoRes.insert(m, DataExtent(localExtent, 0, localMaxLevel));
            }
        }
    }

#if 0
    OE_INFO << "Extents:" << std::endl;
    for(SortedDataExtentList::const_iterator i = _dataExtentsSortedHiToLoRes.begin();
        i != _dataExtentsSortedHiToLoRes.end();
        ++i)
    {
        OE_INFO << i->toString()
            << " maxLevel=" << (i->maxLevel().isSet()? i->maxLevel().get() : 99u)
            << std::endl;
    }
#endif
}
