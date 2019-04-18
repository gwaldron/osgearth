
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
    Threading::ScopedMutexLock lock(_tilesMutex);
    _map = map;
    clearImpl();
}

void
ElevationPool::clear()
{
    Threading::ScopedMutexLock lock(_tilesMutex);
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
    Threading::ScopedMutexLock lock(_tilesMutex);
    _layers = layers;
    clearImpl();
}

void
ElevationPool::setTileSize(unsigned value)
{
    Threading::ScopedMutexLock lock(_tilesMutex);
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
ElevationPool::fetchTileFromMap(const TileKey& key, const ElevationLayerVector& layers, Tile* tile)
{
    tile->_loadTime = osg::Timer::instance()->tick();

    osg::ref_ptr<osg::HeightField> hf = new osg::HeightField();
    hf->allocate( _tileSize, _tileSize );

    // Initialize the heightfield to nodata
    hf->getFloatArray()->assign( hf->getFloatArray()->size(), NO_DATA_VALUE );

    TileKey keyToUse = key;
    while( !tile->_hf.valid() && keyToUse.valid() )
    {
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
            tile->_hf = GeoHeightField( hf.get(), keyToUse.getExtent() );
            tile->_bounds = keyToUse.getExtent().bounds();
        }
        else
        {
            keyToUse = keyToUse.createParentKey();
        }
    }

    return tile->_hf.valid();
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
ElevationPool::tryTile(const TileKey& key, const ElevationLayerVector& layers, osg::ref_ptr<Tile>& out)
{
    // first see whether the tile is available
    _tilesMutex.lock();

    // locate the tile in the local tile cache:
    osg::observer_ptr<Tile>& tile_obs = _tiles[key];

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
        _tilesMutex.unlock();

        bool ok = fetchTileFromMap(key, layers, tile.get());
        tile->_status.exchange( ok ? STATUS_AVAILABLE : STATUS_FAIL );
        
        out = ok ? tile.get() : 0L;
        return ok;
    }

    // This means the tile object is populated and available for use:
    else if ( tile->_status == STATUS_AVAILABLE )
    {
        OE_TEST << "  getTile(" << key.str() << ") -> available\n";
        out = tile.get();

        // Mark this tile as recently used:
        _mru.push_front(tile.get());

        // prune the MRU if necessary
        if (++_entries > _maxEntries)
        {
            popMRU();
            --_entries;
        }

        _tilesMutex.unlock();
        return true;
    }

    // This means the attempt to populate the tile with data failed.
    else if ( tile->_status == STATUS_FAIL )
    {
        OE_TEST << "  getTile(" << key.str() << ") -> fail\n";
        _tilesMutex.unlock();
        out = 0L;
        return false;
    }

    // This means tile data fetch is still in progress (in another thread)
    // and the caller should check back later.
    else //if ( tile->_status == STATUS_IN_PROGRESS )
    {
        OE_DEBUG << "  getTile(" << key.str() << ") -> in progress...waiting\n";
        _tilesMutex.unlock();
        out = 0L;
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
ElevationPool::getTile(const TileKey& key, const ElevationLayerVector& layers, osg::ref_ptr<ElevationPool::Tile>& output)
{   
    OE_START_TIMER(get);

    const double timeout = 30.0;
    osg::ref_ptr<Tile> tile;
    while( tryTile(key, layers, tile) && !tile.valid() && OE_GET_TIMER(get) < timeout)
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
            output = tile.get();
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
    ElevationEnvelope* e = new ElevationEnvelope();
    e->_inputSRS = srs; 
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
    }

    return e;
}

//........................................................................

ElevationEnvelope::ElevationEnvelope() :
_pool(0L)
{
    //nop
}

ElevationEnvelope::~ElevationEnvelope()
{
    //nop
}

bool
ElevationEnvelope::sample(double x, double y, float& out_elevation, float& out_resolution)
{
    out_elevation = NO_DATA_VALUE;
    out_resolution = 0.0f;
    bool foundTile = false;

    GeoPoint p(_inputSRS.get(), x, y, 0.0f, ALTMODE_ABSOLUTE);

    if (p.transformInPlace(_mapProfile->getSRS()))
    {
        // find the tile containing the point:
        for(ElevationPool::QuerySet::const_iterator tile_ref = _tiles.begin();
            tile_ref != _tiles.end();
            ++tile_ref)
        {
            ElevationPool::Tile* tile = tile_ref->get();

            if (tile->_bounds.contains(p.x(), p.y()))
            {
                foundTile = true;

                // Found an intersecting tile; sample the elevation:
                if (tile->_hf.getElevation(0L, p.x(), p.y(), INTERP_BILINEAR, 0L, out_elevation))
                {
                    out_resolution = tile->_hf.getXInterval();
                    // got it; finished
                    break;
                }
            }
        }

        // If we didn't find a tile containing the point, we need to ask the clamper
        // for the tile so we can add it to the query set.
        if (!foundTile)
        {
            TileKey key = _mapProfile->createTileKey(p.x(), p.y(), _lod);
            osg::ref_ptr<ElevationPool::Tile> tile;

            osg::ref_ptr<ElevationPool> pool;

            if (_pool.lock(pool) && pool->getTile(key, _layers, tile))
            {
                // Got the new tile; put it in the query set:
                _tiles.insert(tile.get());

                // Then sample the elevation:
                if (tile->_hf.getElevation(0L, p.x(), p.y(), INTERP_BILINEAR, 0L, out_elevation))
                {
                    out_resolution = 0.5*(tile->_hf.getXInterval() + tile->_hf.getYInterval());
                }
            }
        }
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
    METRIC_SCOPED("ElevationEnvelope::getElevation");
    float elevation, resolution;
    sample(x, y, elevation, resolution);
    return elevation;
}

std::pair<float, float>
ElevationEnvelope::getElevationAndResolution(double x, double y)
{
    METRIC_SCOPED("ElevationEnvelope::getElevationAndResolution");
    float elevation, resolution;
    sample(x, y, elevation, resolution);
    return std::make_pair(elevation, resolution);
}

unsigned
ElevationEnvelope::getElevations(const std::vector<osg::Vec3d>& input,
                                 std::vector<float>& output)
{
    METRIC_SCOPED_EX("ElevationEnvelope::getElevations", 1, "num", toString(input.size()).c_str());

    unsigned count = 0u;

    output.reserve(input.size());
    output.clear();

    // for each input point:
    for (std::vector<osg::Vec3d>::const_iterator v = input.begin(); v != input.end(); ++v)
    {
        float elevation, resolution;
        sample(v->x(), v->y(), elevation, resolution);
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
        
        if (sample(v->x(), v->y(), elevation, resolution))
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
        if (sample(centroid.x(), centroid.y(), elevation, resolution))
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
