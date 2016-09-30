
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
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
#include <osgEarth/TileKey>
#include <osg/Shape>

using namespace osgEarth;

#define LC "[ElevationPool] "

ElevationPool::ElevationPool() :
_entries(0u),
_maxEntries( 128u )
{
    //nop
}

void
ElevationPool::setMap(const Map* map)
{
    Threading::ScopedMutexLock lock(_tilesMutex);
    _frame.setMap( map );
    _tiles.clear();
    _mru.clear();
    _entries = 0u;
}

bool
ElevationPool::fetchTileFromMap(const TileKey& key, Tile* tile)
{
    // TODO: parameterize me
    const int tileSize = 33;

    tile->_loadTime = osg::Timer::instance()->tick();

    osg::ref_ptr<osg::HeightField> hf = new osg::HeightField();
    hf->allocate( tileSize, tileSize );

    // Initialize the heightfield to nodata
    hf->getFloatArray()->assign( hf->getFloatArray()->size(), NO_DATA_VALUE );

    TileKey keyToUse = key;
    while( !tile->_hf.valid() && keyToUse.valid() )
    {
        if (_frame.populateHeightField(hf, keyToUse, false /*heightsAsHAE*/, 0L))
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
ElevationPool::tryTile(const TileKey& key, osg::ref_ptr<Tile>& out)
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
        OE_DEBUG << "  getTile(" << key.str() << ") -> fetch from map\n";
        tile->_status.exchange(STATUS_IN_PROGRESS);
        _tilesMutex.unlock();

        bool ok = fetchTileFromMap(key, tile.get());
        tile->_status.exchange( ok ? STATUS_AVAILABLE : STATUS_FAIL );
        
        out = ok ? tile.get() : 0L;
        return ok;
    }

    // This means the tile object is populated and available for use:
    else if ( tile->_status == STATUS_AVAILABLE )
    {
        OE_DEBUG << "  getTile(" << key.str() << ") -> available\n";
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
        OE_DEBUG << "  getTile(" << key.str() << ") -> fail\n";
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
        return true;            // out:NULL => check back later please.
    }
}

bool
ElevationPool::getTile(const TileKey& key, osg::ref_ptr<ElevationPool::Tile>& output)
{
    // Synchronize the MapFrame to its Map; if there's an update,
    // clear out the internal cache and MRU.
    if ( _frame.needsSync() )
    {
        if (_frame.sync())
        {
            _tilesMutex.lock();
            _tiles.clear();
            _mru.clear();
            _entries = 0u;
            _tilesMutex.unlock();
        }
    }
   
    OE_START_TIMER(get);

    const double timeout = 30.0;
    osg::ref_ptr<Tile> tile;
    while( tryTile(key, tile) && !tile.valid() && OE_GET_TIMER(get) < timeout)
    {
        // condition: another thread is working on fetching the tile from the map,
        // so wait and try again later. Do this until we succeed or time out.
        OpenThreads::Thread::YieldCurrentThread();
    }

    if ( !tile.valid() && OE_GET_TIMER(get) >= timeout )
    {
        // this means we timed out trying to fetch the map tile.
        OE_WARN << LC << "Timout fetching tile " << key.str() << std::endl;
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
    e->_mapProfile = _frame.getProfile();
    e->_lod = lod;
    e->_clamper = this;
    return e;
}

//........................................................................

bool
ElevationEnvelope::sample(double x, double y, float& out_elevation, float& out_resolution)
{
    out_elevation = NO_DATA_VALUE;
    out_resolution = 0.0f;
    bool foundTile = false;

    GeoPoint p(_inputSRS, x, y, 0.0f, ALTMODE_ABSOLUTE);

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
            if (_clamper->getTile(key, tile))
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
    float elevation, resolution;
    sample(x, y, elevation, resolution);
    return elevation;
}

std::pair<float, float>
ElevationEnvelope::getElevationAndResolution(double x, double y)
{
    float elevation, resolution;
    sample(x, y, elevation, resolution);
    return std::make_pair(elevation, resolution);
}

unsigned
ElevationEnvelope::getElevations(const std::vector<osg::Vec3d>& input,
                               std::vector<float>& output)
{
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

    if (count < input.size())
    {
        OE_WARN << LC << "Issue: Envelope had failed samples" << std::endl;
        for (ElevationPool::QuerySet::const_iterator tile_ref = _tiles.begin(); tile_ref != _tiles.end(); ++tile_ref)
        {
            ElevationPool::Tile* tile = tile_ref->get();
            OE_WARN << LC << " ... tile " << tile->_bounds.toString() << std::endl;
        }
        OE_WARN << LC << std::endl;
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

    unsigned count = 0;

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
