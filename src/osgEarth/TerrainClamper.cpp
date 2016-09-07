
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
#include <osgEarth/TerrainClamper>
#include <osgEarth/TileKey>
#include <osg/Shape>

using namespace osgEarth;

#define LC "[TerrainClamper] "

TerrainClamper::TerrainClamper() :
_entries(0u),
_maxEntries( 128u )
{
    //nop
}

void
TerrainClamper::setMap(const Map* map)
{
    _frame.setMap( map );
}

bool
TerrainClamper::fetchTileFromMap(const TileKey& key, Tile* tile)
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

bool
TerrainClamper::tryTile(const TileKey& key, osg::ref_ptr<Tile>& out)
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
            _mru.pop_back();
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
            _mru.pop_back();
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
TerrainClamper::getTile(const TileKey& key, osg::ref_ptr<TerrainClamper::Tile>& output)
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

TerrainEnvelope*
TerrainClamper::createEnvelope(const SpatialReference* srs, unsigned lod)
{
    TerrainEnvelope* e = new TerrainEnvelope();
    e->_inputSRS = srs;
    e->_mapProfile = _frame.getProfile();
    e->_lod = lod;
    e->_clamper = this;
    return e;
}

//........................................................................

float
TerrainEnvelope::sample(double x, double y)
{
    float elevation = NO_DATA_VALUE;
    bool foundTile = false;

    GeoPoint p(_inputSRS, x, y, 0.0f, ALTMODE_ABSOLUTE);

    if (p.transformInPlace(_mapProfile->getSRS()))
    {
        // find the tile containing the point:
        for(TerrainClamper::QuerySet::const_iterator tile_ref = _tiles.begin();
            tile_ref != _tiles.end();
            ++tile_ref)
        {
            TerrainClamper::Tile* tile = tile_ref->get();

            if (tile->_bounds.contains(p.x(), p.y()))
            {
                foundTile = true;

                // Found an intersecting tile; sample the elevation:
                if (tile->_hf.getElevation(0L, p.x(), p.y(), INTERP_BILINEAR, 0L, elevation))
                {
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
            osg::ref_ptr<TerrainClamper::Tile> tile;
            if (_clamper->getTile(key, tile))
            {
                // Got the new tile; put it in the query set:
                _tiles.insert(tile.get());

                // Then sample the elevation:
                tile->_hf.getElevation(0L, p.x(), p.y(), INTERP_BILINEAR, 0L, elevation);
            }
        }
    }
    else
    {
        OE_WARN << LC << "sample: xform failed" << std::endl;
    }

    // push the result, even if it was not found and it's NO_DATA_VALUE
    return elevation;
}

unsigned
TerrainEnvelope::getElevations(const std::vector<osg::Vec3d>& input,
                               std::vector<float>& output)
{
    unsigned count = 0u;

    output.reserve(input.size());
    output.clear();

    // for each input point:
    for (std::vector<osg::Vec3d>::const_iterator v = input.begin(); v != input.end(); ++v)
    {
        float elevation = sample(v->x(), v->y());
        output.push_back(elevation);
        if (elevation != NO_DATA_VALUE)
            ++count;
    }

    if (count < input.size())
    {
        OE_WARN << LC << "Issue: Envelope had failed samples" << std::endl;
        for (TerrainClamper::QuerySet::const_iterator tile_ref = _tiles.begin(); tile_ref != _tiles.end(); ++tile_ref)
        {
            TerrainClamper::Tile* tile = tile_ref->get();
            OE_WARN << LC << " ... tile " << tile->_bounds.toString() << std::endl;
        }
        OE_WARN << LC << std::endl;
    }

    return count;
}

bool
TerrainEnvelope::getElevationExtrema(const std::vector<osg::Vec3d>& input,
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

        float elevation = sample(v->x(), v->y());
        if ( elevation != NO_DATA_VALUE )
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

        float elevation = sample(centroid.x(), centroid.y());
        if (elevation != NO_DATA_VALUE)
        {
            if (elevation < min) min = elevation;
            if (elevation > max) max = elevation;
        }
    }

    return (min <= max);
}
