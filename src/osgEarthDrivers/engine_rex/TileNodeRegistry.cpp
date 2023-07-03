/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
#include "TileNodeRegistry"

#include <osgEarth/Metrics>

using namespace osgEarth::REX;
using namespace osgEarth;

#define LC "[TileNodeRegistry] "

#define OE_TEST OE_NULL

#define SENTRY_VALUE nullptr

#define PROFILING_REX_TILES "Live Terrain Tiles"

//----------------------------------------------------------------------------

TileNodeRegistry::TileNodeRegistry() :
    _notifyNeighbors(false),
    _mutex("TileNodeRegistry(OE)")
{
    //nop
}

TileNodeRegistry::~TileNodeRegistry()
{
    releaseAll(nullptr);
}

void
TileNodeRegistry::setNotifyNeighbors(bool value)
{
    _notifyNeighbors = value;
}

void
TileNodeRegistry::setDirty(
    const GeoExtent& extent,
    unsigned minLevel,
    unsigned maxLevel,
    const CreateTileManifest& manifest)
{
    ScopedMutexLock lock(_mutex);
    
    for( TileTable::iterator i = _tiles.begin(); i != _tiles.end(); ++i )
    {
        const TileKey& key = i->first;

        if (minLevel <= key.getLOD() && 
            maxLevel >= key.getLOD() &&
            (extent.isInvalid() || extent.intersects(key.getExtent())))
        {
            i->second._tile->refreshLayers(manifest);
        }
    }
}

void
TileNodeRegistry::add(TileNode* tile)
{
    ScopedMutexLock lock(_mutex);

    auto& entry = _tiles[tile->getKey()];
    entry._tile = tile;
    bool recyclingOrphan = entry._trackerToken != nullptr;
    entry._trackerToken = _tracker.use(tile, nullptr);

    // Start waiting on our neighbors.
    // (If we're recycling and orphaned record, we need to remove old listeners first)
    if (_notifyNeighbors)
    {
        const TileKey& key = tile->getKey();

        // If we're recycling, we need to remove the old listeners first
        if (recyclingOrphan)
        {
            stopListeningFor(key.createNeighborKey(1, 0), key);
            stopListeningFor(key.createNeighborKey(0, 1), key);
        }

        startListeningFor(key.createNeighborKey(1, 0), tile);
        startListeningFor(key.createNeighborKey(0, 1), tile);

        // check for tiles that are waiting on this tile, and notify them!
        TileKeyOneToMany::iterator notifier = _notifiers.find( tile->getKey() );
        if ( notifier != _notifiers.end() )
        {
            TileKeySet& listeners = notifier->second;

            for(TileKeySet::iterator listener = listeners.begin(); listener != listeners.end(); ++listener)
            {
                TileTable::iterator i = _tiles.find( *listener );
                if ( i != _tiles.end())
                {
                    i->second._tile->notifyOfArrival( tile );
                }
            }
            _notifiers.erase( notifier );
        }

        OE_DEBUG << LC
            << ": tiles=" << _tiles.size()
            << ", notifiers=" << _notifiers.size()
            << std::endl;
    }
}

void
TileNodeRegistry::startListeningFor(
    const TileKey& tileToWaitFor,
    TileNode* waiter)
{
    // ASSUME EXCLUSIVE LOCK

    TileTable::iterator i = _tiles.find(tileToWaitFor);
    if (i != _tiles.end())
    {
        TileNode* tile = i->second._tile.get();

        OE_DEBUG << LC << waiter->getKey().str() << " listened for " << tileToWaitFor.str()
            << ", but it was already in the repo.\n";

        waiter->notifyOfArrival( tile );
    }
    else
    {
        OE_DEBUG << LC << waiter->getKey().str() << " listened for " << tileToWaitFor.str() << ".\n";
        _notifiers[tileToWaitFor].insert( waiter->getKey() );
    }
}

void
TileNodeRegistry::stopListeningFor(
    const TileKey& tileToWaitFor,
    const TileKey& waiterKey)
{
    // ASSUME EXCLUSIVE LOCK

    TileKeyOneToMany::iterator i = _notifiers.find(tileToWaitFor);
    if (i != _notifiers.end())
    {
        // remove the waiter from this set:
        i->second.erase(waiterKey);

        // if the set is now empty, remove the set entirely
        if (i->second.empty())
        {
            _notifiers.erase(i);
        }
    }
}

void
TileNodeRegistry::releaseAll(osg::State* state)
{
    ScopedMutexLock lock(_mutex);

    for (auto& tile : _tiles)
    {
        tile.second._tile->releaseGLObjects(state);
    }
    _tiles.clear();

    _tracker.reset();

    _notifiers.clear();

    _tilesToUpdate.clear();

    OE_PROFILING_PLOT(PROFILING_REX_TILES, (float)(_tiles.size()));
}

void
TileNodeRegistry::touch(TileNode* tile, osg::NodeVisitor& nv)
{
    ScopedMutexLock lock(_mutex);

    TileTable::iterator i = _tiles.find(tile->getKey());

    OE_SOFT_ASSERT_AND_RETURN(i != _tiles.end(), void());

    _tracker.use(tile, i->second._trackerToken);

    if (tile->updateRequired())
    {
        _tilesToUpdate.push_back(tile->getKey());
    }
}

void
TileNodeRegistry::update(osg::NodeVisitor& nv)
{
    ScopedMutexLock lock(_mutex);

    if (!_tilesToUpdate.empty())
    {
        // Sorting these from high to low LOD will reduce the number 
        // of inheritance steps each updated image will have to perform
        // against the tile's children
        std::sort(
            _tilesToUpdate.begin(),
            _tilesToUpdate.end(),
            [](const TileKey& lhs, const TileKey& rhs) {
                return lhs.getLOD() > rhs.getLOD();
            });

        for (auto& key : _tilesToUpdate)
        {
            auto iter = _tiles.find(key);
            if (iter != _tiles.end())
            {
                iter->second._tile->update(nv);
            }
        }

        _tilesToUpdate.clear();
    }
}

void
TileNodeRegistry::collectDormantTiles(
    osg::NodeVisitor& nv,
    double oldestAllowableTime,
    unsigned oldestAllowableFrame,
    float farthestAllowableRange,
    unsigned maxTiles,
    std::vector<osg::observer_ptr<TileNode>>& output)
{
    ScopedMutexLock lock(_mutex);

    unsigned count = 0u;

    const auto disposeTile = [&](TileNode* tile) -> bool
    {
        OE_SOFT_ASSERT_AND_RETURN(tile != nullptr, true);

        const TileKey& key = tile->getKey();

        if (tile->getDoNotExpire() == false &&
            tile->getLastTraversalTime() < oldestAllowableTime &&
            tile->getLastTraversalFrame() < oldestAllowableFrame &&
            tile->getLastTraversalRange() > farthestAllowableRange &&
            tile->areSiblingsDormant())
        {
            if (_notifyNeighbors)
            {
                // remove neighbor listeners:
                stopListeningFor(key.createNeighborKey(1, 0), key);
                stopListeningFor(key.createNeighborKey(0, 1), key);
            }

            output.push_back(tile);

            _tiles.erase(key);

            return true; // dispose it
        }
        else
        {
            tile->resetTraversalRange();

            return false; // keep it
        }
    };
    
    _tracker.flush(maxTiles, disposeTile);

    OE_PROFILING_PLOT(PROFILING_REX_TILES, (float)(_tiles.size()));
}
