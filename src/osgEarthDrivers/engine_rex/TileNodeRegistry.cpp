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
//#define OE_TEST OE_INFO

#define SENTRY_VALUE NULL

#define PROFILING_REX_TILES "Live Terrain Tiles"

//----------------------------------------------------------------------------

TileNodeRegistry::TileNodeRegistry(const std::string& name) :
_name              ( name ),
_revisioningEnabled( false ),
_notifyNeighbors   ( false ),
_firstLOD          ( 0u ),
_mutex("TileNodeRegistry(OE)")
{
    _tracker.push_front(SENTRY_VALUE);
    _sentryptr = _tracker.begin();
}

TileNodeRegistry::~TileNodeRegistry()
{
    releaseAll(NULL);
}

void
TileNodeRegistry::setRevisioningEnabled(bool value)
{
    _revisioningEnabled = value;
}

void
TileNodeRegistry::setNotifyNeighbors(bool value)
{
    _notifyNeighbors = value;
}

void
TileNodeRegistry::setMapRevision(const Revision& rev,
                                 bool            setToDirty)
{
    if ( _revisioningEnabled )
    {
        if ( _maprev != rev || setToDirty )
        {
            _mutex.lock();

            if ( _maprev != rev || setToDirty )
            {
                _maprev = rev;

                for( TileTable::iterator i = _tiles.begin(); i != _tiles.end(); ++i )
                {
                    if ( setToDirty )
                    {
                        i->second._tile->refreshAllLayers();
                    }
                }
            }

            _mutex.unlock();
        }
    }
}

void
TileNodeRegistry::setDirty(const GeoExtent& extent,
                           unsigned         minLevel,
                           unsigned         maxLevel,
                           const CreateTileManifest& manifest)
{
    _mutex.lock();
    
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

    _mutex.unlock();
}

void
TileNodeRegistry::add(TileNode* tile)
{
    _mutex.lock();

    // It is possible that a Tile with the same key is already in the registry. 
    // This can happen when a Tile's ancestor gets unloaded, orphaning
    // the registry records for its descendants, but the orphaned record has
    // not yet itself been removed by the Unloader. So we have to check!

    bool recyclingOrphan = false;
    TrackerEntry* se;
    TableEntry* te;

    TileTable::iterator i = _tiles.find(tile->getKey());
    if (i != _tiles.end())
    {
        // found an orphan! Reuse and overwrite it.
        recyclingOrphan = true;
        te = &i->second;
        se = (*te->_trackerptr);
        _tracker.erase(te->_trackerptr); // since we need to move it to the front
        OE_DEBUG << "Reused orphaned tile record " << tile->getKey().str() << std::endl;
    }
    else
    {
        te = &_tiles[tile->getKey()];
        se = new TrackerEntry();
    }

    // init the tracker entry and place it at the front of the tracker:
    se->_tile = tile;
    se->_lastTime = DBL_MAX;
    se->_lastFrame = ~0;
    se->_lastRange = FLT_MAX;
    _tracker.push_front(se);

    // init the table entry:
    te->_tile = tile;
    te->_trackerptr = _tracker.begin();
    
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

        OE_DEBUG << LC << _name 
            << ": tiles=" << _tiles.size()
            << ", notifiers=" << _notifiers.size()
            << std::endl;
    }

    _mutex.unlock();
}

void
TileNodeRegistry::startListeningFor(const TileKey& tileToWaitFor, TileNode* waiter)
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
TileNodeRegistry::stopListeningFor(const TileKey& tileToWaitFor, const TileKey& waiterKey)
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

    for (Tracker::iterator i = _tracker.begin(); i != _tracker.end(); ++i)
    {
        TrackerEntry* e = *i;
        delete e;
    }
    _tracker.clear();
    _tracker.push_front(SENTRY_VALUE);
    _sentryptr = _tracker.begin();

    _notifiers.clear();

    _tilesToUpdate.clear();

    OE_PROFILING_PLOT(PROFILING_REX_TILES, (float)(_tiles.size()));
}

void
TileNodeRegistry::touch(TileNode* tile, osg::NodeVisitor& nv)
{
    ScopedMutexLock lock(_mutex);

    // Find the tracker for this tile and update its timestamp
    TileTable::iterator i = _tiles.find(tile->getKey());
    if (i != _tiles.end())
    {
        TableEntry& e = i->second;
        TrackerEntry* se = (*e._trackerptr);
        se->_lastTime = _clock->getTime();
        se->_lastFrame = _clock->getFrame();

        const osg::BoundingSphere& bs = tile->getBound();
        float range = nv.getDistanceToViewPoint(bs.center(), true) - bs.radius();
        se->_lastRange = osg::minimum(se->_lastRange, range);

        // Move the tracker to the front of the list (ahead of the sentry).
        // Once a cull traversal is complete, all visited tiles will be
        // in front of the sentry, leaving all non-visited tiles behind it.
        _tracker.erase(e._trackerptr);
        _tracker.push_front(se);
        e._trackerptr = _tracker.begin();

        // Does it need an update traversal?
        if (tile->updateRequired())
        {
            _tilesToUpdate.push_back(tile->getKey());
        }
    }
    else
    {
        OE_WARN << LC << "UPDATE FAILED - TILE " << tile->getKey().str() << " not in TILE TABLE!" << std::endl;
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

    // After cull, all visited tiles are in front of the sentry, and all
    // non-visited tiles are behind it. Start at the sentry position and
    // iterate over the non-visited tiles, checking them for deletion.
    Tracker::iterator i = _sentryptr;
    Tracker::iterator tmp;
    for(++i; i != _tracker.end() && count < maxTiles; ++i)
    {
        TrackerEntry* se = *i;

        const TileKey& key = se->_tile->getKey();

        if (se->_tile->getDoNotExpire() == false &&
            se->_lastTime < oldestAllowableTime &&
            se->_lastFrame < oldestAllowableFrame &&
            se->_lastRange > farthestAllowableRange &&
            se->_tile->areSiblingsDormant())
        {
            if (_notifyNeighbors)
            {
                // remove neighbor listeners:
                stopListeningFor(key.createNeighborKey(1, 0), key);
                stopListeningFor(key.createNeighborKey(0, 1), key);
            }

            // back up the iterator so we can safely erase the tracker entry:
            tmp = i;
            --i;

            // put the tile on the output list:
            output.push_back(se->_tile);

            // remove it from the main tile table:
            _tiles.erase(key);

            // remove it from the tracker list:
            _tracker.erase(tmp);
            delete se;

            ++count;
        }
        else
        {
            // reset the range in preparation for the next frame.
            se->_lastRange = FLT_MAX;
        }
    }

    // reset the sentry.
    _tracker.erase(_sentryptr);
    _tracker.push_front(SENTRY_VALUE);
    _sentryptr =_tracker.begin();

    OE_PROFILING_PLOT(PROFILING_REX_TILES, (float)(_tiles.size()));
}

osg::ref_ptr<TileNode>
TileNodeRegistry::get(const TileKey& key) const
{
    ScopedMutexLock lock(_mutex);

    osg::ref_ptr<TileNode> result;

    auto iter = _tiles.find(key);
    if (iter != _tiles.end())
    {
        result = iter->second._tile.get();
    }

    return result;
}
