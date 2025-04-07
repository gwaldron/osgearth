/* osgEarth
 * Copyright 2008-2014 Pelican Mapping
 * MIT License
 */
#include "Unloader"
#include "TileNode"
#include "TileNodeRegistry"

#include <osgEarth/Metrics>
#include <osgEarth/NodeUtils>

#undef  LC
#define LC "[UnloaderGroup] "

using namespace osgEarth::REX;


UnloaderGroup::UnloaderGroup(TileNodeRegistry* tiles, const TerrainOptionsAPI& api) :
    _options(api),
    _tiles(tiles),
    //_minResidentTiles(0u),
    //_maxAge(0.1),
    //_minRange(0.0f),
    //_maxTilesToUnloadPerFrame(~0),
    _frameLastUpdated(0u)
{
    ADJUST_UPDATE_TRAV_COUNT(this, +1);
}

void
UnloaderGroup::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.UPDATE_VISITOR )
    {
        unsigned frame = _clock->getFrame();
        bool runUpdate = (_frameLastUpdated < frame);

        if (runUpdate && _tiles->size() > _options.getMinResidentTiles())
        {
            _frameLastUpdated = frame;

            OE_PROFILING_ZONE_NAMED("Expire Tiles");

            double now = _clock->getTime();

            unsigned count = 0u;

            // Have to enforce both the time delay AND a frame delay since the frames can
            // stop while the time rolls on (e.g., if you are dragging the window)
            double oldestAllowableTime = now - _options.getMinExpiryTime();
            unsigned oldestAllowableFrame = osg::maximum(frame, 3u) - 3u;

            // Remove them from the registry:
            _tiles->collectDormantTiles(
                nv,
                oldestAllowableTime,
                oldestAllowableFrame,
                _options.getMinExpiryRange(),
                _options.getMaxTilesToUnloadPerFrame(),
                _deadpool);

            // Remove them from the scene graph:
            for(auto& tile_weakptr : _deadpool)
            {
                // may be NULL since we're removing scene graph objects as we go!
                osg::ref_ptr<TileNode> tile;
                if (!tile_weakptr.lock(tile))
                    continue;

                if (tile.valid())
                {
                    osg::ref_ptr<TileNode> parent;
                    tile->getParentTile(parent);

                    // Check that this tile doesn't have any live quadtree siblings. If it does,
                    // we don't want to remove them too!
                    // GW: moved this check to the collectAbandonedTiles function where it belongs
                    if (parent.valid())
                    {
                        parent->removeSubTiles();
                        ++count;
                    }
                }
            }

            if (_deadpool.empty() == false)
            {
                //OE_DEBUG << LC << "Unloaded " << count << " of " << _deadpool.size() << " dormant tiles; " << _tiles->size() << " remain active." << std::endl;
            }

            _deadpool.clear();
        }
    }

    osg::Group::traverse( nv );
}

