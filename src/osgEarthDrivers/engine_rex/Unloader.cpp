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
#include "Unloader"
#include "TileNode"
#include "TileNodeRegistry"

#include <osgEarth/Metrics>
#include <osgEarth/NodeUtils>

#undef  LC
#define LC "[UnloaderGroup] "

using namespace osgEarth::REX;


UnloaderGroup::UnloaderGroup(TileNodeRegistry* tiles) :
_tiles(tiles),
_cacheSize(0u),
_maxAge(0.1),
_maxTilesToUnloadPerFrame(~0)
{
    ADJUST_EVENT_TRAV_COUNT(this, +1);
}

void
UnloaderGroup::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.EVENT_VISITOR )
    {
        OE_PROFILING_ZONE_NAMED("Expire Tiles");

        osg::Timer_t now = nv.getFrameStamp()->getReferenceTime();

        unsigned count = 0u;

        // Have to enforce both the time delay AND a frame delay since the frames can
        // stop while the time rolls on (e.g., if you are dragging the window)
        double olderThanTime = now - _maxAge;
        unsigned olderThanFrame = osg::maximum(nv.getFrameStamp()->getFrameNumber(), 3u) - 3u;

        // Remove them from the registry:
        _tiles->collectTheDead(olderThanTime, olderThanFrame, _maxTilesToUnloadPerFrame, _deadpool);

        // Remove them from the scene graph:
        for(std::vector<osg::observer_ptr<TileNode> >::iterator i = _deadpool.begin();
            i != _deadpool.end();
            ++i)
        {
            // may be NULL since we're removing scene graph objects as we go!
            osg::ref_ptr<TileNode> tile = i->get();

            if (tile.valid() && tile->getNumParents() > 0)
            {
                TileNode* parent = dynamic_cast<TileNode*>(tile->getParent(0));
                if (parent) // && parent->areSubTilesDormant(nv.getFrameStamp()))
                {
                    parent->removeSubTiles();
                    ++count;
                }
            }
        }

        if (_deadpool.empty() == false)
        {
            OE_DEBUG << "Unloaded " << count << " of " << _deadpool.size() << " expired tiles; " << _tiles->size() << " remain active." << std::endl;
        }

        _deadpool.clear();
    }

    osg::Group::traverse( nv );
}

