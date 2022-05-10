/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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

#ifndef OSGEARTH_TERRAIN_TILE_NODE_H
#define OSGEARTH_TERRAIN_TILE_NODE_H 1

#include <osgEarth/Common>
#include <vector>

namespace osg {
    class RenderInfo;
}

namespace osgEarth
{
    class TileKey;

    /**
     * Base class for a terrain engine's representation of a tile.
     * This is largely for internal use and subject to change, so
     * be careful relying on the structure of this object.
     */
    class TerrainTile
    {
    public:    
        //! TileKey represented by this tile node
        virtual const TileKey& getKey() const = 0;
    };

    //! Synonym
    using TerrainTileNode = TerrainTile;
};

#endif // OSGEARTH_TILE_NODE_H
