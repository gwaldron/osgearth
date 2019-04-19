/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarth/TileKey>
#include <osgEarth/TerrainTileModel>
#include <osg/MatrixTransform>
#include <osg/Texture>

namespace osgEarth
{
    /**
     * Base class for a terrain engine's representation of a tile.
     * This is largely for internal use and subject to change, so
     * be careful relying on the structure of this object.
     */
    class OSGEARTH_EXPORT TerrainTileNode
    {
    public:    
        //! TileKey represented by this tile node
        virtual const TileKey& getKey() const = 0;

        virtual double getMinimumExpirationTime() const = 0;
        virtual void setMinimumExpirationTime(double minExpiryTime) = 0;
        
        virtual unsigned int getMinimumExpirationFrames() const = 0;
        virtual void setMinimumExpirationFrames(unsigned int minExpiryFrames) = 0;


        virtual void loadChildren() = 0;


    protected:
        virtual ~TerrainTileNode() { }
    };
};

#endif // OSGEARTH_TILE_NODE_H
