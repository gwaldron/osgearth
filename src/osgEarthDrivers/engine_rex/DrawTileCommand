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
#ifndef OSGEARTH_REX_TERRAIN_DRAW_TILE_COMMAND_H
#define OSGEARTH_REX_TERRAIN_DRAW_TILE_COMMAND_H 1

#include "TileRenderModel"
#include "DrawState"
#include "GeometryPool"
#include <osgEarth/PatchLayer>
#include <osgEarth/TileKey>
#include <osg/Matrix>
#include <osg/Geometry>
#include <list>

using namespace osgEarth;

namespace osgEarth { namespace Drivers { namespace RexTerrainEngine
{
    /**
     * All data necessary to draw a single terrain tile.
     */
    struct DrawTileCommand
    {
        // ModelView matrix to apply before rendering this tile
        //osg::Matrixf _modelViewMatrix;
        osg::ref_ptr<const osg::RefMatrix> _modelViewMatrix;

        // Samplers that are shared between all rendering passes
        const Samplers* _sharedSamplers;

        // Samplers specific to one rendering pass
        const Samplers* _colorSamplers;

        // Tile geometry, if present
        osg::ref_ptr<SharedGeometry> _geom;

        // Tile key
        const TileKey* _key;

        // Tile key value to push to uniform just before drawing
        osg::Vec4f _keyValue;

        // Elevation texel sampling coefficients (so we sample elevation
        // data on center rather than on edge) - We might be able to move
        // this to the Layer level as long as the elevation texture size
        // doesn't change
        osg::Vec2f _elevTexelCoeff;

        // Coefficient used for tile vertex morphing
        osg::Vec2f _morphConstants;

        // Custom draw callback to call instead of rendering _geom
        PatchLayer::DrawCallback* _drawCallback;

        // When drawing _geom, whether to render as GL_PATCHES 
        // instead of GL_TRIANGLES (for patch layers)
        bool _drawPatch;

        // Distance from camera to center of tile
        float _range;

        // Layer order for this tile. i.e., if this is zero, then this command
        // is drawing a tile for the first LayerDrawable.
        int _layerOrder;

        // Renders the tile.
        void draw(osg::RenderInfo& ri, DrawState& ds, osg::Referenced* layerData) const;

        // Less than operator will ensure that tiles are sorted high-to-low LOD
        // (to minimize Z overdraw) and then grouped by shared geometry
        // (to minimize buffer binds). Both of these make a significant performance
        // difference based on benchmarking.
        bool operator < (const DrawTileCommand& rhs) const
        {
            if (_key->getLOD() > rhs._key->getLOD()) return true;
            if (_key->getLOD() < rhs._key->getLOD()) return false;
            return _geom < rhs._geom;
        }

        DrawTileCommand() :
            _sharedSamplers(0L),
            _colorSamplers(0L),
            _geom(0L),
            _elevTexelCoeff(1.0f, 0.0f),
            _drawCallback(0L),
            _drawPatch(false),
            _range(0.0f),
            _layerOrder(INT_MAX){ }
    };

    /**
     * Ordered list of tile drawing commands.
     * List is faster than vector here (benchmarked)
     */
    //typedef std::list<DrawTileCommand> DrawTileCommands;
    typedef std::vector<DrawTileCommand> DrawTileCommands;

} } } // namespace 

#endif // OSGEARTH_REX_TERRAIN_DRAW_TILE_COMMAND_H
