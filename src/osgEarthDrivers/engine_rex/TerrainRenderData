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
#ifndef OSGEARTH_REX_TERRAIN_DRAWABLE_H
#define OSGEARTH_REX_TERRAIN_DRAWABLE_H 1

#include "RenderBindings"
#include "DrawState"
#include "LayerDrawable"

using namespace osgEarth;

namespace osgEarth { namespace Drivers { namespace RexTerrainEngine
{
    /**
     * Main data structure assembled by the TerrainCuller that contains
     * everything necessary to render one frame of the terrain.
     */
    class TerrainRenderData
    {
    public:
        TerrainRenderData() :
            _bindings(0L) { }

        /** Set up the map layers before culling the terrain */
        void setup(const Map* map, const RenderBindings& bindings, unsigned frameNum, osgUtil::CullVisitor* cv);

        /** Optimize for best state sharing (when using geometry pooling). Returns total tile count. */
        unsigned sortDrawCommands();

        /** Add a Drawable for a layer. Add these in the order you wish to render them. */
        LayerDrawable* addLayerDrawable(const Layer*);

        /** Layers to draw */
        LayerDrawableList& layers() { return _layerList; }
        const LayerDrawableList& layers() const { return _layerList; }

        /** Look up a LayerDrawable by its source layer UID. */
        osg::ref_ptr<LayerDrawable>& layer(UID uid) { return _layerMap[uid]; }

        // Draw state shared by all layers during one frame.
        osg::ref_ptr<DrawState> _drawState;

        // Layers of type RENDERTYPE_TERRAIN_PATCH
        PatchLayerVector& patchLayers() { return _patchLayers; }
        
    private:

        LayerDrawableList     _layerList;
        LayerDrawableMap      _layerMap;
        const RenderBindings* _bindings;
        PatchLayerVector      _patchLayers;
    };

} } } // namespace 

#endif // OSGEARTH_REX_TERRAIN_DRAWABLE_H
