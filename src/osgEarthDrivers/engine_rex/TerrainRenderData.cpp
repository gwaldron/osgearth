
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
#include "TerrainRenderData"
#include "TileNode"
#include "SurfaceNode"
#include <osgEarth/CameraUtils>

using namespace osgEarth::REX;

#undef  LC
#define LC "[TerrainRenderData] "


unsigned
TerrainRenderData::sortDrawCommands()
{
    unsigned total = 0;
    for(auto layer : _layerList)
    {
        //TODO: review and benchmark list vs. vector vs. unsorted here.
        DrawTileCommands& tiles = layer->_tiles;
        std::sort(tiles.begin(), tiles.end());
        total += tiles.size();
    }
    return total;
}

void
TerrainRenderData::reset(
    const Map* map,
    const RenderBindings& bindings,
    unsigned frameNum,
    PersistentData& persistent,
    bool useGL4Rendering,
    osgUtil::CullVisitor* cv,
    EngineContext* context)
{
    _bindings = &bindings;
    _useGL4Rendering = useGL4Rendering;
    _persistent = &persistent;
    _context = context;

    // Create a new State object to track sampler and uniform settings
    _drawState = DrawState::create();
    _drawState->_bindings = &bindings;

    _layersByUID.clear();
    _layerList.clear();
    _patchLayers.clear();

    // Is this a depth camera? Because if it is, we don't need any color layers.
    const osg::Camera* cam = cv->getCurrentCamera();
    bool isDepthCamera = CameraUtils::isDepthCamera(cam);

    // Make a drawable for each rendering pass (i.e. each render-able map layer).
    LayerVector layers;
    map->getLayers(layers);

    for (auto& layer : layers)
    {
        if (layer->isOpen())
        {
            bool render =
                (layer->getRenderType() == Layer::RENDERTYPE_TERRAIN_SURFACE) ||
                (layer->getRenderType() == Layer::RENDERTYPE_TERRAIN_PATCH);

            if ( render )
            {
                // If this is an image layer, check the enabled/visible states.
                VisibleLayer* visLayer = dynamic_cast<VisibleLayer*>(layer.get());
                if (visLayer)
                {
                    // Check the visibility flag as well as the cull mask
                    render = visLayer->getVisible() && ((cv->getTraversalMask() & visLayer->getMask()) != 0);
                }

                if (render)
                {
                    if (layer->getRenderType() == Layer::RENDERTYPE_TERRAIN_SURFACE)
                    {
                        LayerDrawable* ld = addLayerDrawable(layer);

                        // If the current camera is depth-only, leave this layer in the set
                        // but mark it as no-draw. We keep it in the set so the culler doesn't
                        // inadvertently think it's an orphaned layer.
                        if (isDepthCamera)
                        {
                            ld->_draw = false;
                        }
                    }

                    else // if (layer->getRenderType() == Layer::RENDERTYPE_TERRAIN_PATCH)
                    {
                        PatchLayer* patchLayer = static_cast<PatchLayer*>(layer.get()); // asumption!

                        if (patchLayer->getAcceptCallback() != nullptr &&
                            patchLayer->getAcceptCallback()->acceptLayer(*cv, cv->getCurrentCamera()))
                        {
                            patchLayers().push_back(dynamic_cast<PatchLayer*>(layer.get()));
                            addLayerDrawable(layer);
                        }
                    }
                }
            }
        }
    }

    // Include a "blank" layer for missing data.
    addLayerDrawable(nullptr);
}

LayerDrawable*
TerrainRenderData::addLayerDrawable(
    const Layer* layer)
{
    auto& drawable = _persistent->_drawables[layer];

    if (!drawable.valid())
    {
        drawable = new LayerDrawable();
        drawable->_useIndirectRendering = _useGL4Rendering;
        drawable->_context = _context;

        drawable->_layer = layer;
        drawable->_visibleLayer = dynamic_cast<const VisibleLayer*>(layer);
        drawable->_imageLayer = dynamic_cast<const ImageLayer*>(layer);
        drawable->_patchLayer = dynamic_cast<const PatchLayer*>(layer);

        if (layer)
        {
            drawable->setName(layer->getName());
            drawable->setStateSet(layer->getStateSet());
            drawable->_renderType = layer->getRenderType();
        }
    }

    // reset state:
    drawable->_tiles.clear();
    drawable->_clearOsgState = false;

    drawable->_drawOrder = _layerList.size();
    _layerList.push_back(drawable);

    drawable->_drawState = _drawState;

    drawable->dirtyBound();
    
    if (layer)
    {
        _layersByUID[layer->getUID()] = drawable;
    }
    else
    {
        _layersByUID[-1] = drawable;
    }

    return drawable.get();
}
