
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
    for (LayerDrawableList::iterator i = _layerList.begin(); i != _layerList.end(); ++i)
    {
        //TODO: review and benchmark list vs. vector vs. unsorted here.
        DrawTileCommands& cmds = i->get()->_tiles;
        std::sort(cmds.begin(), cmds.end());
        total += i->get()->_tiles.size();
    }
    return total;
}

void
TerrainRenderData::reset(
    const Map* map,
    const RenderBindings& bindings,
    LayerDrawableTable& drawables,
    unsigned frameNum,
    osgUtil::CullVisitor* cv)
{
    _bindings = &bindings;

    // Create a new State object to track sampler and uniform settings
    _drawState = new DrawState();
    _drawState->_bindings = &bindings;

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
                        LayerDrawable* ld = addLayerDrawable(layer, cam, drawables);

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
                            addLayerDrawable(layer, cam, drawables);
                        }
                    }
                }
            }
        }
    }

    // Include a "blank" layer for missing data.
    addLayerDrawable(nullptr, cam, drawables);
}

LayerDrawable*
TerrainRenderData::addLayerDrawable(
    const Layer* layer,
    const osg::Camera* camera,
    LayerDrawableTable& drawables)
{
    ScopedMutexLock lock(drawables);

    auto key = std::make_pair(layer, camera);
    osg::ref_ptr<LayerDrawable>& drawable = drawables[key];
    //osg::ref_ptr<LayerDrawable> drawable;
    if (!drawable.valid())
    {
        drawable = new LayerDrawable();

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

    drawable->_drawState = _drawState.get();

    if (layer)
    {
        _layerMap[layer->getUID()] = drawable;
    }
    else
    {
        _layerMap[-1] = drawable;
    }

    return drawable.get();
}
