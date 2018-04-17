/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
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

using namespace osgEarth::Drivers::RexTerrainEngine;

#undef  LC
#define LC "[TerrainRenderData] "


void
TerrainRenderData::sortDrawCommands()
{
    for (LayerDrawableList::iterator i = _layerList.begin(); i != _layerList.end(); ++i)
    {
        i->get()->_tiles.sort();
    }
}

void
TerrainRenderData::setup(const Map* map,
                         const RenderBindings& bindings,
                         unsigned frameNum,
                         osgUtil::CullVisitor* cv)
{
    _bindings = &bindings;

    // Create a new State object to track sampler and uniform settings
    _drawState = new DrawState();
    _drawState->_frame = frameNum;
    _drawState->_bindings = &bindings;
    
    // Is this a depth camera? Because if it is, we don't need any color layers.
    const osg::Camera* cam = cv->getCurrentCamera();
    bool isDepthCamera =
        cam &&
        (cam->getClearMask() & GL_COLOR_BUFFER_BIT) == 0u &&
        (cam->getClearMask() & GL_DEPTH_BUFFER_BIT) != 0u;

    // Make a drawable for each rendering pass (i.e. each render-able map layer).
    LayerVector layers;
    map->getLayers(layers);

    for (LayerVector::const_iterator i = layers.begin(); i != layers.end(); ++i)
    {
        Layer* layer = i->get();
        if (layer->getEnabled())
        {
            bool render =
                (layer->getRenderType() == Layer::RENDERTYPE_TERRAIN_SURFACE) || // && !isDepthCamera) ||
                (layer->getRenderType() == Layer::RENDERTYPE_TERRAIN_PATCH);

            if ( render )
            {
                // If this is an image layer, check the enabled/visible states.
                VisibleLayer* visLayer = dynamic_cast<VisibleLayer*>(layer);
                if (visLayer)
                {
                    render = visLayer->getVisible();
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
                        PatchLayer* patchLayer = static_cast<PatchLayer*>(layer); // asumption!

                        if (patchLayer->getAcceptCallback() != 0L &&
                            patchLayer->getAcceptCallback()->acceptLayer(*cv, cv->getCurrentCamera()))
                        {
                            patchLayers().push_back(dynamic_cast<PatchLayer*>(layer));
                            addLayerDrawable(layer);
                        }
                    }
                }
            }
        }
    }

    // Include a "blank" layer for missing data.
    LayerDrawable* blank = addLayerDrawable(0L);
    blank->getOrCreateStateSet()->setDefine("OE_TERRAIN_RENDER_IMAGERY", osg::StateAttribute::OFF);
}

namespace
{
    struct DebugCallback : public osg::Drawable::DrawCallback
    {
        std::string _s;
        DebugCallback(const std::string& s) : _s(s) { }
        void drawImplementation(osg::RenderInfo& ri, const osg::Drawable* d) const {
            OE_WARN << "  Drawing Layer: " << _s << std::endl;
            d->drawImplementation(ri);
        }

    };
}

LayerDrawable*
TerrainRenderData::addLayerDrawable(const Layer* layer)
{
    UID uid = layer ? layer->getUID() : -1;
    LayerDrawable* ld = new LayerDrawable();
    _layerList.push_back(ld);
    _layerMap[uid] = ld;
    ld->_layer = layer;
    ld->_visibleLayer = dynamic_cast<const VisibleLayer*>(layer);
    ld->_imageLayer = dynamic_cast<const ImageLayer*>(layer);
    ld->_order = _layerList.size() - 1;
    ld->_drawState = _drawState.get();
    if (layer)
    {
        ld->setStateSet(layer->getStateSet());
        ld->_renderType = layer->getRenderType();
    }
    return ld;
}
