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
TerrainRenderData::setup(const MapFrame& frame,
                         const RenderBindings& bindings,
                         unsigned frameNum,
                         osgUtil::CullVisitor* cv)
{
    _bindings = &bindings;

    // Create a new State object to track sampler and uniform settings
    _drawState = new DrawState();
    _drawState->_frame = frameNum;
    _drawState->_bindings = &bindings;

    // Make a drawable for each rendering pass (i.e. each render-able map layer).
    for(LayerVector::const_iterator i = frame.layers().begin();
        i != frame.layers().end();
        ++i)
    {
        Layer* layer = i->get();
        if (layer->getEnabled())
        {
            if (layer->getRenderType() == Layer::RENDERTYPE_TILE ||
                layer->getRenderType() == Layer::RENDERTYPE_PATCH)
            {
                bool render = true;

                // If this is an image layer, check the enabled/visible states.
                VisibleLayer* visLayer = dynamic_cast<VisibleLayer*>(layer);
                if (visLayer)
                {
                    render = visLayer->getVisible();
                }

                if (render)
                {
                    if (layer->getRenderType() == Layer::RENDERTYPE_PATCH)
                    {
                        PatchLayer* patchLayer = static_cast<PatchLayer*>(layer); // asumption!

                        if (patchLayer->getAcceptCallback() != 0L &&
                            patchLayer->getAcceptCallback()->acceptLayer(*cv, cv->getCurrentCamera()))
                        {
                            patchLayers().push_back(dynamic_cast<PatchLayer*>(layer));
                            addLayerDrawable(layer, frame);
                        }
                    }
                    else
                    {
                        addLayerDrawable(layer, frame);
                    }
                }
            }
        }
    }

    // Include a "blank" layer for missing data.
    LayerDrawable* blank = addLayerDrawable(0L, frame);
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
TerrainRenderData::addLayerDrawable(const Layer* layer, const MapFrame& frame)
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
    //ld->setDrawCallback(new DebugCallback(layer ? layer->getName() : "[blank]"));
    return ld;
}
