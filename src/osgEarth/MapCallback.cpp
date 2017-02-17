/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarth/MapCallback>
#include <osgEarth/MapModelChange>
#include <osgEarth/ImageLayer>
#include <osgEarth/ElevationLayer>
#include <osgEarth/ModelLayer>
#include <osgEarth/MaskLayer>
#include <osgEarth/Map>

#define LC "[MapCallback] "

using namespace osgEarth;

void
MapCallback::onMapModelChanged( const MapModelChange& change )
{
    switch( change.getAction() )
    {
    case MapModelChange::ADD_LAYER: 
        onLayerAdded(change.getLayer(), change.getFirstIndex());
        break;

    case MapModelChange::REMOVE_LAYER:
        onLayerRemoved(change.getLayer(), change.getFirstIndex());
        break;

    case MapModelChange::MOVE_LAYER:
        onLayerMoved(change.getLayer(), change.getFirstIndex(), change.getSecondIndex());
        break;

    default: 
        break;
    }
}

void
MapCallback::invokeOnLayerAdded(const Map* map)
{
    LayerVector layers;
    map->getLayers(layers);
    unsigned index = 0;
    if (layers.size() > 0)
    {
        onBeginUpdate();
        for (LayerVector::iterator i = layers.begin(); i != layers.end(); ++i)
            onLayerAdded(i->get(), index++);
        onEndUpdate();
    }
}

void
MapCallback::invokeOnLayerRemoved(const Map* map)
{
    LayerVector layers;
    map->getLayers(layers);
    unsigned index = 0;
    if (layers.size() > 0)
    {
        onBeginUpdate();
        for (LayerVector::iterator i = layers.begin(); i != layers.end(); ++i)
            onLayerRemoved(i->get(), index++);
        onEndUpdate();
    }
}

void
MapCallback::onLayerAdded(Layer* layer, unsigned index)
{
    if (dynamic_cast<ImageLayer*>(layer))
        onImageLayerAdded(static_cast<ImageLayer*>(layer), index);
    else if (dynamic_cast<ElevationLayer*>(layer))
        onElevationLayerAdded(static_cast<ElevationLayer*>(layer), index);
    else if (dynamic_cast<ModelLayer*>(layer))
        onModelLayerAdded(static_cast<ModelLayer*>(layer), index);
    else if (dynamic_cast<MaskLayer*>(layer))
        onMaskLayerAdded(static_cast<MaskLayer*>(layer));
}

void 
MapCallback::onLayerRemoved(Layer* layer, unsigned index)
{
    if (dynamic_cast<ImageLayer*>(layer))
        onImageLayerRemoved(static_cast<ImageLayer*>(layer), index);
    else if (dynamic_cast<ElevationLayer*>(layer))
        onElevationLayerRemoved(static_cast<ElevationLayer*>(layer), index);
    else if (dynamic_cast<ModelLayer*>(layer))
        onModelLayerRemoved(static_cast<ModelLayer*>(layer), index);
    else if (dynamic_cast<MaskLayer*>(layer))
        onMaskLayerRemoved(static_cast<MaskLayer*>(layer));
}

void
MapCallback::onLayerMoved(Layer* layer, unsigned oldIndex, unsigned newIndex)
{
    if (dynamic_cast<ImageLayer*>(layer))
        onImageLayerMoved(static_cast<ImageLayer*>(layer), oldIndex, newIndex);
    else if (dynamic_cast<ElevationLayer*>(layer))
        onElevationLayerMoved(static_cast<ElevationLayer*>(layer), oldIndex, newIndex);
    else if (dynamic_cast<ModelLayer*>(layer))
        onModelLayerMoved(static_cast<ModelLayer*>(layer), oldIndex, newIndex);
}
