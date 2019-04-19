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
#include <osgEarth/MapCallback>
#include <osgEarth/MapModelChange>
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

    case MapModelChange::ENABLE_LAYER:
        onLayerEnabled(change.getLayer());
        break;

    case MapModelChange::DISABLE_LAYER:
        onLayerDisabled(change.getLayer());
        break;

    case MapModelChange::BEGIN_BATCH_UPDATE:
	onBeginUpdate();
	break;

    case MapModelChange::END_BATCH_UPDATE:
	onEndUpdate();
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
