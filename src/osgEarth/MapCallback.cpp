/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
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

    case MapModelChange::OPEN_LAYER:
        onLayerOpened(change.getLayer());
        break;

    case MapModelChange::CLOSE_LAYER:
        onLayerClosed(change.getLayer());
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
