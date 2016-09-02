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

#define LC "[MapCallback] "

using namespace osgEarth;

void
MapCallback::onMapModelChanged( const MapModelChange& change )
{
    switch( change.getAction() )
    {
    case MapModelChange::ADD_ELEVATION_LAYER: 
        onElevationLayerAdded( change.getElevationLayer(), change.getFirstIndex() );
        break;
    case MapModelChange::ADD_IMAGE_LAYER:
        onImageLayerAdded( change.getImageLayer(), change.getFirstIndex() ); 
        break;
    case MapModelChange::ADD_MASK_LAYER:
        onMaskLayerAdded( change.getMaskLayer() );
        break;
    case MapModelChange::ADD_MODEL_LAYER:
        onModelLayerAdded( change.getModelLayer(), change.getFirstIndex() ); 
        break;
    case MapModelChange::REMOVE_ELEVATION_LAYER:
        onElevationLayerRemoved( change.getElevationLayer(), change.getFirstIndex() ); 
        break;
    case MapModelChange::REMOVE_IMAGE_LAYER:
        onImageLayerRemoved( change.getImageLayer(), change.getFirstIndex() );
        break;
    case MapModelChange::REMOVE_MASK_LAYER:
        onMaskLayerRemoved( change.getMaskLayer() ); 
        break;
    case MapModelChange::REMOVE_MODEL_LAYER:
        onModelLayerRemoved( change.getModelLayer() ); 
        break;
    case MapModelChange::MOVE_ELEVATION_LAYER:
        onElevationLayerMoved( change.getElevationLayer(), change.getFirstIndex(), change.getSecondIndex() ); 
        break;
    case MapModelChange::MOVE_IMAGE_LAYER:
        onImageLayerMoved( change.getImageLayer(), change.getFirstIndex(), change.getSecondIndex() ); 
        break;
    case MapModelChange::MOVE_MODEL_LAYER:
        onModelLayerMoved( change.getModelLayer(), change.getFirstIndex(), change.getSecondIndex() ); 
        break;
    case MapModelChange::UNSPECIFIED: 
        break;
    default: 
        break;
    }
}
