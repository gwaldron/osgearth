/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthQt/DataManager>

#include <osgEarth/Map>

using namespace osgEarth::QtGui;

namespace
{
  struct DataManagerMapCallback : public osgEarth::MapCallback
  {
    DataManagerMapCallback(DataManager* dm) : _dm(dm) { }

    void onMapModelChanged( const MapModelChange& change )
    {
      _dm->onMapChanged(change);
    }

    osg::observer_ptr<DataManager> _dm;
  };
}

DataManager::DataManager(osgEarth::Map* map) : _map(map)
{
  if (_map)
  {
    _map->addMapCallback(new DataManagerMapCallback(this));
  }
}

void DataManager::onMapChanged(const osgEarth::MapModelChange& change)
{
  switch( change.getAction() )
  {
  case MapModelChange::ADD_ELEVATION_LAYER: 
    emit elevationLayerAdded( change.getElevationLayer(), change.getFirstIndex() ); break;
  case MapModelChange::ADD_IMAGE_LAYER:
    emit imageLayerAdded( change.getImageLayer(), change.getFirstIndex() ); break;
  case MapModelChange::ADD_MASK_LAYER:
    emit maskLayerAdded( change.getMaskLayer() ); break;
  case MapModelChange::ADD_MODEL_LAYER:
		emit modelLayerAdded( change.getModelLayer(), change.getFirstIndex() ); break;
  case MapModelChange::REMOVE_ELEVATION_LAYER:
    emit elevationLayerRemoved( change.getElevationLayer(), change.getFirstIndex() ); break;
  case MapModelChange::REMOVE_IMAGE_LAYER:
    emit imageLayerRemoved( change.getImageLayer(), change.getFirstIndex() ); break;
  case MapModelChange::REMOVE_MASK_LAYER:
    emit maskLayerRemoved( change.getMaskLayer() ); break;
  case MapModelChange::REMOVE_MODEL_LAYER:
    emit modelLayerRemoved( change.getModelLayer() ); break;
  case MapModelChange::MOVE_ELEVATION_LAYER:
    emit elevationLayerMoved( change.getElevationLayer(), change.getFirstIndex(), change.getSecondIndex() ); break;
  case MapModelChange::MOVE_IMAGE_LAYER:
    emit imageLayerMoved( change.getImageLayer(), change.getFirstIndex(), change.getSecondIndex() ); break;
	case MapModelChange::MOVE_MODEL_LAYER:
		emit modelLayerMoved( change.getModelLayer(), change.getFirstIndex(), change.getSecondIndex() ); break;
  }

  emit mapChanged();
}