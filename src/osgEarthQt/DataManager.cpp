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
#include <osgEarth/MapNode>

using namespace osgEarth::QtGui;
using namespace osgEarth::Annotation;

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

DataManager::DataManager(osgEarth::Map* map) : _map(map), _maxUndoStackSize( 128 )
{
  if (_map)
  {
    _map->addMapCallback(new DataManagerMapCallback(this));
  }
}

DataManager::DataManager(osgEarth::MapNode* mapNode) : _maxUndoStackSize( 128 )
{
  if (mapNode)
  {
    _map = mapNode->getMap();
    if (_map)
      _map->addMapCallback(new DataManagerMapCallback(this));

    //Look for viewpoints in the MapNode externals
    const Config& externals = mapNode->externalConfig();
    const ConfigSet children = externals.children("viewpoint");
    if (children.size() > 0)
    {
      for( ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i )
        _viewpoints.push_back(Viewpoint(*i));
    }
  }
}

void DataManager::addAnnotation(osgEarth::Annotation::AnnotationNode* annotation, osg::Group* parent)
{
  if (!annotation || !parent)
    return;

  //osg::Node* root = parent ? parent : 

  if (parent->addChild(annotation))
  {
    {
      Threading::ScopedWriteLock lock( _dataMutex );
      _annotations.push_back(annotation);
    }

    emit annotationAdded(annotation);
    emit mapChanged();
  }
}

void DataManager::removeAnnotaton(osgEarth::Annotation::AnnotationNode* annotation, osg::Group* parent)
{
  if (!annotation)
    return;

  osg::ref_ptr<osgEarth::Annotation::AnnotationNode> annoToRemove = annotation;

  bool removed = false;
  if (parent)
  {
    removed = parent->removeChild(annotation);
  }
  else
  {
    osg::Node::ParentList p = annotation->getParents();
    for (osg::Node::ParentList::iterator it = p.begin(); it != p.end(); ++it)
    {
      if ((*it)->removeChild(annotation))
        removed = true;
    }
  }

  Threading::ScopedWriteLock lock( _dataMutex );
  for(AnnotationVector::iterator it = _annotations.begin(); it != _annotations.end(); ++it)
  {
    if (it->get() == annoToRemove.get())
    {
      _annotations.erase(it);
      removed = true;
      break;
    }
  }

  if (removed)
  {
    emit annotationRemoved(annoToRemove);
    emit mapChanged();
  }
}

void DataManager::getAnnotations(AnnotationVector& out_annotations) const
{
  out_annotations.reserve(_annotations.size());

  Threading::ScopedReadLock lock(const_cast<DataManager*>(this)->_dataMutex);
  for(AnnotationVector::const_iterator it = _annotations.begin(); it != _annotations.end(); ++it)
    out_annotations.push_back(it->get());
}

void DataManager::getViewpoints(std::vector<osgEarth::Viewpoint>& out_viewpoints) const
{
  out_viewpoints.reserve(_viewpoints.size());

  Threading::ScopedReadLock lock(const_cast<DataManager*>(this)->_dataMutex);
  for(std::vector<osgEarth::Viewpoint>::const_iterator it = _viewpoints.begin(); it != _viewpoints.end(); ++it)
    out_viewpoints.push_back(*it);
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

//---------------------------------------------------------------------------
// ActionManager

void DataManager::addBeforeActionCallback( ActionCallback* cb )
{
    _beforeCallbacks.push_back( cb );
}

void DataManager::addAfterActionCallback( ActionCallback* cb )
{
    _afterCallbacks.push_back( cb );
}

bool DataManager::doAction( void* sender, Action* action_, bool reversible )
{
    // this ensures that the action will be unref'd and deleted after running
    osg::ref_ptr<Action> action = action_;

    bool undoInProgress = sender == this;

    for( ActionCallbackList::iterator i = _beforeCallbacks.begin(); i != _beforeCallbacks.end(); ++i )
        i->get()->operator()( sender, action.get() );

    bool actionSucceeded = false;

    if ( !action->isCanceled() || undoInProgress )
    {
        actionSucceeded = action->doAction( sender, this );

        if ( !undoInProgress && actionSucceeded )
        {
            if ( action->isCheckpoint() )
            {
                clearUndoActions();
            }
            else if ( reversible && action->isReversible() )
            {
                _undoStack.push_back( action.get() );
                if ( (int)_undoStack.size() > _maxUndoStackSize )
                {
                    _undoStack.pop_front();
                }
            }
        }

        //todo: during-action callbacks here? like in pogo?

        for( ActionCallbackList::iterator j = _afterCallbacks.begin(); j != _afterCallbacks.end(); ++j )
            j->get()->operator()( sender, action.get() );
    }

    return actionSucceeded;
}

bool DataManager::undoAction()
{
    if ( !canUndo() )
        return false;

		osg::ref_ptr<ReversibleAction> action = static_cast<ReversibleAction*>( _undoStack.back().get() );
		_undoStack.pop_back();

		bool undoSucceeded = action->undoAction( this, this );

		// if the undo failed, we are probably in some undefined application state, so
    // clear out the undo stack just to be safe.
    if ( !undoSucceeded )
    {
        clearUndoActions();
    }

		return undoSucceeded;
}

bool DataManager::canUndo() const
{
    return _undoStack.size() > 0;
}

void DataManager::clearUndoActions()
{
    _undoStack.clear();
}

ReversibleAction* DataManager::getNextUndoAction() const
{
    return _undoStack.size() > 0 ? static_cast<ReversibleAction*>( _undoStack.front().get() ): 0L;
}