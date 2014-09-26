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
#include <osgEarthQt/DataManager>

#include <osgEarth/Map>
#include <osgEarth/MapModelChange>
#include <osgEarth/MapNode>

using namespace osgEarth::QtGui;
using namespace osgEarth::Annotation;


DataManager::DataManager(osgEarth::MapNode* mapNode) : _mapNode(mapNode), _maxUndoStackSize( 128 )
{
  if (_mapNode)
  {
    _map = _mapNode->getMap();

    //Look for viewpoints in the MapNode externals
    const Config& externals = _mapNode->externalConfig();
    const ConfigSet children = externals.children("viewpoint");
    if (children.size() > 0)
    {
      for( ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i )
        _viewpoints.push_back(Viewpoint(*i));
    }
  }

  initialize();
}

void DataManager::initialize()
{
  _selectedDecoration = "selected";

  _elevationCallback = new DataManagerElevationLayerCallback(this);
  _imageCallback = new DataManagerImageLayerCallback(this);
  _modelCallback = new DataManagerModelLayerCallback(this);

  if (_map)
  {
    osgEarth::ElevationLayerVector elevLayers;
    _map->getElevationLayers(elevLayers);
    for (osgEarth::ElevationLayerVector::const_iterator it = elevLayers.begin(); it != elevLayers.end(); ++it)
      (*it)->addCallback(_elevationCallback);

    osgEarth::ImageLayerVector imageLayers;
    _map->getImageLayers(imageLayers);
    for (osgEarth::ImageLayerVector::const_iterator it = imageLayers.begin(); it != imageLayers.end(); ++it)
      (*it)->addCallback(_imageCallback);

    osgEarth::ModelLayerVector modelLayers;
    _map->getModelLayers(modelLayers);
    for (osgEarth::ModelLayerVector::const_iterator it = modelLayers.begin(); it != modelLayers.end(); ++it)
      (*it)->addCallback(_modelCallback);

    _map->addMapCallback(new DataManagerMapCallback(this));
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

void DataManager::removeAnnotation(osgEarth::Annotation::AnnotationNode* annotation, osg::Group* parent)
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

  {
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

void DataManager::addSelectedAnnotation(osgEarth::Annotation::AnnotationNode* annotation)
{
  bool added = false;

  if (annotation)
  {
    Threading::ScopedWriteLock lock(const_cast<DataManager*>(this)->_dataMutex);

    added = std::find(_selection.begin(), _selection.end(), annotation) == _selection.end();

    if (added)
    {
      annotation->setDecoration(_selectedDecoration);
      _selection.push_back(annotation);
    }
  }

  if (added)
    emit selectionChanged();
}

void DataManager::removeSelectedAnnotation(osgEarth::Annotation::AnnotationNode* annotation)
{
  bool removed = false;

  if (annotation)
  {
    Threading::ScopedWriteLock lock(const_cast<DataManager*>(this)->_dataMutex);

    AnnotationVector::iterator found = std::find(_selection.begin(), _selection.end(), annotation);
    if (found != _selection.end())
    {
      annotation->clearDecoration();
      _selection.erase(found);
      removed = true;
    }
  }

  if (removed)
    emit selectionChanged();
}

void DataManager::setSelectedAnnotations(const AnnotationVector& annotations)
{
  if (_selection.size() == 0 && annotations.size() == 0)
    return;

  clearSelectedAnnotations();

  {
    Threading::ScopedWriteLock lock(const_cast<DataManager*>(this)->_dataMutex);

    for (AnnotationVector::const_iterator itNew = annotations.begin(); itNew != annotations.end(); ++itNew)
    {
      (*itNew)->setDecoration(_selectedDecoration);
      _selection.push_back(*itNew);
    }
  }

  emit selectionChanged();
}

void DataManager::clearSelectedAnnotations()
{
  if (_selection.size() == 0)
    return;

  {
    Threading::ScopedWriteLock lock(const_cast<DataManager*>(this)->_dataMutex);

    for (AnnotationVector::iterator itOld = _selection.begin(); itOld != _selection.end(); ++itOld)
      (*itOld)->clearDecoration();

    _selection.clear();
  }

  emit selectionChanged();
}

bool DataManager::isSelected(osgEarth::Annotation::AnnotationNode* annotation)
{
  if (!annotation)
    return false;

  Threading::ScopedReadLock lock(const_cast<DataManager*>(this)->_dataMutex);
  return std::find(_selection.begin(), _selection.end(), annotation) != _selection.end();
}

void DataManager::getViewpoints(std::vector<osgEarth::Viewpoint>& out_viewpoints) const
{
  out_viewpoints.reserve(_viewpoints.size());

  Threading::ScopedReadLock lock(const_cast<DataManager*>(this)->_dataMutex);
  for(std::vector<osgEarth::Viewpoint>::const_iterator it = _viewpoints.begin(); it != _viewpoints.end(); ++it)
    out_viewpoints.push_back(*it);
}

void DataManager::onMapChanged()
{
  emit mapChanged();
}

void DataManager::onMapChanged(const osgEarth::MapModelChange& change)
{
  switch( change.getAction() )
  {
  case MapModelChange::ADD_ELEVATION_LAYER: 
    change.getElevationLayer()->addCallback(_elevationCallback); break;
  case MapModelChange::ADD_IMAGE_LAYER:
    change.getImageLayer()->addCallback(_imageCallback); break;
  case MapModelChange::ADD_MODEL_LAYER:
		change.getModelLayer()->addCallback(_modelCallback); break;
  case MapModelChange::REMOVE_ELEVATION_LAYER:
    change.getElevationLayer()->removeCallback(_elevationCallback); break;
  case MapModelChange::REMOVE_IMAGE_LAYER:
    change.getImageLayer()->removeCallback(_imageCallback); break;
  case MapModelChange::REMOVE_MODEL_LAYER:
    change.getModelLayer()->removeCallback(_modelCallback); break;
  default: break;
  }

  onMapChanged();
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

    if ( actionSucceeded )
    {
        ViewVector& views = action_->getViews();
        for( ViewVector::iterator i = views.begin(); i != views.end(); ++i )
            i->get()->requestRedraw();
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

    if ( undoSucceeded )
    {
        ViewVector& views = action->getViews();
        for( ViewVector::iterator i = views.begin(); i != views.end(); ++i )
            i->get()->requestRedraw();
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
