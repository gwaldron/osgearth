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
#include <osgEarthQt/Actions>
//#include <osgEarthQt/DataManager>

#include <osgEarth/Map>

using namespace osgEarth::QtGui;



//---------------------------------------------------------------------------

class ActionManagerImpl : public ActionManager // no export
{
public:
    ActionManagerImpl( Application* app );

public: // ActionManager interface

    void addBeforeActionCallback( ActionCallback* cb );
    void addAfterActionCallback( ActionCallback* cb );
    bool doAction( void* sender, Action* action, bool reversible =true );
    bool undoAction();
    bool canUndo() const;
    void clearUndoActions();
    ReversibleAction* getNextUndoAction() const;

private:
    osg::ref_ptr<Application> _app;
    std::list< osg::ref_ptr<Action> > _undoStack;

    typedef std::list< osg::ref_ptr<ActionCallback> > ActionCallbackList;
    ActionCallbackList _beforeCallbacks;
    ActionCallbackList _afterCallbacks;
    int _maxUndoStackSize;
};

ActionManagerImpl::ActionManagerImpl( DataManager* app ) :
_app( app ),
_maxUndoStackSize( 128 ) // arbitrary
{
    //nop
}

void
ActionManagerImpl::addBeforeActionCallback( ActionCallback* cb )
{
    _beforeCallbacks.push_back( cb );
}

void
ActionManagerImpl::addAfterActionCallback( ActionCallback* cb )
{
    _afterCallbacks.push_back( cb );
}

bool
ActionManagerImpl::doAction( void* sender, Action* action_, bool reversible )
{
    // this ensures that the action will be unref'd and deleted after running
    osg::ref_ptr<Action> action = action_;

    bool undoInProgress = sender == this;

    for( ActionCallbackList::iterator i = _beforeCallbacks.begin(); i != _beforeCallbacks.end(); ++i )
        i->get()->operator()( sender, action.get() );

    bool actionSucceeded = false;

    if ( !action->isCanceled() || undoInProgress )
    {
        actionSucceeded = action->doAction( sender, _app.get() );

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

bool
ActionManagerImpl::undoAction()
{
    if ( !canUndo() )
        return false;

		osg::ref_ptr<ReversibleAction> action = static_cast<ReversibleAction*>( _undoStack.back().get() );
		_undoStack.pop_back();

		bool undoSucceeded = action->undoAction( this, _app.get() );

		// if the undo failed, we are probably in some undefined application state, so
    // clear out the undo stack just to be safe.
    if ( !undoSucceeded )
    {
        clearUndoActions();
    }

		return undoSucceeded;
}

bool
ActionManagerImpl::canUndo() const
{
    return _undoStack.size() > 0;
}

void
ActionManagerImpl::clearUndoActions()
{
    _undoStack.clear();
}

ReversibleAction*
ActionManagerImpl::getNextUndoAction() const
{
    return _undoStack.size() > 0 ? static_cast<ReversibleAction*>( _undoStack.front().get() ): 0L;
}
