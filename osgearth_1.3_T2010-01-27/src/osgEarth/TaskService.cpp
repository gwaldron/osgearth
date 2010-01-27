/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarth/TaskService>
#include <osg/Notify>

using namespace osgEarth;
using namespace OpenThreads;

/**************************************************************************/

TaskRequest::TaskRequest( float priority ) :
osg::Referenced( true ),
_priority( priority ),
_state( STATE_IDLE )
{
    _progress = new ProgressCallback();
}

void
TaskRequest::run()
{
    if ( _state == STATE_IN_PROGRESS )
    {
        (*this)( _progress.get() );
    }
    else
    {
        _progress->cancel();
    }
}

void 
TaskRequest::cancel()
{
    _progress->cancel();
}

bool
TaskRequest::wasCanceled() const
{
    return _progress->isCanceled();
}

/**************************************************************************/

TaskRequestQueue::TaskRequestQueue() :
osg::Referenced( true ),
_done( false )
{
}

void
TaskRequestQueue::clear()
{
    ScopedLock<Mutex> lock(_mutex);
    _requests.clear();
}

void 
TaskRequestQueue::add( TaskRequest* request )
{
    ScopedLock<Mutex> lock(_mutex);

    request->setState( TaskRequest::STATE_PENDING );

    // install a progress callback if one isn't already installed
    if ( !request->getProgressCallback() )
        request->setProgressCallback( new ProgressCallback() );

    // insert by priority.
    bool inserted = false;
    for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); i++ )
    {
        if ( request->getPriority() > i->get()->getPriority() )
        {
            _requests.insert( i, request );
            inserted = true;
            //osg::notify(osg::NOTICE) << "TaskRequestQueue size=" << _requests.size() << std::endl;
            break;
        }
    }

    if ( !inserted )
        _requests.push_back( request );

    // since there is data in the queue, wake up one waiting task thread.
    _cond.signal();
}

TaskRequest* 
TaskRequestQueue::get()
{
    ScopedLock<Mutex> lock(_mutex);

    while ( !_done && _requests.empty() )
    {
        // releases the mutex and waits on the condition.
        _cond.wait( &_mutex );
    }

    if ( _done )
    {
        return 0L;
    }

    osg::ref_ptr<TaskRequest> next = _requests.front();
    _requests.pop_front();

    // I'm done, someone else take a turn:
    // (technically this shouldn't be necessary since add() bumps the semaphore once
    // for each request in the queue)
    _cond.signal();

    return next.release();
}

void
TaskRequestQueue::setDone()
{
    // we need to obtain the mutex since we're using the Condition
    ScopedLock<Mutex> lock(_mutex);

    _done = true;

    // wake everyone up so they can see the _done flag set and exit.
    //_cond.broadcast();

    // alternative to buggy win32 broadcast:
    for(int i=0; i<128; i++)
        _cond.signal();
}

/**************************************************************************/

TaskThread::TaskThread( TaskRequestQueue* queue ) :
_queue( queue ),
_done( false )
{
    //nop
}

void
TaskThread::run()
{
    while( !_done )
    {
        _request = _queue->get();

        if ( _done )
            break;

        if (_request.valid())
        { 
            // discard a completed or canceled request:
            if ( _request->getState() != TaskRequest::STATE_PENDING )
            {
                _request->cancel();
            }

            else if ( !_request->wasCanceled() )
            {
                _request->setState( TaskRequest::STATE_IN_PROGRESS );
                _request->run();
            }
            
            _request->setState( TaskRequest::STATE_COMPLETED );

            // Release the request
            _request = 0;
        }
    }
}

int
TaskThread::cancel()
{
    if ( isRunning() )
    {
        _done = true;  

        if (_request.valid())
        {
            _request->cancel();
        }

        while( isRunning() )
        {        
            OpenThreads::Thread::YieldCurrentThread();
        }
    }
    return 0;
}

/**************************************************************************/

TaskService::TaskService( int numThreads ):
osg::Referenced( true ),
_lastRemoveFinishedThreadsStamp(0)
{
    _queue = new TaskRequestQueue();
    setNumThreads( numThreads );
}

void
TaskService::add( TaskRequest* request )
{
    _queue->add( request );
}

TaskService::~TaskService()
{
    _queue->setDone();

    for( TaskThreads::iterator i = _threads.begin(); i != _threads.end(); i++ )
    {
        (*i)->setDone(true);
    }

    for( TaskThreads::iterator i = _threads.begin(); i != _threads.end(); i++ )
    {
        (*i)->cancel();
        delete (*i);
    }
}

int
TaskService::getStamp() const
{
    return _queue->getStamp();
}

void
TaskService::setStamp( int stamp )
{
    _queue->setStamp( stamp );
    //Remove finished threads every 60 frames
    if (stamp - _lastRemoveFinishedThreadsStamp > 60)
    {
      removeFinishedThreads();
      _lastRemoveFinishedThreadsStamp = stamp;
    }
}

int
TaskService::getNumThreads() const
{
    return _numThreads;
}

void
TaskService::setNumThreads(int numThreads )
{
    _numThreads = osg::maximum(1, numThreads);
    adjustThreadCount();
}

void
TaskService::adjustThreadCount()
{
    OpenThreads::ScopedLock<OpenThreads::ReentrantMutex> lock(_threadMutex);
    removeFinishedThreads();
    int numActiveThreads = 0;
    for( TaskThreads::iterator i = _threads.begin(); i != _threads.end(); i++ )
    {
        if (!(*i)->getDone()) numActiveThreads++;
    }

    int diff = _numThreads - numActiveThreads;
    if (diff > 0)
    {
        osg::notify(osg::INFO) << "Adding " << diff << " threads to TaskService " << std::endl;
        //We need to add some threads
        for (unsigned int i = 0; i < diff; ++i)
        {
            TaskThread* thread = new TaskThread( _queue.get() );
            _threads.push_back( thread );
            thread->start();
        }       
    }
    else if (diff < 0)
    {
        diff = osg::absolute( diff );
        osg::notify(osg::INFO) << "Removing " << diff << " threads from TaskService " << std::endl;
        int numRemoved = 0;
        //We need to remove some threads
        for( TaskThreads::iterator i = _threads.begin(); i != _threads.end(); i++ )
        {
            if (!(*i)->getDone())
            {
                (*i)->setDone( true );
                numRemoved++;
                if (numRemoved == diff) break;
            }
        }
    }  
}

void
TaskService::removeFinishedThreads()
{
    OpenThreads::ScopedLock<OpenThreads::ReentrantMutex> lock(_threadMutex);
    unsigned int numRemoved = 0;
    for (TaskThreads::iterator i = _threads.begin(); i != _threads.end();)
    {
        //Erase the threads are not running
        if (!(*i)->isRunning())
        {
            i = _threads.erase( i );
            numRemoved++;
        }
        else
        {
            i++;
        }
    }
    if (numRemoved > 0)
    {
        osg::notify(osg::INFO) << "Removed " << numRemoved << " finished threads " << std::endl;
    }
}

