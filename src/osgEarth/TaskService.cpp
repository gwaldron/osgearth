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
#include <osgEarth/TaskService>
#include <osg/Notify>
#include <osg/Math>

using namespace osgEarth;
using namespace OpenThreads;

#define LC "[TaskService] "

//------------------------------------------------------------------------

TaskRequest::TaskRequest( float priority ) :
osg::Referenced( true ),
_priority( priority ),
_state( STATE_IDLE ),
_stamp(0),
_startTime(0),
_endTime(0),
_completedEvent(0L)
{
    _progress = new ProgressCallback();
}

void
TaskRequest::run()
{
    if ( _state == STATE_IN_PROGRESS )
    {
        _startTime = osg::Timer::instance()->tick();
        (*this)( _progress.get() );        
        _endTime = osg::Timer::instance()->tick();
    }
    else
    {
        _progress->cancel();
    }
}

void 
TaskRequest::cancel()
{
    //OE_INFO << LC << "TR [" << getName() << "] canceled" << std::endl;
    _progress->cancel();
}

bool
TaskRequest::wasCanceled() const
{
    return _progress->isCanceled();
}

//------------------------------------------------------------------------

TaskRequestQueue::TaskRequestQueue(unsigned int maxSize) :
osg::Referenced( true ),
_done( false ),
_maxSize( maxSize ),
_stamp(0)
{
    //nop
}

void
TaskRequestQueue::clear()
{
    ScopedLock<Mutex> lock(_mutex);
    _requests.clear();
}

void
TaskRequestQueue::cancel()
{
    ScopedLock<Mutex> lock(_mutex);
    for (TaskRequestPriorityMap::iterator it = _requests.begin(); it != _requests.end(); ++it)
        (*it).second->cancel();

    _requests.clear();
}

bool
TaskRequestQueue::isFull() const
{
    return _maxSize > 0 && (_maxSize == _requests.size());
}

bool
TaskRequestQueue::isEmpty() const
{
    return !_done && _requests.empty();
}

unsigned int
TaskRequestQueue::getNumRequests() const
{
    ScopedLock<Mutex> lock(const_cast<TaskRequestQueue*>(this)->_mutex);
    return _requests.size();
}

void 
TaskRequestQueue::add( TaskRequest* request )
{
    request->setState( TaskRequest::STATE_PENDING );

    // install a progress callback if one isn't already installed
    if ( !request->getProgressCallback() )
        request->setProgressCallback( new ProgressCallback() );

    {
        // Lock on the add mutex so no one else can add.
        ScopedLock<Mutex> lock( _mutex );

        while(isFull())
        {
            _notFull.wait(&_mutex);
        }

        // Check to make sure the bounded queue is working correctly.
        if (_maxSize > 0 && _requests.size() > _maxSize)
        {
            OE_NOTICE << "ERROR:  TaskRequestQueue requests " << getNumRequests() << " > max size of " << _maxSize << std::endl;
        }

        // insert by priority.
        _requests.insert( std::pair<float,TaskRequest*>(request->getPriority(), request) );
    }

    //OE_NOTICE << "There are now " << _requests.size() << " tasks" << std::endl;

    // since there is data in the queue, wake up one waiting task thread.
    _notEmpty.signal(); 
}

TaskRequest* 
TaskRequestQueue::get()
{
    
    osg::ref_ptr<TaskRequest> next;
    {
        ScopedLock<Mutex> lock(_mutex);

        while ( isEmpty() )
        {                
            _notEmpty.wait( &_mutex );        
        }

        if ( _done )
        {
            return 0L;
        }

        next = _requests.begin()->second.get(); //_requests.front();
        _requests.erase( _requests.begin() ); //_requests.pop_front();
    }

    // I'm done, someone else take a turn:
    // (technically this shouldn't be necessary since add() bumps the semaphore once
    // for each request in the queue)    
    _notFull.signal();    

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

    // alternative to buggy win32 broadcast (OSG pre-r10457 on windows)
    for(int i=0; i<128; i++) {
        _notFull.signal();
        _notEmpty.signal();
    }
}

//------------------------------------------------------------------------

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
            PoisonPill* poison = dynamic_cast< PoisonPill* > ( _request.get());
            if ( poison )
            {
                OE_DEBUG << this->getThreadId() << " received poison pill.  Shutting down" << std::endl;
                // Add the poison pill back to the queue to kill any other threads.  If I'm going down, you're all going down with me!
                _queue->add( poison );
                break;
            }
            

            // discard a completed or canceled request:
            if ( _request->getState() != TaskRequest::STATE_PENDING )
            {
                _request->cancel();
            }

            else if ( !_request->wasCanceled() )
            {
                if ( _request->getProgressCallback() )
                    _request->getProgressCallback()->onStarted();

                _request->setState( TaskRequest::STATE_IN_PROGRESS );
                _request->run();

                //OE_INFO << LC << "Task \"" << _request->getName() << "\" runtime = " << _request->runTime() << " s." << std::endl;
            }
            else
            {
                //OE_INFO << LC << "Task \"" << _request->getName() << "\" was cancelled before it ran." << std::endl;
            }
            
            _request->setState( TaskRequest::STATE_COMPLETED );

            // signal the completion of a request.
            if ( _request->getProgressCallback() )
                _request->getProgressCallback()->onCompleted();

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

//------------------------------------------------------------------------

TaskService::TaskService( const std::string& name, int numThreads, unsigned int maxSize ):
osg::Referenced( true ),
_lastRemoveFinishedThreadsStamp(0),
_name(name),
_numThreads( 0 )
{
    _queue = new TaskRequestQueue( maxSize );
    setNumThreads( numThreads );
}

unsigned int
TaskService::getNumRequests() const
{
    return _queue->getNumRequests();
}

void
TaskService::add( TaskRequest* request )
{   
    //OE_INFO << LC << "TS [" << _name << "] adding request [" << request->getName() << "]" << std::endl;
    _queue->add( request );
}

void TaskService::waitforThreadsToComplete()
{        
    for( TaskThreads::iterator i = _threads.begin(); i != _threads.end(); i++ )
    {
        (*i)->join();
    }    
}

bool TaskService::areThreadsRunning()
{
    for( TaskThreads::iterator i = _threads.begin(); i != _threads.end(); i++ )
    {                
        if ((*i)->isRunning())
        {
            return true;
        }
    }    
    return false;
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
    if ( _numThreads != numThreads )
    {
        _numThreads = osg::maximum(1, numThreads);
        adjustThreadCount();
    }
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
        OE_DEBUG << LC << "Adding " << diff << " threads to TaskService " << std::endl;
        //We need to add some threads
        for (int i = 0; i < diff; ++i)
        {
            TaskThread* thread = new TaskThread( _queue.get() );
            _threads.push_back( thread );
            thread->start();
        }       
    }
    else if (diff < 0)
    {
        diff = osg::absolute( diff );
        OE_DEBUG << LC << "Removing " << diff << " threads from TaskService " << std::endl;
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

    OE_INFO << LC << "TaskService [" << _name << "] using " << _numThreads << " threads" << std::endl;
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
        OE_DEBUG << LC << "Removed " << numRemoved << " finished threads " << std::endl;
    }
}

void
TaskService::cancelAll()
{
    if (_numThreads > 0)
    {
        _numThreads = 0;
        adjustThreadCount();

        OE_INFO << LC << "Cancelled all threads in TaskService [" << _name << "]" << std::endl;
    }
}

//------------------------------------------------------------------------

TaskServiceManager::TaskServiceManager( int numThreads ) :
_numThreads( 0 ),
_targetNumThreads( numThreads )
{
    //nop
}

void
TaskServiceManager::setNumThreads( int numThreads )
{
    reallocate( numThreads );
}

TaskService*
TaskServiceManager::add( UID uid, float weight )
{
    ScopedLock<Mutex> lock( _taskServiceMgrMutex );

    if ( weight <= 0.0f )
        weight = 0.001;

    TaskServiceMap::iterator i = _services.find( uid );
    if ( i != _services.end() )
    {
        i->second.second = weight;
        reallocate( _targetNumThreads );
        return i->second.first.get();
    }
    else
    {
        TaskService* newService = new TaskService( "", 1 );
        _services[uid] = WeightedTaskService( newService, weight );
        reallocate( _targetNumThreads );
        return newService;
    }
}

TaskService*
TaskServiceManager::get( UID uid ) const
{
    ScopedLock<Mutex> lock( const_cast<TaskServiceManager*>(this)->_taskServiceMgrMutex );
    TaskServiceMap::const_iterator i = _services.find(uid);
    return i != _services.end() ? i->second.first.get() : 0L;
}

TaskService*
TaskServiceManager::getOrAdd( UID uid, float weight ) 
{
    TaskService* service = get( uid );
    return service ? service : add( uid, weight );
}

void
TaskServiceManager::remove( TaskService* service )
{
    ScopedLock<Mutex> lock( _taskServiceMgrMutex );
    for( TaskServiceMap::iterator i = _services.begin(); i != _services.end(); ++i )
    {
        if ( i->second.first.get() == service ) 
        {
            _services.erase( i );
            reallocate( _targetNumThreads );
            break;
        }
    }
}

void
TaskServiceManager::remove( UID uid )
{
    ScopedLock<Mutex> lock( _taskServiceMgrMutex );
    _services.erase( uid );
    reallocate( _targetNumThreads );
}

void
TaskServiceManager::setWeight( TaskService* service, float weight )
{
    ScopedLock<Mutex> lock( _taskServiceMgrMutex );

    if ( weight <= 0.0f )
        weight = 0.001;

    if ( !service )
        return;

    for( TaskServiceMap::iterator i = _services.begin(); i != _services.end(); ++i )
    {
        if ( i->second.first.get() == service )
        {
            i->second.second = weight;
            reallocate( _targetNumThreads );
            break;
        }
    }    
}

void
TaskServiceManager::reallocate( int numThreads )
{
    // first, total up all the weights.
    float totalWeight = 0.0f;
    for( TaskServiceMap::const_iterator i = _services.begin(); i != _services.end(); ++i )
        totalWeight += i->second.second;

    // next divide the total thread pool size by the relative weight of each service.
    _numThreads = 0;
    for( TaskServiceMap::const_iterator i = _services.begin(); i != _services.end(); ++i )
    {
        int threads = osg::maximum( 1, (int)( (float)_targetNumThreads * (i->second.second / totalWeight) ) );
        i->second.first->setNumThreads( threads );
        _numThreads += threads;
    }
}
