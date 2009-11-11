#include <osgEarth/TaskService>

using namespace osgEarth;
using namespace OpenThreads;

/**************************************************************************/

TaskRequest::TaskRequest( float priority ) :
_priority( priority ),
_state( STATE_IDLE ),
osg::Referenced( true )
{
    _progress = new ProgressCallback();
    //_progress = new ConsoleProgressCallback();
}

void
TaskRequest::run()
{
    if ( _state == STATE_IN_PROGRESS )
    {
        (*this)( _progress.get() );

        _state = _progress->isCanceled() ? STATE_CANCELED : STATE_COMPLETED;
    }
}

void 
TaskRequest::cancel() {
    _progress->cancel();
}

bool
TaskRequest::isCanceled() const {
    return _progress->isCanceled();
}

/**************************************************************************/

TaskRequestQueue::TaskRequestQueue() :
_done( false )
{
//    _block = new osg::RefBlock();
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
    //_block->set( true );

    //osg::notify(osg::NOTICE) << "Queue has items, setting block to true " << std::endl;
    //osg::notify(osg::NOTICE) << "TaskRequestQueue size=" << _requests.size() << std::endl;
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


    //if (_requests.empty())
    //{
    //    _block->block();
    //    //osg::notify(osg::NOTICE) << "Unblocked" << std::endl;
    //}

    //ScopedLock<Mutex> lock(_mutex);

    //if (_requests.empty()) return 0;
    //
    //osg::ref_ptr<TaskRequest> next = _requests.front();
    //_requests.pop_front();
    //if (_requests.empty())
    //{
    //    //osg::notify(osg::NOTICE) << "Queue empty, setting block to false " << std::endl;
    //    _block->set(false);
    //}
    ////osg::notify(osg::NOTICE) << "TaskRequestQueue size=" << _requests.size() << std::endl;
    //return next.release();
}

void
TaskRequestQueue::setDone()
{
    // we need to obtain the mutex since we're using the Condition
    ScopedLock<Mutex> lock(_mutex);

    _done = true;

    // wake everyone up so they can see the _done flag set and exit.
    //_cond.broadcast();

    // alternative to buggy broadcast:
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
        //osg::notify(osg::NOTICE) << "Run thraed " << std::endl;
        if ( _done )
            break;

        if (_request.valid())
        {
            // discard a completed or canceled request:
            if ( _request->getState() != TaskRequest::STATE_PENDING || _request->isCanceled() )
            {
                _request = 0;
                continue;
            }

            //osg::notify(osg::NOTICE) << "Thread " << this->getThreadId() << " running request" << std::endl;

            _request->setState( TaskRequest::STATE_IN_PROGRESS );

            //osg::notify(osg::NOTICE) << "Executing Task (" << _request->getPriority() << ") : " << _request->getName() << std::endl;
            _request->run();

            //Release the request
            _request = 0;
        }
        else
        {
            OpenThreads::Thread::YieldCurrentThread();
        }

        //osg::notify(osg::NOTICE) << "Thread " << this->getThreadId() << " completed request" << std::endl;
    }
}

int
TaskThread::cancel()
{
    if ( isRunning() )
    {
      _done = true;  

      //Remove any pending requests
      //_queue->clear();

      //_queue->release();

      if (_request.valid())
          _request->cancel();

      while (isRunning())
      {
          //_queue->release();          
          OpenThreads::Thread::YieldCurrentThread();
      }
    }
    return 0;
}

/**************************************************************************/

TaskService::TaskService( int numThreads ):
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

