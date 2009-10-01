#include <osgEarth/TaskService>

using namespace osgEarth;
using namespace OpenThreads;

/**************************************************************************/

TaskRequest::TaskRequest( float priority ) :
_priority( priority ),
_state( STATE_IDLE )
{
    _progress = new TaskProgress();
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

TaskRequestQueue::TaskRequestQueue()
{
    _block = new osg::RefBlock();
}

void 
TaskRequestQueue::add( TaskRequest* request )
{
    ScopedLock<Mutex> lock(_mutex);

    request->setState( TaskRequest::STATE_PENDING );
    
    // insert by priority.
    for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); i++ )
    {
        if ( request->getPriority() > i->get()->getPriority() )
        {
            _requests.insert( i, request );
            //osg::notify(osg::NOTICE) << "Queue has items, setting block to true " << std::endl;
            _block->set( true );
            return;
        }
    }

    _requests.push_back( request );
    //osg::notify(osg::NOTICE) << "Queue has items, setting block to true " << std::endl;
    _block->set( true );
}

TaskRequest* 
TaskRequestQueue::get()
{
    if (_requests.empty())
    {
        _block->block();
        //osg::notify(osg::NOTICE) << "Unblocked" << std::endl;
    }

    ScopedLock<Mutex> lock(_mutex);

    if (_requests.empty()) return 0;
    
    osg::ref_ptr<TaskRequest> next = _requests.front();
    _requests.pop_front();
    if (_requests.empty())
    {
        //osg::notify(osg::NOTICE) << "Queue empty, setting block to false " << std::endl;
        _block->set(false);
    }
    return next.release();
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
            // discard an out-of-date request:
            if ( _queue->getStamp() - _request->getStamp() > 2 )
            {
                //osg::notify(osg::NOTICE) << "discarding request due to expiration" << std::endl;
                _request->setState( TaskRequest::STATE_IDLE ); // allows it to re-schedule
                continue;
            }

            // discard a completed or canceled request:
            if ( _request->getState() != TaskRequest::STATE_PENDING )
            {
                continue;
            }

            //osg::notify(osg::NOTICE) << "Thread " << this->getThreadId() << " running request" << std::endl;

            _request->setState( TaskRequest::STATE_IN_PROGRESS );

            _request->run();
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
    if (isRunning())
    {
      _done = true;      

      _queue->release();

      if (_request.valid())
          _request->cancel();

      while (isRunning())
      {
          _queue->release();          
          OpenThreads::Thread::YieldCurrentThread();
      }
    }
    return 0;
}

/**************************************************************************/

TaskService::TaskService( int numThreads )
{
    _queue = new TaskRequestQueue();

    if ( numThreads <= 0 ) numThreads = 1;
    for( int i=0; i<numThreads; i++ )
    {
        TaskThread* thread = new TaskThread( _queue.get() );
        _threads.push_back( thread  );
        thread->start();
    }
}

void
TaskService::add( TaskRequest* request )
{
    _queue->add( request );
}

TaskService::~TaskService()
{
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

void
TaskService::setStamp( int stamp )
{
    _queue->setStamp( stamp );
}
