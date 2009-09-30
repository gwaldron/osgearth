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

TaskRequestQueue::TaskRequestQueue() :
_done( false )
{
    //nop
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
            _cond.signal();
            return;
        }
    }

    _requests.push_back( request );
    _cond.signal();
}

TaskRequest* 
TaskRequestQueue::get()
{
    ScopedLock<Mutex> lock(_mutex);
    
    while( !_done && _requests.empty() )
        _cond.wait(&_mutex);

    if ( _done )
        return 0L;

    osg::ref_ptr<TaskRequest> next = _requests.front();
    _requests.pop_front();
    return next.release();
}

void 
TaskRequestQueue::shutdown()
{
    ScopedLock<Mutex> lock(_mutex);
    _done = true;
    _cond.broadcast();
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
    while( !_done && (_request = _queue->get()).valid() )
    {
        if ( !_request.valid() )
            break;

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
        
        //osg::notify(osg::NOTICE) << "Thread " << this->getThreadId() << " completed request" << std::endl;
    }
}

void
TaskThread::shutdown()
{
    _done = true;
    if ( _request.valid() )
        _request->cancel();

    //this->cancel();
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
        (*i)->shutdown();
        (*i)->join();
        delete (*i);
    }

    _queue->shutdown();
}

void
TaskService::setStamp( int stamp )
{
    _queue->setStamp( stamp );
}
