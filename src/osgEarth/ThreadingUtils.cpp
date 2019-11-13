/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2018 Pelican Mapping
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
#include <osgEarth/ThreadingUtils>

#include <osgDB/ReadFile>
#include <osgEarth/Utils>
#include <osgEarth/URI>

#ifdef _WIN32
    extern "C" unsigned long __stdcall GetCurrentThreadId();
#elif defined(__APPLE__) || defined(__LINUX__) || defined(__FreeBSD__) || defined(__FreeBSD_kernel__) || defined(__ANDROID__)
#   include <unistd.h>
#   include <sys/syscall.h>
#else
#   include <pthread.h>
#endif

using namespace osgEarth::Threading;
using namespace osgEarth::Util;

//------------------------------------------------------------------------

unsigned osgEarth::Threading::getCurrentThreadId()
{
  /*   OpenThreads::Thread* t = OpenThreads::Thread::CurrentThread();
   return t ? t->getThreadId() : 0u;*/
  
#ifdef _WIN32
  return (unsigned)::GetCurrentThreadId();
#elif __APPLE__
  return ::syscall(SYS_thread_selfid);
#elif __ANDROID__
  return gettid();
#elif __LINUX__
  return (unsigned)::syscall(SYS_gettid);
#elif defined(__FreeBSD__) || defined(__FreeBSD_kernel__)
  long  tid;
  syscall(SYS_thr_self, &tid);
  return (unsigned)tid;
#else
  /* :XXX: this truncates to 32 bits, but better than nothing */
  return (unsigned)pthread_self();
#endif
}

//...................................................................

Event::Event() :
_set(false)
{
    //nop
}

Event::~Event()
{
    reset();
    for (int i = 0; i < 255; ++i) // workaround buggy broadcast
        _cond.signal();
}

bool Event::wait()
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_m);
    return _set ? true : (_cond.wait(&_m) == 0);
}

bool Event::wait(unsigned timeout_ms)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_m);
    return _set ? true : (_cond.wait(&_m, timeout_ms) == 0);
}

bool Event::waitAndReset()
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_m);
    if (_set) {
        _set = false;
        return true;
    }
    else {
        bool value = _cond.wait(&_m) == 0;
        _set = false;
        return value;
    }
}

void Event::set()
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_m);
    if (!_set) {
        _set = true;
        _cond.broadcast(); // possible deadlock before OSG r10457 on windows
    }
}

void Event::reset()
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_m);
    _set = false;
}

//...................................................................

MultiEvent::MultiEvent(int num) :
_set(num), _num(num) 
{
    //nop
}

MultiEvent::~MultiEvent()
{
    reset();
    for (int i = 0; i < 255; ++i) // workaround buggy broadcast
        _cond.signal();
}

bool MultiEvent::wait()
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_m);
    while (_set > 0)
        if (_cond.wait(&_m) != 0)
            return false;
    return true;
}

bool MultiEvent::waitAndReset()
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_m);
    while (_set > 0)
        if (_cond.wait(&_m) != 0)
            return false;
    _set = _num;
    return true;
}

void MultiEvent::set()
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_m);
    if (_set > 0)
        --_set;
    if (_set == 0)
        _cond.broadcast(); // possible deadlock before OSG r10457 on windows
    //_cond.signal();
}

void MultiEvent::reset()
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_m);
    _set = _num;
}

//...................................................................

ReadWriteMutex::ReadWriteMutex() :
_readerCount(0)
{
    _noWriterEvent.set();
    _noReadersEvent.set();
}

void ReadWriteMutex::readLock()
{
    for (;;)
    {
        _noWriterEvent.wait();           // wait for a writer to quit if there is one
        incrementReaderCount();          // register this reader
        if (!_noWriterEvent.isSet())     // double lock check, in case a writer snuck in while inrementing
            decrementReaderCount();      // if it did, undo the registration and try again
        else
            break;                       // otherwise, we're in
    }
}

void ReadWriteMutex::readUnlock()
{
    decrementReaderCount();              // unregister this reader
}

void ReadWriteMutex::writeLock()
{
    for (;;)
    {
        _noReadersEvent.wait(); // wait for no readers

        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_lockWriterMutex);
        _noWriterEvent.wait();  // wait for no writers
        _noWriterEvent.reset(); // signal that there is now a writer

        if (_noReadersEvent.isSet()) // still no readers? done.
            break;
        else
            _noWriterEvent.set(); // otherwise, a reader snuck in, so try again.
    }
}

void ReadWriteMutex::writeUnlock()
{
    _noWriterEvent.set();
}

void ReadWriteMutex::incrementReaderCount()
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_readerCountMutex);
    _readerCount++;            // add a reader
    _noReadersEvent.reset();   // there's at least one reader now so clear the flag
}

void ReadWriteMutex::decrementReaderCount()
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_readerCountMutex);
    _readerCount--;               // remove a reader
    if (_readerCount <= 0)      // if that was the last one, signal that writers are now allowed
        _noReadersEvent.set();
}

ThreadPool::ThreadPool(unsigned int numThreads) :
    _numThreads(numThreads)
{
    _queue = new osg::OperationQueue;
    startThreads();
}

ThreadPool::~ThreadPool()
{
    stopThreads();
}

osg::OperationQueue* ThreadPool::getQueue() const
{
    return _queue.get();
}

void ThreadPool::startThreads()
{
    for (unsigned int i = 0; i < _numThreads; ++i)
    {
        osg::OperationsThread* thread = new osg::OperationsThread();
        thread->setOperationQueue(_queue.get());
        thread->start();
        _threads.push_back(thread);
    }
}

void ThreadPool::stopThreads()
{
    for (unsigned int i = 0; i < _threads.size(); ++i)
    {
        osg::ref_ptr< osg::OperationsThread > thread = _threads[i].get();
        thread->setDone(true);
        thread->join();
    }
}

#if 0
namespace {
    class LoadNodeOperation : public osg::Operation
    {
    public:
        LoadNodeOperation(const URI& uri, osgDB::Options* options, osgUtil::IncrementalCompileOperation* ico, osgEarth::Threading::Promise<osg::Node> promise) :
            _uri(uri),
            _promise(promise),
            _ico(ico),
            _options(options)
        {
        }

        void operator()(osg::Object*)
        {
            if (!_promise.isAbandoned())
            {
                // Read the node
                osgEarth::ReadResult result = _uri.readNode(_options.get());
                //osg::ref_ptr< osg::Node > result = osgDB::readNodeFile(_url, _options.get());

                // If we have an ICO, wait for it to be compiled
                if (result.succeeded() && _ico.valid())
                {
                    osg::ref_ptr<osgUtil::IncrementalCompileOperation::CompileSet> compileSet =
                        new osgUtil::IncrementalCompileOperation::CompileSet(result.getNode());

                    _ico->add(compileSet.get());

                    // spin wait
                    while (
                        !_promise.isAbandoned() &&          // user hasn't gone away?
                        !compileSet->compiled() &&          // compilation not finished?
                        compileSet->referenceCount() > 1)   // compiler disappeared?
                    {
                        OpenThreads::Thread::microSleep(1000);
                    }
                }

                _promise.resolve(result.getNode());
            }
        }

        osgEarth::Threading::Promise<osg::Node> _promise;
        osg::ref_ptr< osgUtil::IncrementalCompileOperation > _ico;
        osg::ref_ptr< osgDB::Options > _options;
        URI _uri;
    };
}

Future<osg::Node> osgEarth::Threading::readNodeAsync(const URI& uri, osgUtil::IncrementalCompileOperation* ico, osgDB::Options* options)
{
    osg::ref_ptr<ThreadPool> threadPool;
    if (options)
    {
        threadPool = OptionsData<ThreadPool>::get(options, "threadpool");
    }

    Promise<osg::Node> promise;

    osg::ref_ptr< osg::Operation > operation = new LoadNodeOperation(uri, options, ico, promise);

    if (operation.valid())
    {
        if (threadPool.valid())
        {
            threadPool->getQueue()->add(operation);
        }
        else
        {
            OE_WARN << "Immediately resolving async operation, please set a ThreadPool on the Options object" << std::endl;
            operation->operator()(0);
        }
    }

    return promise.getFuture();
}
#endif
