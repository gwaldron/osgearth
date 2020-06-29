/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <osgDB/Options>
#include <osg/OperationThread>
#include "Utils"
#include "Metrics"

#ifdef _WIN32
  #ifndef OSGEARTH_PROFILING
    // because Tracy already does this in its header file..
    extern "C" unsigned long __stdcall GetCurrentThreadId();
  #endif
#elif defined(__APPLE__) || defined(__LINUX__) || defined(__FreeBSD__) || defined(__FreeBSD_kernel__) || defined(__ANDROID__)
#   include <unistd.h>
#   include <sys/syscall.h>
#else
#   include <pthread.h>
#endif

using namespace osgEarth::Threading;
using namespace osgEarth::Util;

//...................................................................

#ifdef OSGEARTH_PROFILING
#define MUTEX_TYPE tracy::Lockable<std::recursive_mutex>
#else
#define MUTEX_TYPE std::recursive_mutex
#endif

Mutex::Mutex()
{
#ifdef OSGEARTH_PROFILING
    tracy::SourceLocationData* s = new tracy::SourceLocationData();
    s->name = nullptr;
    s->function = "unnamed";
    s->file = __FILE__;
    s->line = __LINE__;
    s->color = 0;
    _handle = new tracy::Lockable<std::mutex>(s);
    _data = s;
#else
    _handle = new std::mutex();
    _data = NULL;
#endif
}

Mutex::Mutex(const std::string& name, const char* file, std::uint32_t line) :
    _name(name)
{
#ifdef OSGEARTH_PROFILING        
    tracy::SourceLocationData* s = new tracy::SourceLocationData();
    s->name = nullptr;
    s->function = _name.c_str();
    s->file = file;
    s->line = line;
    s->color = 0;
    _handle = new tracy::Lockable<std::mutex>(s);
    _data = s;
#else
    _handle = new std::mutex();
    _data = NULL;
#endif
}

Mutex::~Mutex()
{
    delete static_cast<MUTEX_TYPE*>(_handle);
}

void
Mutex::setName(const std::string& name)
{
    _name = name;
#ifdef OSGEARTH_PROFILING
    if (_data)
    {
        tracy::SourceLocationData* s = static_cast<tracy::SourceLocationData*>(_data);
        s->function = _name.c_str();
    }
#endif
}

void
Mutex::lock()
{
    if (_name.empty()) {
        volatile int i=0; // breakpoint for finding unnamed mutexes -GW
    }
    static_cast<MUTEX_TYPE*>(_handle)->lock();
}

void
Mutex::unlock()
{
    static_cast<MUTEX_TYPE*>(_handle)->unlock();
}

bool
Mutex::try_lock()
{
    return static_cast<MUTEX_TYPE*>(_handle)->try_lock();
}

//...................................................................

#ifdef OSGEARTH_PROFILING
#define RECURSIVE_MUTEX_TYPE tracy::Lockable<std::recursive_mutex>
#else
#define RECURSIVE_MUTEX_TYPE std::recursive_mutex
#endif

RecursiveMutex::RecursiveMutex() :
    _enabled(true)
{
#ifdef OSGEARTH_PROFILING
    tracy::SourceLocationData* s = new tracy::SourceLocationData();
    s->name = nullptr;
    s->function = "unnamed recursive";
    s->file = __FILE__;
    s->line = __LINE__;
    s->color = 0;
    _handle = new tracy::Lockable<std::recursive_mutex>(s);
#else
    _handle = new std::recursive_mutex();
#endif
}

RecursiveMutex::RecursiveMutex(const std::string& name, const char* file, std::uint32_t line) :
    _name(name),
    _enabled(true)
{
#ifdef OSGEARTH_PROFILING        
    tracy::SourceLocationData* s = new tracy::SourceLocationData();
    s->name = nullptr;
    s->function = _name.c_str();
    s->file = file;
    s->line = line;
    s->color = 0;
    _handle = new tracy::Lockable<std::recursive_mutex>(s);
#else
    _handle = new std::recursive_mutex();
#endif
}

RecursiveMutex::~RecursiveMutex()
{
    if (_handle)
        delete static_cast<RECURSIVE_MUTEX_TYPE*>(_handle);
}

void
RecursiveMutex::disable()
{
    _enabled = false;
}

void
RecursiveMutex::lock()
{
    if (_enabled)
        static_cast<RECURSIVE_MUTEX_TYPE*>(_handle)->lock();
}

void
RecursiveMutex::unlock()
{
    if (_enabled)
        static_cast<RECURSIVE_MUTEX_TYPE*>(_handle)->unlock();
}

bool
RecursiveMutex::try_lock()
{
    if (_enabled)
        return static_cast<MUTEX_TYPE*>(_handle)->try_lock();
    else
        return true;
}

//...................................................................

unsigned osgEarth::Threading::getCurrentThreadId()
{  
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
Event::Event(const std::string& name) :
    _set(false),
    _m(name)
{
    //nop
}

Event::~Event()
{
    _set = false;
    for(int i=0; i<255; ++i)  // workaround buggy broadcast
        _cond.notify_all();
}

void
Event::setName(const std::string& name)
{
    _m.setName(name);
}

bool Event::wait()
{
    std::unique_lock<Mutex> lock(_m);
    if (!_set)
        _cond.wait(lock);
    return true;
}

bool Event::wait(unsigned timeout_ms)
{
    std::unique_lock<Mutex> lock(_m);
    if (!_set)
    {
        std::cv_status result = _cond.wait_for(lock, std::chrono::milliseconds(timeout_ms));
        return result == std::cv_status::no_timeout ? true : false;
    }
    return true;
}

bool Event::waitAndReset()
{
    std::unique_lock<Mutex> lock(_m);
    if (!_set)
        _cond.wait(lock);
    _set = false;
    return true;
}

void Event::set()
{
    std::unique_lock<Mutex> lock(_m);
    if (!_set) {
        _set = true;
        _cond.notify_all();
    }
}

void Event::reset()
{
    std::unique_lock<Mutex> lock(_m);
    _set = false;
}

//...................................................................

ReadWriteMutex::ReadWriteMutex() :
    _readers(0), _writers(0)
{
    //NOP
}

ReadWriteMutex::ReadWriteMutex(const std::string& name) :
    _readers(0), _writers(0), _m(name)
{
    //NOP
}

void ReadWriteMutex::read_lock()
{
    std::unique_lock<Mutex> lock(_m);
    while (_writers > 0)
        _unlocked.wait(lock);
    ++_readers;
}

void ReadWriteMutex::read_unlock()
{
    std::unique_lock<Mutex> lock(_m);
    --_readers;
    if (_readers == 0)
        _unlocked.notify_all();
}

void ReadWriteMutex::write_lock()
{
    std::unique_lock<Mutex> lock(_m);
    while (_writers > 0 || _readers > 0)
        _unlocked.wait(lock);
    ++_writers;
}

void ReadWriteMutex::write_unlock()
{
    std::unique_lock<Mutex> lock(_m);
    _writers = 0;
    _unlocked.notify_all();
}

void ReadWriteMutex::setName(const std::string& name)
{
    _m.setName(name);
}
        

#if 0
ReadWriteMutex::ReadWriteMutex() :
    _readerCount(0),
    _noWriterEvent("RWMutex unnamed"),
    _noReadersEvent("RWMutex unnamed")
{
    _noWriterEvent.set();
    _noReadersEvent.set();
}

ReadWriteMutex::ReadWriteMutex(const std::string& name) :
    _readerCount(0),
    _noWriterEvent(name),
    _noReadersEvent(name)
{
    _noWriterEvent.set();
    _noReadersEvent.set();
}

void
ReadWriteMutex::setName(const std::string& name)
{
    _noWriterEvent.setName(name);
    _noReadersEvent.setName(name);
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

        std::lock_guard<Mutex> lock(_lockWriterMutex);
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
    std::lock_guard<Mutex> lock(_readerCountMutex);
    _readerCount++;            // add a reader
    _noReadersEvent.reset();   // there's at least one reader now so clear the flag
}

void ReadWriteMutex::decrementReaderCount()
{
    std::lock_guard<Mutex> lock(_readerCountMutex);
    _readerCount--;               // remove a reader
    if (_readerCount <= 0)      // if that was the last one, signal that writers are now allowed
        _noReadersEvent.set();
}
#endif


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

void
ThreadPool::put(osgDB::Options* options)
{
    if (options)
    {
        OptionsData<ThreadPool>::set(options, "osgEarth::ThreadPool", this);
    }
}

osg::ref_ptr<ThreadPool>
ThreadPool::get(const osgDB::Options* options)
{
    if (!options) return NULL;
    return OptionsData<ThreadPool>::get(options, "osgEarth::ThreadPool");
}

