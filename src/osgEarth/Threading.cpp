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
#include <osgEarth/Threading>
#include <osgDB/Options>
#include <osg/OperationThread>
#include "Utils"
#include "Metrics"

#ifdef _WIN32
#   include <Windows.h>
#   include <processthreadsapi.h>
#elif defined(__APPLE__) || defined(__LINUX__) || defined(__FreeBSD__) || defined(__FreeBSD_kernel__) || defined(__ANDROID__)
#   include <unistd.h>
#   include <sys/syscall.h>
#   include <pthread.h>
#endif

// b/c windows defines override std:: functions
#undef min
#undef max

using namespace osgEarth::Threading;
using namespace osgEarth::Util;

//...................................................................

//#ifdef OSGEARTH_PROFILING
//#define MUTEX_TYPE tracy::Lockable<std::recursive_mutex>
//#else
//#define MUTEX_TYPE std::recursive_mutex
//#endif

Mutex::Mutex() :
    _metricsData(nullptr)
{
#ifdef OSGEARTH_PROFILING
    if (Metrics::enabled())
    {
        tracy::SourceLocationData* s = new tracy::SourceLocationData();
        s->name = nullptr;
        s->function = "unnamed";
        s->file = __FILE__;
        s->line = __LINE__;
        s->color = 0;
        _handle = new tracy::Lockable<std::mutex>(s);
        _metricsData = s;
    }
    else
    {
        _handle = new std::mutex();
    }
#else
    _handle = new std::mutex();
#endif
}

Mutex::Mutex(const std::string& name, const char* file, std::uint32_t line) :
    _name(name),
    _metricsData(nullptr)
{
#ifdef OSGEARTH_PROFILING
    if (Metrics::enabled())
    {
        tracy::SourceLocationData* s = new tracy::SourceLocationData();
        s->name = nullptr;
        s->function = _name.c_str();
        s->file = file;
        s->line = line;
        s->color = 0;
        _handle = new tracy::Lockable<std::mutex>(s);
        _metricsData = s;
    }
    else
    {
        _handle = new std::mutex();
    }
#else
    _handle = new std::mutex();
#endif
}

Mutex::~Mutex()
{
#ifdef OSGEARTH_PROFILING
    if (_metricsData)
        delete static_cast<tracy::Lockable<std::mutex>*>(_handle);
    else
#endif
        delete static_cast<std::mutex*>(_handle);
}

void
Mutex::setName(const std::string& name)
{
    _name = name;
#ifdef OSGEARTH_PROFILING
    if (_metricsData)
    {
        tracy::SourceLocationData* s = static_cast<tracy::SourceLocationData*>(_metricsData);
        s->function = _name.c_str();
    }
#endif
}

void
Mutex::lock()
{
    if (_name.empty()) {
        volatile int x =0 ; // breakpoint for finding unnamed mutexes
    }

#ifdef OSGEARTH_PROFILING
    if (_metricsData)
        static_cast<tracy::Lockable<std::mutex>*>(_handle)->lock();
    else
#endif
        static_cast<std::mutex*>(_handle)->lock();
}

void
Mutex::unlock()
{
#ifdef OSGEARTH_PROFILING
    if (_metricsData)
        static_cast<tracy::Lockable<std::mutex>*>(_handle)->unlock();
    else
#endif
        static_cast<std::mutex*>(_handle)->unlock();
}

bool
Mutex::try_lock()
{
#ifdef OSGEARTH_PROFILING
    if (_metricsData)
        return static_cast<tracy::Lockable<std::mutex>*>(_handle)->try_lock();
    else
#endif
        return static_cast<std::mutex*>(_handle)->try_lock();
}

//...................................................................

RecursiveMutex::RecursiveMutex() :
    _enabled(true),
    _metricsData(nullptr)
{
#ifdef OSGEARTH_PROFILING
    if (Metrics::enabled())
    {
        tracy::SourceLocationData* s = new tracy::SourceLocationData();
        s->name = nullptr;
        s->function = "unnamed recursive";
        s->file = __FILE__;
        s->line = __LINE__;
        s->color = 0;
        _handle = new tracy::Lockable<std::recursive_mutex>(s);
        _metricsData = s;
    }
    else
    {
        _handle = new std::recursive_mutex();
    }
#else
    _handle = new std::recursive_mutex();
#endif
}

RecursiveMutex::RecursiveMutex(const std::string& name, const char* file, std::uint32_t line) :
    _name(name),
    _enabled(true),
    _metricsData(nullptr)
{
#ifdef OSGEARTH_PROFILING
    if (Metrics::enabled())
    {
        tracy::SourceLocationData* s = new tracy::SourceLocationData();
        s->name = nullptr;
        s->function = _name.c_str();
        s->file = file;
        s->line = line;
        s->color = 0;
        _handle = new tracy::Lockable<std::recursive_mutex>(s);
        _metricsData = s;
    }
    else
    {
        _handle = new std::recursive_mutex();
    }
#else
    _handle = new std::recursive_mutex();
#endif
}

RecursiveMutex::~RecursiveMutex()
{
    if (_handle)
    {
#ifdef OSGEARTH_PROFILING
        if (_metricsData)
            delete static_cast<tracy::Lockable<std::recursive_mutex>*>(_handle);
        else
#endif
            delete static_cast<std::recursive_mutex*>(_handle);
    }
}

void
RecursiveMutex::disable()
{
    _enabled = false;
}

void
RecursiveMutex::setName(const std::string& name)
{
    _name = name;

#ifdef OSGEARTH_PROFILING
    if (_metricsData)
    {
        tracy::SourceLocationData* s = static_cast<tracy::SourceLocationData*>(_metricsData);
        s->function = _name.c_str();
    }
#endif
}

void
RecursiveMutex::lock()
{
    if (_enabled)
    {
#ifdef OSGEARTH_PROFILING
        if (_metricsData)
            static_cast<tracy::Lockable<std::recursive_mutex>*>(_handle)->lock();
        else
#endif
            static_cast<std::recursive_mutex*>(_handle)->lock();
    }
}

void
RecursiveMutex::unlock()
{
    if (_enabled)
    {
#ifdef OSGEARTH_PROFILING
        if (_metricsData)
            static_cast<tracy::Lockable<std::recursive_mutex>*>(_handle)->unlock();
        else
#endif
            static_cast<std::recursive_mutex*>(_handle)->unlock();
    }
}

bool
RecursiveMutex::try_lock()
{
    if (_enabled)
    {
#ifdef OSGEARTH_PROFILING
        if (_metricsData)
            return static_cast<tracy::Lockable<std::recursive_mutex>*>(_handle)->try_lock();
        else
#endif
            return static_cast<std::recursive_mutex*>(_handle)->try_lock();
    }
    else return true;
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


#undef LC
#define LC "[ThreadPool] "

ThreadPool::ThreadPool(unsigned numThreads) :
    _name("osgEarth.ThreadPool"),
    _numThreads(numThreads),
    _done(false),
    _queueMutex("ThreadPool")
{
    startThreads();
}

ThreadPool::ThreadPool(const std::string& name, unsigned numThreads) :
    _name(name),
    _numThreads(numThreads),
    _done(false),
    _queueMutex("ThreadPool")
{
    startThreads();
}

ThreadPool::~ThreadPool()
{
    stopThreads();
}

void ThreadPool::run(osg::Operation* op)
{
    if (op)
    {
        Threading::ScopedMutexLock lock(_queueMutex);
        _queue.push(op);
        _block.notify_all();
    }
}

unsigned ThreadPool::getNumOperationsInQueue() const
{
    return _queue.size();
}

void ThreadPool::startThreads()
{
    _done = false;

    for(unsigned i=0; i<_numThreads; ++i)
    {
        _threads.push_back(std::thread( [this]
        {
            OE_DEBUG << LC << "Thread " << std::this_thread::get_id() << " started." << std::endl;

            OE_THREAD_NAME(this->_name.c_str());

            while(!_done)
            {
                osg::ref_ptr<osg::Operation> op;
                {
                    std::unique_lock<Mutex> lock(_queueMutex);

                    _block.wait(lock, [this] {
                        return !_queue.empty() || _done;
                    });

                    if (!_queue.empty() && !_done)
                    {
                        op = _queue.front();
                        _queue.pop();
                    }
                }

                if (op.valid())
                {
                    // run the op:
                    (*op.get())(nullptr);

                    // if it's a keeper, requeue it
                    if (op->getKeep())
                    {
                        Threading::ScopedMutexLock lock(_queueMutex);
                        _queue.push(op);
                    }
                    op = nullptr;
                }
            }
            OE_DEBUG << LC << "Thread " << std::this_thread::get_id() << " exiting." << std::endl;
        }));
    }
}

void ThreadPool::stopThreads()
{
    _done = true;
    _block.notify_all();

    for(unsigned i=0; i<_numThreads; ++i)
    {
        if (_threads[i].joinable())
        {
            _threads[i].join();
        }
    }

    _threads.clear();
    
    // Clear out the queue
    {
        Threading::ScopedMutexLock lock(_queueMutex);
        Queue emptyQueue;
        _queue.swap(emptyQueue);
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

void
osgEarth::Threading::setThreadName(const std::string& name)
{
#if (defined _WIN32 && defined _WIN32_WINNT_WIN10 && defined _WIN32_WINNT && _WIN32_WINNT >= _WIN32_WINNT_WIN10) || (defined __CYGWIN__)

    wchar_t buf[256];
    mbstowcs(buf, name.c_str(), 256);
    ::SetThreadDescription(::GetCurrentThread(), buf);

#elif defined _GNU_SOURCE && !defined __EMSCRIPTEN__ && !defined __CYGWIN__

    const auto sz = strlen( name.c_str() );
    if( sz <= 15 )
    {
        pthread_setname_np( pthread_self(), name.c_str() );
    }
    else
    {
        char buf[16];
        memcpy( buf, name.c_str(), 15 );
        buf[15] = '\0';
        pthread_setname_np( pthread_self(), buf );
    }
#endif
}


#undef LC
#define LC "[JobArena] "

Mutex JobArena::_arenas_mutex("OE:JobArena");

std::unordered_map<std::string, osg::ref_ptr<JobArena>> JobArena::_arenas;

std::unordered_map<std::string, unsigned> JobArena::_arenaSizes;

#define OE_ARENA_DEFAULT_SIZE 2u

JobArena*
JobArena::arena(const std::string& name)
{
    ScopedMutexLock lock(_arenas_mutex);
    osg::ref_ptr<JobArena>& arena = _arenas[name];
    if (!arena.valid())
    {
        auto iter = _arenaSizes.find(name);
        unsigned numThreads = iter != _arenaSizes.end() ? iter->second : OE_ARENA_DEFAULT_SIZE;
        arena = new JobArena(name, numThreads);
    }
    return arena.get();
}

void
JobArena::setSize(const std::string& name, unsigned numThreads)
{
    ScopedMutexLock lock(_arenas_mutex);

    _arenaSizes[name] = numThreads;

    auto iter = _arenas.find(name);
    if (iter != _arenas.end())
    {
        osg::ref_ptr<JobArena>& arena = iter->second;
        OE_SOFT_ASSERT_AND_RETURN(arena.get() != nullptr, __func__,);
        arena->stopThreads();
        arena->_numThreads = numThreads;
        arena->startThreads();
    }
}

JobArena::JobArena(const std::string& name, unsigned numThreads) :
    _name("OE.JobArena[" + name + "]"),
    _numThreads(numThreads),
    _done(false),
    _queueMutex("OE.JobArena.Queue")
{
    startThreads();
}

JobArena::~JobArena()
{
    stopThreads();
}

void
JobArena::startThreads()
{
    _done = false;

    for (unsigned i = 0; i < _numThreads; ++i)
    {
        _threads.push_back(std::thread([this]
            {
                OE_INFO << LC << "Arena " << _name << ": thread " << std::this_thread::get_id() << " started." << std::endl;

                OE_THREAD_NAME(this->_name.c_str());

                while (!_done)
                {
                    std::function<void()> job;
                    bool have_job = false;
                    {
                        std::unique_lock<Mutex> lock(_queueMutex);

                        //_block.wait_for(lock, std::chrono::seconds(1));

                        _block.wait(lock, [this] {
                            return _queue.empty() == false || _done == true;
                            });

                        if (!_queue.empty() && !_done)
                        {
                            job = std::move(_queue.front());
                            have_job = true;
                            _queue.pop_front();
                        }
                    }

                    if (have_job)
                    {
                        job();
                    }
                }
                OE_DEBUG << LC << "Thread " << std::this_thread::get_id() << " exiting." << std::endl;
            }
        ));
    }
}

void JobArena::stopThreads()
{
    _done = true;
    _block.notify_all();

    for (unsigned i = 0; i < _numThreads; ++i)
    {
        if (_threads[i].joinable())
        {
            _threads[i].join();
        }
    }

    _threads.clear();

    // Clear out the queue
    {
        Threading::ScopedMutexLock lock(_queueMutex);
        _queue.clear();
    }
}