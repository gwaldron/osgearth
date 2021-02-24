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
#include "Utils"
#include "Metrics"
#include <cstdlib>

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

unsigned osgEarth::Threading::getConcurrency()
{
    int value = std::thread::hardware_concurrency();
    return value > 0 ? (unsigned)value : 4u;
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
    if (!_set)
    {
        std::unique_lock<Mutex> lock(_m);
        if (!_set)
            _cond.wait(lock);
    }
    return true;
}

bool Event::wait(unsigned timeout_ms)
{
    if (!_set)
    {
        std::unique_lock<Mutex> lock(_m);
        if (!_set) // double check
        {
            std::cv_status result = _cond.wait_for(lock, std::chrono::milliseconds(timeout_ms));
            return result == std::cv_status::no_timeout ? true : false;
        }
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
    if (!_set)
    {
        std::unique_lock<Mutex> lock(_m);
        if (!_set) {
            _set = true;
            _cond.notify_all();
        }
    }
}

void Event::reset()
{
    std::lock_guard<Mutex> lock(_m);
    _set = false;
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
#define LC "[Semaphore]"

Semaphore::Semaphore() :
    _count(0),
    _m("oe.Semaphore")
{
    //nop
}

Semaphore::Semaphore(const std::string& name) :
    _count(0),
    _m(name)
{
    //nop
}

void
Semaphore::acquire()
{
    ScopedMutexLock lock(_m);
    ++_count;
}

void
Semaphore::release()
{
    ScopedMutexLock lock(_m);
    _count = std::max(_count - 1, 0);
    if (_count == 0)
        _cv.notify_all();
}

void
Semaphore::reset()
{
    ScopedMutexLock lock(_m);
    _count = 0;
    _cv.notify_all();
}

std::size_t
Semaphore::count() const
{
    ScopedMutexLock lock(_m);
    return _count;
}

void
Semaphore::join()
{
    ScopedMutexLock lock(_m);
    _cv.wait(
        _m, 
        [this]()
        {
            return _count == 0;
        }
    );
}

void
Semaphore::join(Cancelable* cancelable)
{
    ScopedMutexLock lock(_m);
    _cv.wait_for(
        _m,
        std::chrono::seconds(1),
        [this, cancelable]() {
            return
                (_count == 0) ||
                (cancelable && cancelable->isCanceled());
        }
    );
    _count = 0;
}


#undef LC
#define LC "[JobGroup]"

JobGroup::JobGroup() :
    _sema(std::make_shared<Semaphore>())
{
    //nop
}

JobGroup::JobGroup(const std::string& name) :
    _sema(std::make_shared<Semaphore>(name))
{
    //nop
}

void
JobGroup::join()
{
    if (_sema != nullptr && _sema.use_count() > 1)
    {
        _sema->join();
    }
}

void
JobGroup::join(Cancelable* cancelable)
{
    if (_sema != nullptr && _sema.use_count() > 1)
    {
        _sema->join(cancelable);
    }
}


#undef LC
#define LC "[JobArena] "

// JobArena statics:
Mutex JobArena::_arenas_mutex("OE:JobArena");
std::unordered_map<std::string, std::shared_ptr<JobArena>> JobArena::_arenas;
std::unordered_map<std::string, unsigned> JobArena::_arenaSizes;
std::string JobArena::_defaultArenaName = "oe.default";
JobArena::Metrics JobArena::_allMetrics;

#define OE_ARENA_DEFAULT_SIZE 2u

JobArena::JobArena(const std::string& name, unsigned concurrency) :
    _name(name),
    _targetConcurrency(concurrency),
    _done(false),
    _queueMutex("OE.JobArena[" + name + "]")
{
    // find a slot in the stats
    int new_index = -1;
    for (int i = 0; i < 512 && new_index < 0; ++i)
        if (_allMetrics._arenas[i].active == false)
            new_index = i;
    _metrics = &_allMetrics._arenas[new_index];
    _metrics->arenaName = name;
    _metrics->concurrency = 0;
    _metrics->active = true;
    if (new_index >= _allMetrics.maxArenaIndex)
        _allMetrics.maxArenaIndex = new_index;

    startThreads();
}

JobArena::~JobArena()
{
    _metrics->free();

    stopThreads();
}

const std::string&
JobArena::defaultArenaName()
{
    return _defaultArenaName;
}

void
JobArena::shutdownAll()
{
    ScopedMutexLock lock(_arenas_mutex);
    OE_INFO << LC << "Shutting down all job arenas." << std::endl;
    _arenas.clear();
}

JobArena*
JobArena::get(const std::string& name)
{
    ScopedMutexLock lock(_arenas_mutex);
    
    if (_arenas.empty())
    {
        std::atexit(JobArena::shutdownAll);
    }

    std::shared_ptr<JobArena>& arena = _arenas[name];
    if (arena == nullptr)
    {
        auto iter = _arenaSizes.find(name);
        unsigned numThreads = iter != _arenaSizes.end() ? iter->second : OE_ARENA_DEFAULT_SIZE;
        
        arena = std::make_shared<JobArena>(name, numThreads);
    }
    return arena.get();
}

void
JobArena::setConcurrency(unsigned value)
{
    value = std::max(value, 1u);
    _targetConcurrency = value;
    startThreads();
}
void
JobArena::setConcurrency(const std::string& name, unsigned value)
{
    // this method exists so you can set an arena's concurrency
    // before the arena is actually created

    value = std::max(value, 1u);

    ScopedMutexLock lock(_arenas_mutex);
    if (_arenaSizes[name] != value)
    {
        _arenaSizes[name] = value;

        auto iter = _arenas.find(name);
        if (iter != _arenas.end())
        {
            std::shared_ptr<JobArena> arena = iter->second;
            OE_SOFT_ASSERT_AND_RETURN(arena != nullptr, __func__, );
            arena->setConcurrency(value);
        }
    }
}

void
JobArena::dispatch(
    const Job& job,
    Delegate& delegate)
{
    // If we have a group semaphore, acquire it BEFORE queuing the job
    JobGroup* group = job.getGroup();
    std::shared_ptr<Semaphore> sema = group ? group->_sema : nullptr;
    if (sema)
    {
        sema->acquire();
    }

    if (_targetConcurrency > 0)
    {
        std::lock_guard<Mutex> lock(_queueMutex);
        _queue.emplace(job, delegate, sema);
        _metrics->numJobsPending++;
        _block.notify_one();
    }

    else
    {
        // no threads? run synchronously.
        delegate();

        if (sema)
        {
            sema->release();
        }
    }
}

void
JobArena::startThreads()
{
    _done = false;

    OE_INFO << LC << "Arena \"" << _name << "\" concurrency=" << _targetConcurrency << std::endl;

    // Not enough? Start up more
    while(_metrics->concurrency < _targetConcurrency)
    {
        _threads.push_back(std::thread([this]
            {
                //OE_INFO << LC << "Arena \"" << _name << "\" starting thread " << std::this_thread::get_id() << std::endl;
                _metrics->concurrency++;

                OE_THREAD_NAME(std::string("oe.arena[" + _name + "]").c_str());

                while (!_done)
                {
                    QueuedJob next;

                    bool have_next = false;
                    {
                        std::unique_lock<Mutex> lock(_queueMutex);

                        _block.wait(lock, [this] {
                            return _queue.empty() == false || _done == true;
                        });

                        if (!_queue.empty() && !_done)
                        {
                            next = std::move(_queue.top());
                            have_next = true;
                            _queue.pop();
                        }
                    }

                    if (have_next)
                    {
                        _metrics->numJobsRunning++;
                        _metrics->numJobsPending--;

                        bool job_executed = next._delegate();

                        if (!job_executed)
                        {
                            _metrics->numJobsCanceled++;
                            //OE_INFO << LC << "Job canceled" << std::endl;
                        }

                        // release the group semaphore if necessary
                        if (next._groupsema != nullptr)
                        {
                            next._groupsema->release();
                        }

                        _metrics->numJobsRunning--;
                    }

                    // See if we no longer need this thread because the
                    // target concurrency has been reduced
                    ScopedMutexLock quitLock(_quitMutex);
                    if (_targetConcurrency < _metrics->concurrency)
                    {
                        _metrics->concurrency--;
                        break;
                    }
                }

                // exit thread here
                //OE_INFO << LC << "Thread " << std::this_thread::get_id() << " exiting" << std::endl;
            }
        ));
    }
}

void JobArena::stopThreads()
{
    _done = true;

    // Clear out the queue
    {
        std::lock_guard<Mutex> lock(_queueMutex);

        // reset any group semaphores so that JobGroup.join()
        // will not deadlock.
        while (_queue.empty() == false)
        {
            if (_queue.top()._groupsema != nullptr)
            {
                _queue.top()._groupsema->reset();
            }
            _queue.pop();
        }

        // wake up all threads so they can exit
        _block.notify_all();
    }

    // wait for them to exit
    for (unsigned i = 0; i < _threads.size(); ++i)
    {
        if (_threads[i].joinable())
        {
            _threads[i].join();
        }
    }

    _threads.clear();
}

const JobArena::Metrics::Arena&
JobArena::Metrics::arena(int index) const
{
    return _arenas[index];
}

int
JobArena::Metrics::totalJobsPending() const
{
    int count = 0;
    for (int i = 0; i <= maxArenaIndex; ++i)
        if (arena(i).active)
            count += arena(i).numJobsPending;
    return count;
}

int
JobArena::Metrics::totalJobsRunning() const
{
    int count = 0;
    for (int i = 0; i <= maxArenaIndex; ++i)
        if (arena(i).active)
            count += arena(i).numJobsRunning;
    return count;
}

int
JobArena::Metrics::totalJobsCanceled() const
{
    int count = 0;
    for (int i = 0; i <= maxArenaIndex; ++i)
        if (arena(i).active)
            count += arena(i).numJobsCanceled;
    return count;
}
