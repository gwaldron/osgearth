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
#include "Threading"
#include "Utils"
#include "Metrics"
#include <cstdlib>
#include <climits>
#include <mutex>

#ifdef _WIN32
#   include <Windows.h>
//#   include <processthreadsapi.h>
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

#ifdef OSGEARTH_MUTEX_CONTENTION_TRACKING

Mutex::Mutex() :
    _handle(nullptr),
    _metricsData(nullptr)
{
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
}

Mutex::Mutex(const std::string& name, const char* file, std::uint32_t line) :
    _name(name),
    _handle(nullptr),
    _metricsData(nullptr)
{
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
}

Mutex::~Mutex()
{
    if (_metricsData)
        delete static_cast<tracy::Lockable<std::mutex>*>(_handle);
    else
        delete static_cast<std::mutex*>(_handle);
}

void
Mutex::setName(const std::string& name)
{
    _name = name;
    if (_metricsData)
    {
        tracy::SourceLocationData* s = static_cast<tracy::SourceLocationData*>(_metricsData);
        s->function = _name.c_str();
    }
}

void
Mutex::lock()
{
    //if (_name.empty()) {
    //    volatile int x =0 ; // breakpoint for finding unnamed mutexes
    //}

    if (_metricsData)
        static_cast<tracy::Lockable<std::mutex>*>(_handle)->lock();
    else
        static_cast<std::mutex*>(_handle)->lock();
}

void
Mutex::unlock()
{
    if (_metricsData)
        static_cast<tracy::Lockable<std::mutex>*>(_handle)->unlock();
    else
        static_cast<std::mutex*>(_handle)->unlock();
}

bool
Mutex::try_lock()
{
    if (_metricsData)
        return static_cast<tracy::Lockable<std::mutex>*>(_handle)->try_lock();
    else
        return static_cast<std::mutex*>(_handle)->try_lock();
}

#endif // OSGEARTH_MUTEX_CONTENTION_TRACKING

//...................................................................

#ifdef OSGEARTH_MUTEX_CONTENTION_TRACKING

RecursiveMutex::RecursiveMutex() :
    _enabled(true),
    _metricsData(nullptr)
{
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
}

RecursiveMutex::RecursiveMutex(const std::string& name, const char* file, std::uint32_t line) :
    _name(name),
    _enabled(true),
    _metricsData(nullptr)
{
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
}

RecursiveMutex::~RecursiveMutex()
{
    if (_handle)
    {
        if (_metricsData)
            delete static_cast<tracy::Lockable<std::recursive_mutex>*>(_handle);
        else
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

    if (_metricsData)
    {
        tracy::SourceLocationData* s = static_cast<tracy::SourceLocationData*>(_metricsData);
        s->function = _name.c_str();
    }
}

void
RecursiveMutex::lock()
{
    if (_enabled)
    {
        if (_metricsData)
            static_cast<tracy::Lockable<std::recursive_mutex>*>(_handle)->lock();
        else
            static_cast<std::recursive_mutex*>(_handle)->lock();
    }
}

void
RecursiveMutex::unlock()
{
    if (_enabled)
    {
        if (_metricsData)
            static_cast<tracy::Lockable<std::recursive_mutex>*>(_handle)->unlock();
        else
            static_cast<std::recursive_mutex*>(_handle)->unlock();
    }
}

bool
RecursiveMutex::try_lock()
{
    if (_enabled)
    {
        if (_metricsData)
            return static_cast<tracy::Lockable<std::recursive_mutex>*>(_handle)->try_lock();
        else
            return static_cast<std::recursive_mutex*>(_handle)->try_lock();
    }
    else return true;
}

#endif // OSGEARTH_MUTEX_CONTENTION_TRACKING

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

Event::~Event()
{
    _set = false;
    for(int i=0; i<255; ++i)  // workaround buggy broadcast
        _cond.notify_all();
}

bool Event::wait()
{
    while(!_set)
    {
        std::unique_lock<std::mutex> lock(_m);
        if (!_set)
            _cond.wait(lock);
    }
    return _set;
}

bool Event::wait(unsigned timeout_ms)
{
    if (!_set)
    {
        std::unique_lock<std::mutex> lock(_m);
        if (!_set) // double check
        {
            _cond.wait_for(lock, std::chrono::milliseconds(timeout_ms));
        }
    }
    return _set;
}

bool Event::waitAndReset()
{
    std::unique_lock<std::mutex> lock(_m);
    if (!_set)
        _cond.wait(lock);
    _set = false;
    return true;
}

void Event::set()
{
    if (!_set)
    {
        std::unique_lock<std::mutex> lock(_m);
        if (!_set) {
            _set = true;
            _cond.notify_all();
        }
    }
}

void Event::reset()
{
    std::lock_guard<std::mutex> lock(_m);
    _set = false;
}


void
osgEarth::Threading::setThreadName(const std::string& name)
{
#if (defined _WIN32 && defined _WIN32_WINNT_WIN10 && defined _WIN32_WINNT && _WIN32_WINNT >= _WIN32_WINNT_WIN10) || (defined __CYGWIN__)
    wchar_t buf[256];
    mbstowcs(buf, name.c_str(), 256);

    //::SetThreadDescription(::GetCurrentThread(), buf);

    // Look up the address of the SetThreadDescription function rather than using it directly.
    typedef ::HRESULT(WINAPI* SetThreadDescription)(::HANDLE hThread, ::PCWSTR lpThreadDescription);
    auto set_thread_description_func = reinterpret_cast<SetThreadDescription>(::GetProcAddress(::GetModuleHandle("Kernel32.dll"), "SetThreadDescription"));
    if (set_thread_description_func)
    {
        set_thread_description_func(::GetCurrentThread(), buf);
    }

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

JobArena::JobArena(const std::string& name, unsigned concurrency, const Type& type) :
    _name(name),
    _targetConcurrency(concurrency),
    _type(type),
    _done(false),
    _queueMutex("OE.JobArena[" + name + "].queue"),
    _quitMutex("OE.JobArena[" + name + "].quit")
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

    if (_type == THREAD_POOL)
    {
        startThreads();
    }
}

JobArena::~JobArena()
{
    _metrics->free();

    if (_type == THREAD_POOL)
    {
        stopThreads();
    }
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
JobArena::get(const std::string& name_)
{
    ScopedMutexLock lock(_arenas_mutex);

    if (_arenas.empty())
    {
        std::atexit(JobArena::shutdownAll);
    }

    std::string name(name_.empty() ? "oe.default" : name_);

    std::shared_ptr<JobArena>& arena = _arenas[name];
    if (arena == nullptr)
    {
        auto iter = _arenaSizes.find(name);
        unsigned numThreads = iter != _arenaSizes.end() ? iter->second : OE_ARENA_DEFAULT_SIZE;

        arena = std::make_shared<JobArena>(name, numThreads);
    }
    return arena.get();
}

JobArena*
JobArena::get(const Type& type_)
{
    if (type_ == THREAD_POOL)
    {
        return get("oe.default");
    }

    ScopedMutexLock lock(_arenas_mutex);

    if (_arenas.empty())
    {
        std::atexit(JobArena::shutdownAll);
    }

    if (type_ == UPDATE_TRAVERSAL)
    {
        std::string name("oe.UPDATE");
        std::shared_ptr<JobArena>& arena = _arenas[name];
        if (arena == nullptr)
        {
            arena = std::make_shared<JobArena>(name, 0, type_);
        }
        return arena.get();
    }

    return nullptr;
}

void
JobArena::setConcurrency(unsigned value)
{
    value = std::max(value, 1u);

    if (_type == THREAD_POOL && _targetConcurrency != value)
    {
        _targetConcurrency = value;
        startThreads();
    }
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
            OE_SOFT_ASSERT_AND_RETURN(arena != nullptr, void());
            arena->setConcurrency(value);
        }
    }
}

void
JobArena::cancelAll()
{
    std::lock_guard<Mutex> lock(_queueMutex);
    _queue.clear();
    _metrics->numJobsCanceled += _metrics->numJobsPending;
    _metrics->numJobsPending = 0;
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

    if (_type == THREAD_POOL)
    {
        if (_targetConcurrency > 0)
        {
            std::lock_guard<Mutex> lock(_queueMutex);
            _queue.emplace_back(job, delegate, sema);
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

    else // _type == traversal
    {
        std::lock_guard<Mutex> lock(_queueMutex);
        _queue.emplace_back(job, delegate, sema);
        _metrics->numJobsPending++;
    }
}

void
JobArena::runJobs()
{
    // cap the number of jobs to run (applies to TRAVERSAL modes only)
    int jobsLeftToRun = INT_MAX;

    while (!_done)
    {
        QueuedJob next;

        bool have_next = false;
        {
            std::unique_lock<Mutex> lock(_queueMutex);
            
            if (_type == THREAD_POOL)
            {
                _block.wait(lock, [this] {
                    return _queue.empty() == false || _done == true;
                    });
            }
            else // traversal type
            {
                // Prevents jobs that re-queue themselves from running 
                // during the same traversal frame.
                if (jobsLeftToRun == INT_MAX)
                    jobsLeftToRun = _queue.size();

                if (_queue.empty() || jobsLeftToRun == 0)
                {
                    return;
                }
            }

            if (!_queue.empty() && !_done)
            {
                // Quickly find the highest priority item in the "queue"
                std::partial_sort(
                    _queue.rbegin(), _queue.rbegin() + 1, _queue.rend(),
                    [](const QueuedJob& lhs, const QueuedJob& rhs) {
                        return lhs._job.getPriority() > rhs._job.getPriority();
                    });

                next = std::move(_queue.back());
                have_next = true;
                _queue.pop_back();
            }
        }

        if (have_next)
        {
            _metrics->numJobsRunning++;
            _metrics->numJobsPending--;

            auto t0 = std::chrono::steady_clock::now();

            bool job_executed = next._delegate();

            auto duration = std::chrono::steady_clock::now() - t0;

            if (job_executed)
            {
                jobsLeftToRun--;

                if (_allMetrics._report != nullptr)
                {
                    if (duration >= _allMetrics._reportMinDuration)
                    {
                        _allMetrics._report(Metrics::Report(next._job, _name, duration));
                    }
                }
            }
            else
            {
                _metrics->numJobsCanceled++;
            }

            // release the group semaphore if necessary
            if (next._groupsema != nullptr)
            {
                next._groupsema->release();
            }

            _metrics->numJobsRunning--;
        }

        if (_type == THREAD_POOL)
        {
            // See if we no longer need this thread because the
            // target concurrency has been reduced
            ScopedMutexLock quitLock(_quitMutex);
            if (_targetConcurrency < _metrics->concurrency)
            {
                _metrics->concurrency--;
                break;
            }
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

                OE_THREAD_NAME(_name.c_str());

                runJobs();

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
        for (auto& queuedjob : _queue)
        {
            if (queuedjob._groupsema != nullptr)
            {
                queuedjob._groupsema->reset();
            }
        }
        _queue.clear();

        //while (_queue.empty() == false)
        //{
        //    if (_queue.back()._groupsema != nullptr)
        //    {
        //        _queue.back()._groupsema->reset();
        //    }
        //    _queue.pop_back();
        //}

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


JobArena::Metrics::Metrics() :
    _arenas(512),
    maxArenaIndex(-1),
    _report(nullptr),
    _reportMinDuration(0)
{
    // nop

    const char* report_us = ::getenv("OSGEARTH_JOB_REPORT_THRESHOLD");
    if (report_us)
    {
        _report = [](const Report& r)
        {
            static Mutex _mutex;
            ScopedMutexLock lock(_mutex);
            std::string jobname = r.job.getName().empty() ? "unknown" : r.job.getName();
            OE_INFO
                << "[Job] " << jobname
                << " (" << r.arena << ") "
                << std::fixed << std::setprecision(1)
                << 0.001f*(float)(std::chrono::duration_cast<std::chrono::microseconds>(r.duration).count()) << " ms" << std::endl;
        };

        _reportMinDuration = std::chrono::microseconds(as<int>(report_us, 132));

        OE_INFO << LC << "Job report min duration set to " << report_us << "us" << std::endl;
    }
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
