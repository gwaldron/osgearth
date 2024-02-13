/**
 * weethreads
 * Copyright 2024 Pelican Mapping
 * MIT License
 */
#pragma once
#include <atomic>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <functional>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <type_traits>
#include <cstdlib>
#include <cfloat>

// MANDATORY: Somewhere in one of your .cpp files, use this macro to instantiate the singleton:
// WEETHREADS_INSTANCE;

// OPTIONAL: Define WEETHREADS_EXPORT if you want to use this library from multiple modules (DLLs)
#ifndef WEETHREADS_EXPORT
#define WEETHREADS_EXPORT
#endif

// OPTIONAL: Customize the namespace by defining WEETHREADS_NAMESPACE before including this file.
#ifndef WEETHREADS_NAMESPACE
#define WEETHREADS_NAMESPACE jobs
#endif

/**
* WeeThreads is an API for scheduling a task to run in the background.
*
* WeeThreads is header-only and has no dependencies aside from the STL.
*
* Usage:
*
*    Use this macro somewhere in your app. Put it in a .cpp file if you plan
     to use multiple modules, DLLs, etc.:
*
*    WEETHREADS_INSTANCE;
*
* Example: Spawn a job with no return value (fire and forget):
*
*    auto job = []() { std::cout << "Hello, world!" << std::endl; };
*    jobs::dispatch(job);
*
* Example: Spawn a job and get a future result:
*
*    auto job = [](jobs::cancelable&) { return 7; };
*    jobs::future<int> result = jobs::dispatch(job);
*    // later...
*    if (result.available())
*       std::cout << "Result = " << result.value() << std::endl;
*    else if (result.canceled())
*       std::cout << "Job was canceled" << std::endl;
*    else
*       // still running.... come back later
*
* Example: Spawn a job and wait for it to complete:
*
*    auto job = [url](jobs::cancelable&) { return fetch_data_from_network(url); };
*    auto result = jobs::dispatch(job);
*    auto value = result.join();
*
* Example: Spwan a job with some context information:
*
*    auto job = []() { std::cout << "Hello, world!" << std::endl; };
*    jobs::context context;
*    context.name = "My Job";
*    context.pool = jobs::get_pool("My Job Pool");
*    context.priority = []() { return 1.0f; };
*    jobs::dispatch(job, context);
*
* Example: Check for cancelation within a job:
*
*   auto job = [url](jobs::cancelable& state) {
*       std::string data;
*       if (!state.canceled())
*           data = fetch_data_from_network(url);
*       return data;
*   };
*
*   auto result = jobs::dispatch(job);
*   // if "result" goes out of scope, "state.canceled()" in the job will return true
*
* This SDK exists because existing solutions do not support two things we need:
* automatic job cancelation, and job prioritization. This system acheives
* cancelation by tracking the reference count of the shared result object contained
* in the Future object; if that reference count goes to one, it means that ONLY the
* scheduler knows about the job, and no one else is around to fetch its result.
* In this case, future.canceled() returns true. It's up to the task itself to
* check the cancelable& object if it wants to quit early.
*/
namespace WEETHREADS_NAMESPACE
{
    /**
    * Interface for something that can be canceled
    */
    class cancelable
    {
    public:
        virtual bool canceled() const { return false; }
    };

    namespace detail
    {
        /**
         * Event with a binary signaled state, for multi-threaded sychronization.
         *
         * The event has two states:
         *  "set" means that a call to wait() will not block;
         *  "unset" means that calls to wait() will block until another thread calls set().
         *
         * The event starts out unset.
         *
         * Typical usage: Thread A creates Thread B to run asynchronous code. Thread A
         * then calls wait(), which blocks Thread A. When Thread B is finished, it calls
         * set(). Thread A then wakes up and continues execution.
         *
         * NOTE: ALL waiting threads will wake up when the Event is cleared.
         */
        struct event
        {
        public:
            //! Construct a new event
            event() : _set(false) { }

            //! DTOR
            ~event() {
                _set = false;
                for (int i = 0; i < 255; ++i)  // workaround buggy broadcast
                    _cond.notify_all();
            }

            //! Block until the event is set, then return true.
            inline bool wait() {
                while (!_set) {
                    std::unique_lock<std::mutex> lock(_m);
                    if (!_set)
                        _cond.wait(lock);
                }
                return _set;
            }

            //! Block until the event is set or the timout expires.
            //! Return true if the event has set, otherwise false.
            template<typename T>
            inline bool wait(T timeout) {
                if (!_set) {
                    std::unique_lock<std::mutex> lock(_m);
                    if (!_set)
                        _cond.wait_for(lock, timeout);
                }
                return _set;
            }

            //! Block until the event is set; then reset it.
            inline bool waitAndReset() {
                std::unique_lock<std::mutex> lock(_m);
                if (!_set)
                    _cond.wait(lock);
                _set = false;
                return true;
            }

            //! Set the event state, causing any waiters to unblock.
            inline void set() {
                if (!_set) {
                    std::unique_lock<std::mutex> lock(_m);
                    if (!_set) {
                        _set = true;
                        _cond.notify_all();
                    }
                }
            }

            //! Reset (unset) the event state; new waiters will block until set() is called.
            inline void reset() {
                std::unique_lock<std::mutex> lock(_m);
                _set = false;
            }

            //! Whether the event state is set (waiters will not block).
            inline bool isSet() const {
                return _set;
            }

        protected:
            std::mutex _m; // do not use Mutex, we never want tracking
            std::condition_variable_any _cond;
            bool _set;
        };


        /**
         * Sempahore lets N users aquire it and then notifies when the
         * count goes back down to zero.
         */
        class semaphore
        {
        public:
            //! Acquire, increasing the usage count by one
            void acquire()
            {
                std::unique_lock<std::mutex> lock(_m);
                ++_count;
            }

            //! Release, decreasing the usage count by one.
            //! When the count reaches zero, joiners will be notified and
            //! the semaphore will reset to its initial state.
            void release()
            {
                std::unique_lock<std::mutex> lock(_m);
                _count = std::max(_count - 1, 0);
                if (_count == 0)
                    _cv.notify_all();
            }

            //! Reset to initialize state; this will cause a join to occur
            //! even if no acquisitions have taken place.
            void reset()
            {
                std::unique_lock<std::mutex> lock(_m);
                _count = 0;
                _cv.notify_all();
            }

            //! Current count in the semaphore
            std::size_t count() const
            {
                std::unique_lock<std::mutex> lock(_m);
                return _count;
            }

            //! Block until the semaphore count returns to zero.
            //! (It must first have left zero)
            //! Warning: this method will block forever if the count
            //! never reaches zero!
            void join()
            {
                std::unique_lock<std::mutex> lock(_m);
                while (_count > 0)
                    _cv.wait(lock);
            }

            //! Block until the semaphore count returns to zero, or
            //! the operation is canceled.
            //! (It must first have left zero)
            void join(cancelable* c)
            {
                _cv.wait_for(_m, std::chrono::seconds(1), [this, c]() {
                    return
                        (_count == 0) ||
                        (c && c->canceled());
                    }
                );
                _count = 0;
            }

        private:
            int _count = 0;
            std::condition_variable_any _cv;
            mutable std::mutex _m;
        };

#if __cplusplus >= 202000L
        template<typename F, typename...Args>
        using result_of_t = typename std::invoke_result<F, Args...>::type;
#else
        template<typename F>
        using result_of_t = typename std::result_of<F>::type;
#endif
    }


    /**
     * Future holds the future result of an asynchronous operation.
     *
     * Usage:
     *   Producer (usually an asynchronous function call) creates a future<T>
     *   (the promise of a future result) and immediately returns it. The Consumer
     *   then performs other work, and eventually (or immediately) checks available()
     *   for a result or canceled() for cancelation. If availabile() is true,
     *   Consumer calls value() to fetch the valid result.
     *
     *   As long as at least two equivalent Future object (i.e. Futures pointing to the
     *   same internal shared data) exist, the Future is considered valid. Once
     *   that count goes to one, the Future is either available (the value is ready)
     *   or empty (i.e., canceled or abandoned).
     */
    template<typename T>
    class future : public cancelable
    {
    private:
        // internal structure to track references to the result
        // One instance of this is shared among all Future instances
        // created from the copy constructor.
        struct shared_t
        {
            T _obj;
            mutable detail::event _ev;
        };

    public:
        //! Default constructor
        future()
        {
            _shared = std::make_shared<shared_t>();
        }

        //! Default copy constructor
        future(const future& rhs) = default;

        //! True is this Future is unused and not connected to any other Future
        bool empty() const
        {
            return !available() && _shared.use_count() == 1;
        }

        //! True if the promise was resolved and a result if available.
        bool available() const
        {
            return _shared->_ev.isSet();
        }

        //! True if a promise exists, but has not yet been resolved;
        //! Presumably the asynchronous task is still working.
        bool working() const
        {
            return !empty() && !available();
        }

        // cancelable interface
        bool canceled() const override
        {
            return empty();
        }

        //! Deference the result object. Make sure you check available()
        //! to check that the future was actually resolved; otherwise you
        //! will just get the default object.
        const T& value() const
        {
            return _shared->_obj;
        }

        //! Dereference this object to const pointer to the result.
        const T* operator -> () const
        {
            return &_shared->_obj;
        }

        //! Same as value(), but if the result is available will reset the
        //! future before returning the result object.
        T release()
        {
            bool avail = available();
            T result = value();
            if (avail)
                reset();
            return result;
        }

        //! Blocks until the result becomes available or the future is abandoned;
        //! then returns the result object.
        const T& join() const
        {
            while (
                !empty() &&
                !_shared->_ev.wait(std::chrono::milliseconds(1)));
            return value();
        }

        //! Blocks until the result becomes available or the future is abandoned
        //! or a cancelation flag is set; then returns the result object. Be sure to
        //! check canceled() after calling join() to see if the return value is valid.
        const T& join(cancelable* p) const
        {
            while (working() && (p == nullptr || !p->canceled()))
            {
                _shared->_ev.wait(std::chrono::milliseconds(1));
            }
            return value();
        }

        //! Blocks until the result becomes available or the future is abandoned
        //! or a cancelation flag is set; then returns the result object. Be sure to
        //! check canceled() after calling join() to see if the return value is valid.
        const T& join(const cancelable& p) const
        {
            return join(&p);
        }

        //! Release reference to a promise, resetting this future to its default state
        void abandon()
        {
            _shared.reset(new shared_t());
        }

        //! synonym for abandon.
        void reset()
        {
            abandon();
        }

        //! Resolve (fulfill) the promise with the provided result value.
        void resolve(const T& value)
        {
            _shared->_obj = value;
            _shared->_ev.set();
        }

        //! Resolve (fulfill) the promise with an rvalue
        void resolve(T&& value)
        {
            _shared->_obj = std::move(value);
            _shared->_ev.set();
        }

        //! Resolve (fulfill) the promise with a default result
        void resolve()
        {
            _shared->_ev.set();
        }

        //! The number of objects, including this one, that
        //! reference the shared container. If this method
        //! returns 1, that means this is the only object with
        //! access to the data. This method will never return zero.
        unsigned refs() const
        {
            return _shared.use_count();
        }

    private:
        std::shared_ptr<shared_t> _shared;
    };

    //! in the "promise/future" pattern, we use the same object for both,
    //! but here's an alias for clarity.
    template<class T> using promise = future<T>;

    /**
    * Include a jobgroup in a context to group together multiple jobs.
    * You can then call jobgroup::join() to wait for the whole group
    * to finish.
    */
    using jobgroup = detail::semaphore;

    /**
    * Context object you can pass to dispatch(...) to control aspects of
    * how the background task is run.
    */
    struct context
    {
        std::string name;
        class jobpool* pool = nullptr;
        std::function<float()> priority = {};
        jobgroup* group = nullptr;
    };

    /**
    * A priority-sorted collection of jobs that are running or waiting
    * to run in a thread pool.
    */
    class jobpool
    {
    public:
        /**
        * Metrics of a thread pool.
        */
        struct metrics_t
        {
            std::string name;
            std::atomic_uint concurrency = { 0u };
            std::atomic_uint pending = { 0u };
            std::atomic_uint running = { 0u };
            std::atomic_uint canceled = { 0u };
            std::atomic_uint total = { 0u };
        };

    public:
        //! Destroy
        ~jobpool()
        {
            stop_threads();
        }

        //! Name of this job pool
        const std::string& name() const
        {
            return _metrics.name;
        }

        metrics_t* metrics()
        {
            return &_metrics;
        }

        //! Set the concurrency of this job scheduler
        void set_concurrency(unsigned value)
        {
            value = std::max(value, 1u);
            if (_targetConcurrency != value)
            {
                _targetConcurrency = value;
                start_threads();
            }
        }

        //! Get the target concurrency (thread count) 
        unsigned concurrency() const
        {
            return _targetConcurrency;
        }

        //! Discard all queued jobs
        void cancel_all()
        {
            std::unique_lock<std::mutex> lock(_queueMutex);
            _queue.clear();
            _metrics.canceled += _metrics.pending;
            _metrics.pending = 0;
        }

        //! Schedule an asynchronous task on this scheduler
        //! Use job::dispatch to run jobs (usually no need to call this directly)
        //! @param delegate Function to execute
        //! @param context Job details
        void dispatch(std::function<bool()>& delegate, const context& context)
        {
            // If we have a group semaphore, acquire it BEFORE queuing the job
            if (context.group)
            {
                context.group->acquire();
            }

            if (_targetConcurrency > 0)
            {
                std::unique_lock<std::mutex> lock(_queueMutex);
                if (!_done)
                {
                    _queue.emplace_back(job{ context, delegate });
                    _metrics.pending++;
                    _metrics.total++;
                    _block.notify_one();
                }
            }
            else
            {
                // no threads? run synchronously.
                delegate();

                if (context.group)
                {
                    context.group->release();
                }
            }
        }

        //! Construct a new job pool.
        //! Do not call this directly - call getPool(name) instead.
        jobpool(const std::string& name, unsigned concurrency) :
            _targetConcurrency(concurrency)
        {
            _metrics.name = name;
            _metrics.concurrency = 0;
        }

        //! Pulls queued jobs and runs them in whatever thread run() is called from.
        //! Runs in a loop until _done is set.
        void run()
        {
            while (!_done)
            {
                job next;
                bool have_next = false;
                {
                    std::unique_lock<std::mutex> lock(_queueMutex);

                    _block.wait(lock, [this] {
                        return _queue.empty() == false || _done == true;
                        });

                    if (!_queue.empty() && !_done)
                    {
                        // Find the highest priority item in the queue.
                        // Note: We could use std::partial_sort or std::nth_element,
                        // but benchmarking proves that a simple brute-force search
                        // is always the fastest.
                        // (Benchmark: https://stackoverflow.com/a/20365638/4218920)
                        // Also note: it is indeed possible for the results of 
                        // priority() to change during the search. We don't care.
                        int index = -1;
                        float highest_priority = -FLT_MAX;
                        for (unsigned i = 0; i < _queue.size(); ++i)
                        {
                            float priority = _queue[i].ctx.priority != nullptr ?
                                _queue[i].ctx.priority() :
                                0.0f;

                            if (index < 0 || priority > highest_priority)
                            {
                                index = i;
                                highest_priority = priority;
                            }
                        }
                        if (index < 0)
                            index = 0;

                        next = std::move(_queue[index]);
                        have_next = true;

                        // move the last element into the empty position:
                        if (index < _queue.size() - 1)
                        {
                            _queue[index] = std::move(_queue.back());
                        }

                        // and remove the last element.
                        _queue.erase(_queue.end() - 1);
                    }
                }

                if (have_next)
                {
                    _metrics.running++;
                    _metrics.pending--;

                    auto t0 = std::chrono::steady_clock::now();

                    bool job_executed = next._delegate();

                    auto duration = std::chrono::steady_clock::now() - t0;

                    if (job_executed == false)
                    {
                        _metrics.canceled++;
                    }

                    // release the group semaphore if necessary
                    if (next.ctx.group != nullptr)
                    {
                        next.ctx.group->release();
                    }

                    _metrics.running--;
                }

                // See if we no longer need this thread because the
                // target concurrency has been reduced
                std::lock_guard<std::mutex> lock(_quitMutex);

                if (_targetConcurrency < _metrics.concurrency)
                {
                    _metrics.concurrency--;
                    break;
                }
            }
        }

        //! Spawn all threads in this scheduler
        inline void start_threads();

        //! Signall all threads to stop
        inline void stop_threads();

        //! Wait for all threads to exit (after calling stop_threads)
        inline void join_threads();


        struct job
        {
            context ctx;
            std::function<bool()> _delegate;

            bool operator < (const job& rhs) const
            {
                float lp = ctx.priority ? ctx.priority() : -FLT_MAX;
                float rp = rhs.ctx.priority ? rhs.ctx.priority() : -FLT_MAX;
                return lp < rp;
            }
        };

        std::string _name; // pool name
        std::vector<job> _queue; // queued operations to run asynchronously
        mutable std::mutex _queueMutex; // protect access to the queue
        mutable std::mutex _quitMutex; // protects access to _done
        std::atomic<unsigned> _targetConcurrency; // target number of concurrent threads in the pool
        std::condition_variable_any _block; // thread waiter block
        bool _done = false; // set to true when threads should exit
        std::vector<std::thread> _threads; // threads in the pool
        metrics_t _metrics; // metrics for this pool
    };

    class metrics
    {
    public:
        //! Total number of pending jobs across all schedulers
        int totalJobsPending() const
        {
            int count = 0;
            for (auto pool : _pools)
                count += pool->pending;
            return count;
        }

        //! Total number of running jobs across all schedulers
        int totalJobsRunning() const
        {
            int count = 0;
            for (auto pool : _pools)
                count += pool->running;
            return count;
        }

        //! Total number of canceled jobs across all schedulers
        int totalJobsCanceled() const
        {
            int count = 0;
            for (auto pool : _pools)
                count += pool->canceled;
            return count;
        }

        //! Total number of active jobs in the system
        int totalJobs() const
        {
            return totalJobsPending() + totalJobsRunning();
        }

        //! Gets a vector of all jobpool metrics structures.
        inline const std::vector<struct jobpool::metrics_t*> all()
        {
            return _pools;
        }

        std::vector<struct jobpool::metrics_t*> _pools;
    };

    /**
    * Runtime singleton object;
    * Declare with WEETHREADS_INSTANCE in one of your .cpp files.
    */
    namespace detail
    {
        struct runtime
        {
            inline runtime();

            inline void kill()
            {
                _alive = false;

                for (auto& pool : _pools)
                    if (pool)
                        pool->stop_threads();

                for (auto& pool : _pools)
                    if (pool)
                        pool->join_threads();
            }

            bool _alive = true;
            std::mutex _mutex;
            std::vector<std::string> _pool_names;
            std::vector<jobpool*> _pools;
            metrics _metrics;
            std::function<void(const char*)> _setThreadName;
        };
    }

    extern WEETHREADS_EXPORT detail::runtime& instance();

    //! Returns the job pool with the given name, creating a new one if it doesn't 
    //! already exist. If you don't specify a name, a default pool is used.
    inline jobpool* get_pool(const std::string& name = {})
    {
        std::lock_guard<std::mutex> lock(instance()._mutex);
        for (auto pool : instance()._pools)
        {
            if (pool->name() == name)
                return pool;
        }
        auto new_pool = new jobpool(name, 2u);
        instance()._pools.push_back(new_pool);
        instance()._metrics._pools.push_back(&new_pool->_metrics);
        new_pool->start_threads();
        return new_pool;
    }

    namespace detail
    {
        inline void pool_dispatch(std::function<bool()> delegate, const context& context)
        {
            auto pool = context.pool ? context.pool : get_pool({});
            if (pool)
                pool->dispatch(delegate, context);
        }
    }

    //! Dispatches a job with no return value. Fire and forget.
    //! @param task Function to run in a thread. Prototype is void(void).
    //! @param context Optional configuration for the asynchronous function call
    inline void dispatch(std::function<void()> task, const context& context = {})
    {
        auto delegate = [task]() mutable -> bool { task(); return true; };
        detail::pool_dispatch(delegate, context);
    }

    //! Dispatches a job and immediately returns a future result.
    //! @param task Function to run in a thread. Prototype is T(cancelable&)
    //! @param context Optional configuration for the asynchronous function call
    //! @param promise Optional user-supplied promise object
    //! @return Future result of the async function call
    template<typename FUNC, typename T = typename detail::result_of_t<FUNC(cancelable&)>>
    inline future<T> dispatch(FUNC task, const context& context = {}, future<T> promise = {})
    {
        std::function<bool()> delegate = [task, promise]() mutable
            {
                bool good = !promise.canceled();
                if (good)
                    promise.resolve(task(promise));
                return good;
            };

        detail::pool_dispatch(delegate, context);

        return promise;
    }


    inline void jobpool::start_threads()
    {
        _done = false;

        // Not enough? Start up more
        while (_metrics.concurrency < _targetConcurrency)
        {
            _metrics.concurrency++;

            _threads.push_back(std::thread([this]
                {
                    if (instance()._setThreadName)
                    {
                        instance()._setThreadName(_name.c_str());
                    }
                    run();
                }
            ));
        }
    }

    inline void jobpool::stop_threads()
    {
        _done = true;

        // Clear out the queue
        {
            std::unique_lock<std::mutex> lock(_queueMutex);

            // reset any group semaphores so that JobGroup.join()
            // will not deadlock.
            for (auto& queuedjob : _queue)
            {
                if (queuedjob.ctx.group != nullptr)
                {
                    queuedjob.ctx.group->reset();
                }
            }
            _queue.clear();

            // wake up all threads so they can exit
            _block.notify_all();
        }
    }

    //! Wait for all threads to exit (after calling stop_threads)
    inline void jobpool::join_threads()
    {
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

    //! Metrics for all job pool
    inline metrics* get_metrics()
    {
        return &instance()._metrics;
    }

    //! stop all threads, wait for them to exit, and shut down the system
    inline void shutdown()
    {
        instance().kill();
    }

    //! Whether the weethreads runtime is still alive (has not been shutdown)
    inline bool alive()
    {
        return instance()._alive;
    }

    //! Install a function that the SDK can use to set job pool thread names
    //! when it spawns them.
    inline void set_thread_name_function(std::function<void(const char*)> f)
    {
        instance()._setThreadName = f;
    }

    // internal
    inline detail::runtime::runtime()
    {
        std::atexit(shutdown);
    }

    // Use this macro ONCE in your application in a .cpp file to 
    // instaniate the weethreads runtime singleton.
#define WEETHREADS_INSTANCE \
    namespace WEETHREADS_NAMESPACE { \
        static detail::runtime runtime_singleton_instance; \
        detail::runtime& instance() { return runtime_singleton_instance; } \
    }
}
