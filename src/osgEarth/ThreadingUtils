/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#ifndef OSGEARTH_THREADING_UTILS_H
#define OSGEARTH_THREADING_UTILS_H 1

#include <osgEarth/Common>
#include <OpenThreads/Condition>
#include <OpenThreads/Mutex>
#include <OpenThreads/Thread>
#include <osg/ref_ptr>
#include <set>
#include <map>

#define USE_CUSTOM_READ_WRITE_LOCK 1

namespace osgEarth { namespace Threading
{   
    typedef OpenThreads::Mutex Mutex;
    typedef OpenThreads::ScopedLock<OpenThreads::Mutex> ScopedMutexLock;
    typedef OpenThreads::Thread Thread;

    /**
     * Gets the unique ID of the running thread. Use this instead of
     * OpenThreads::Thread::CurrentThread, which only works reliably on
     * threads created with the OpenThreads framework
     */
    extern OSGEARTH_EXPORT unsigned getCurrentThreadId();



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
     */
    class OSGEARTH_EXPORT Event 
    {
    public:
        //! Construct a new event
        Event();

        //! DTOR
        ~Event();

        //! Block until the event is set, then return true if set, false on error.
        bool wait();

        //! Like wait(), but also returns false on timeout.
        bool wait(unsigned timeout_ms);

        //! Like wait(), but resets the state before returning.
        bool waitAndReset();

        //! Set the event state, causing any waiters to unblock.
        void set();

        //! Reset (unset) the event state; new waiters will block until set() is called.
        void reset();

        //! Whether the event state is set (waiters will not block).
        inline bool isSet() const { return _set; }

    protected:
        OpenThreads::Mutex _m;
        OpenThreads::Condition _cond;
        bool _set;
    };

    /** Wraps an Event in a Referenced. */
    class OSGEARTH_EXPORT RefEvent : public Event, public osg::Referenced
    {
    };

    /**
     * Same as Event, but set() must be called N times before unblocking
     * waiting thread(s).
     */
    class OSGEARTH_EXPORT MultiEvent 
    {
    public:
        //! Construct a multi-event with the number of setters.
        MultiEvent(int num =1);

        //! DTOR
        ~MultiEvent();

        //! Block the calling thread until all N set()s are called.
        bool wait();

        //! Same as wait(), but resets the state after returning.
        bool waitAndReset();

        //! Adds one signal to the event; when all signals are set, waiters will unblock.
        void set();
        void notify() { set(); }

        //! Resets the state.
        void reset();

    protected:
        OpenThreads::Mutex _m;
        OpenThreads::Condition _cond;
        int _set, _num;
    };

    /**
     * Future is the consumer-side interface to an asynchronous operation.
     *
     * Usage: 
     *   Producer (usually an asynchronous function call) creates a Promise<Object>
     *   and immediately returns promise.getFuture(). The Consumer then performs other
     *   work, and eventually (or immediately) called Future.get() or Future.release().
     *   Either call will block until the asynchronous operation is complete and the
     *   result in Future is available.
     */
    template<typename T>
    class Future
    {
    private:
        // internal structure to track referenced to the result
        struct RefPtrRef : public osg::Referenced {
            RefPtrRef(T* obj = 0L) : _obj(obj) { }
            osg::ref_ptr<T> _obj;
        };

    public:
        //! Blank CTOR
        Future() {
            _ev = new RefEvent();
            _objRef = new RefPtrRef();
        }

        //! Copy CTOR
        Future(const Future& rhs) : _ev(rhs._ev), _objRef(rhs._objRef.get()) { }

        //! True if the promise was resolved and a result if available.
        bool isAvailable() const {
            return _ev->isSet();
        }

        //! True if the Promise that generated this Future no longer exists.
        bool isAbandoned() const {
            return _objRef->referenceCount() == 1;
        }

        //! The result value; blocks until it is available (or abandonded) and then returns it.
        T* get() {
            while(!_ev->wait(1000u))
                if (isAbandoned()) return 0L;
            return _objRef->_obj.get();
        }

        //! The result value; blocks until available (or abandoned) and returns it; then resets to initial state.
        T* release() {
            while(!_ev->wait(1000u))
                if (isAbandoned()) return 0L;
            T* out = _objRef->_obj.release();
            _ev->reset();
            return out;
        }

    private:
        osg::ref_ptr<RefEvent> _ev;
        osg::ref_ptr<RefPtrRef> _objRef;
        template<typename U> friend class Promise;
    };
    
    /**
     * Promise is the producer-side interface to an asynchronous operation.
     *
     * Usage: The code that initiates an asychronous operation creates a Promise
     *   object, dispatches the asynchronous code, and immediately returns 
     *   Promise.getFuture(). The caller can then call future.get() to block until
     *   the result is available.
     */
    template<typename T>
    class Promise
    {
    public:
        //! This promise's future result.
        const Future<T> getFuture() const { return _future; }

        //! Resolve (fulfill) the promise with the provided result value.
        void resolve(T* value) {
            _future._objRef->_obj = value;
            _future._ev->set();
        }

        //! True if the promise is resolved and the Future holds a valid result.
        bool isResolved() const {
            return _future._ev->isSet();
        }

        //! True is there are no Future objects waiting on this Promise.
        bool isAbandoned() const {
            return _future._objRef->referenceCount() == 1;
        }

    private:
        Future<T> _future;
    };
    
#ifdef USE_CUSTOM_READ_WRITE_LOCK

    /**
     * Custom read/write lock. The read/write lock in OSG can unlock mutexes from a different
     * thread than the one that locked them - this can hang the thread in Windows.
     *
     * Adapted from:
     * http://www.codeproject.com/KB/threads/ReadWriteLock.aspx
     */
    class OSGEARTH_EXPORT ReadWriteMutex
    {
    public:
        //! Construct a read/write mutex.
        ReadWriteMutex();

        //! Lock for reading. Multiple threads can take a read lock simultaneously.
        void readLock();

        //! Unlock for reading.
        void readUnlock();

        //! Lock for writing. This is exclusive; no other read OR write locks may be taken.
        void writeLock();

        //! Release a write lock.
        void writeUnlock();

    protected:

        void incrementReaderCount();
        void decrementReaderCount();

    private:
        int    _readerCount;
        Mutex  _lockWriterMutex;
        Mutex  _readerCountMutex;
        Event  _noWriterEvent;
        Event  _noReadersEvent;
    };


    struct ScopedWriteLock
    {
        ScopedWriteLock( ReadWriteMutex& lock ) : _lock(lock) { _lock.writeLock(); }
        ~ScopedWriteLock() { _lock.writeUnlock(); }
    protected:
        ReadWriteMutex& _lock;
    };

    struct ScopedReadLock
    {
        ScopedReadLock( ReadWriteMutex& lock ) : _lock(lock) { _lock.readLock(); }
        ~ScopedReadLock() { _lock.readUnlock(); }
    protected:
        ReadWriteMutex& _lock;
    };

#else

    typedef OpenThreads::ReadWriteMutex  ReadWriteMutex;
    typedef OpenThreads::ScopedWriteLock ScopedWriteLock;
    typedef OpenThreads::ScopedReadLock  ScopedReadLock;

#endif

} } // namepsace osgEarth::Threading


#endif // OSGEARTH_THREADING_UTILS_H
