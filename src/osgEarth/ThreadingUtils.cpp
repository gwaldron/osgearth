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
#include <osgEarth/ThreadingUtils>

#ifdef _WIN32
    extern "C" unsigned long __stdcall GetCurrentThreadId();
#elif defined(__APPLE__) || defined(__LINUX__) || defined(__FreeBSD__) || defined(__FreeBSD_kernel__) || defined(__ANDROID__)
#   include <unistd.h>
#   include <sys/syscall.h>
#else
#   include <pthread.h>
#endif

using namespace osgEarth::Threading;

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
