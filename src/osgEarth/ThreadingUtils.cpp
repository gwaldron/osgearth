/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#else
#   include <unistd.h>
#   include <sys/syscall.h>
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
#else
  return (unsigned)::syscall(SYS_gettid);
#endif
}
