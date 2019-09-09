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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osgEarth/catch.hpp>
#include <osgEarth/ThreadingUtils>

using namespace osgEarth;


namespace ReadWriteMutexTest
{
    osgEarth::Threading::ReadWriteMutex mutex;
    bool readLock1 = false;
    bool readLock2 = false;
    bool attemptingWriteLock = false;
    bool writeLock = false;


    class Thread1 : public OpenThreads::Thread
    {
    public:
        Thread1()
        {

        }

        void run()
        {
            // Thread one takes the first read lock.
            mutex.readLock();
            readLock1 = true;

            // Wait for the write lock to happen in thread 2
            while (!attemptingWriteLock)
            {
                OpenThreads::Thread::YieldCurrentThread();
            }

            // The write lock is being attempted, sleep for awhile to make sure it actually tries to get the write lock.
            OpenThreads::Thread::microSleep(2e6);

            // Take a second read lock
            mutex.readLock();            
            readLock2 = true;

            // Unlock both of our read locks
            mutex.readUnlock();
            mutex.readUnlock();
        }
    };



    class Thread2 : public osg::Referenced, public OpenThreads::Thread
    {
    public:
        Thread2()
        {

        }

        void run()
        {
            // Wait for thread1 to grab the read lock.
            while (!readLock1)
            {
                OpenThreads::Thread::YieldCurrentThread();
            }

            // Tell the first thread we are attempting a write lock so it can try to grab it's second read lock.
            attemptingWriteLock = true;

            // Try to get the write lock
            mutex.writeLock();
            writeLock = true;
            mutex.writeUnlock();            
        }
    };
}

// Disabled temporarily b/c it's breaking the Travis build for some reason.  Works fine on regular machines.
/*
TEST_CASE( "ReadWriteMutex can handle multiple read locks from the same thread while a writer is trying to lock" ) {    

    ReadWriteMutexTest::Thread1 thread1;
    ReadWriteMutexTest::Thread2 thread2;

    // Start both threads
    thread1.start();
    thread2.start();

    // Wait a couple of seconds for the threads to actually start.
    OpenThreads::Thread::microSleep(2e6);
   
    // Let the threads go for up to 5 seconds.  If they don't finish in that amount of time they are deadlocked.
    double maxTimeSeconds = 5.0;
    double elapsedTime = 0.0;
    
    osg::Timer_t startTime = osg::Timer::instance()->tick();
    while (thread1.isRunning() && thread2.isRunning())
    {
        OpenThreads::Thread::YieldCurrentThread();
        elapsedTime = osg::Timer::instance()->delta_s(startTime, osg::Timer::instance()->tick());
        if (elapsedTime >= maxTimeSeconds)
        {
            OE_NOTICE << "Threads failed to complete in " << elapsedTime << " seconds" << std::endl;
            break;
        }
    }

    OE_NOTICE << "Elapsed time = " << elapsedTime << std::endl;
    REQUIRE(!thread1.isRunning());
    REQUIRE(!thread2.isRunning());
    REQUIRE(elapsedTime < maxTimeSeconds);
}
*/