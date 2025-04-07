/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgEarth/catch.hpp>
#include <osgEarth/Threading>
#include <thread>

using namespace osgEarth;

#if 0
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
            mutex.read_lock();
            readLock1 = true;

            // Wait for the write lock to happen in thread 2
            while (!attemptingWriteLock)
            {
                std::this_thread::yield();
            }

            // The write lock is being attempted, sleep for awhile to make sure it actually tries to get the write lock.
            std::this_thread::sleep_for(std::chrono::seconds(2));

            // Take a second read lock
            mutex.read_lock();
            readLock2 = true;

            // Unlock both of our read locks
            mutex.read_unlock();
            mutex.read_unlock();
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
                std::this_thread::yield();
            }

            // Tell the first thread we are attempting a write lock so it can try to grab it's second read lock.
            attemptingWriteLock = true;

            // Try to get the write lock
            mutex.write_lock();
            writeLock = true;
            mutex.write_unlock();
        }
    };
}

// Disabled temporarily b/c it's breaking the Travis build for some reason.  Works fine on regular machines.

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
#endif