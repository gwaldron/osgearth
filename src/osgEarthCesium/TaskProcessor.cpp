/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/
#include "TaskProcessor"
#include <osgEarth/Threading>
#include <osgEarth/Notify>

using namespace osgEarth;
using namespace osgEarth::Threading;
using namespace osgEarth::Cesium;

const std::string CESIUM_ARENA_NAME = "cesium";

TaskProcessor::TaskProcessor()
{
    jobs::get_pool(CESIUM_ARENA_NAME)->set_concurrency(8);
}

TaskProcessor::~TaskProcessor()
{
}

void TaskProcessor::shutdown()
{    
    // Wait for all jobs to finish
    auto metrics = jobs::get_pool(CESIUM_ARENA_NAME)->metrics();
    unsigned int totalJobs = metrics->pending + metrics->running;
    while (totalJobs != 0)
    {
        std::this_thread::yield();        
        totalJobs = metrics->pending + metrics->running;
    }
}

void TaskProcessor::startTask(std::function<void()> f)
{
    auto task = [this, f]() {
        f();
        return true;
    };
    jobs::context cx;
    cx.pool = jobs::get_pool(CESIUM_ARENA_NAME);
    jobs::dispatch(task, cx);
}