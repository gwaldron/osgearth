/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#include "TaskProcessor"
#include <osgEarth/Threading>

using namespace osgEarth;
using namespace osgEarth::Threading;
using namespace osgEarth::Cesium;

const std::string CESIUM_ARENA_NAME = "cesium";

TaskProcessor::TaskProcessor()
{
    JobArena::get(CESIUM_ARENA_NAME)->setConcurrency(8);
}

TaskProcessor::~TaskProcessor()
{
}

void TaskProcessor::shutdown()
{    
    // Wait for all jobs to finish
    auto metrics = JobArena::get(CESIUM_ARENA_NAME)->metrics();
    unsigned int totalJobs = metrics->numJobsPending + metrics->numJobsRunning;
    while (totalJobs != 0)
    {
        std::this_thread::yield();        
        totalJobs = metrics->numJobsPending + metrics->numJobsRunning;
    }
}

void TaskProcessor::startTask(std::function<void()> f)
{
    auto delegate = [this, f](Cancelable*) {
        f();
    };
    Job job;
    job.setArena(CESIUM_ARENA_NAME);
    job.dispatch(delegate);
}