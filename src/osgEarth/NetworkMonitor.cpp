/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2018 Pelican Mapping
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

#include <osgEarth/NetworkMonitor>
#include <osgEarth/ThreadingUtils>

using namespace osgEarth;
using namespace osgEarth::Contrib;

static NetworkMonitor::Requests s_requests;
static Threading::Mutex s_requestsMutex;
static unsigned long s_requestId = 0;

#define LC "[NetworkMonitor] "

unsigned long NetworkMonitor::begin(const std::string& uri, const std::string& status)
{
    Threading::ScopedMutexLock lock(s_requestsMutex);
    Request req(uri, status);
    unsigned long id = s_requestId++;
    s_requests[id] = req;
    return id;
}

void NetworkMonitor::end(unsigned long handle, const std::string& status)
{
    Threading::ScopedMutexLock lock(s_requestsMutex);
    auto req = s_requests.find(handle);
    if (req == s_requests.end())
    {
        OE_WARN << LC << "Missing handle " << handle << std::endl;
        return;
    }
    Request& r = req->second;
    r.status = status;
    r.endTime = osg::Timer::instance()->tick();
    r.isComplete = true;
}

void NetworkMonitor::getRequests(Requests& out)
{
    Threading::ScopedMutexLock lock(s_requestsMutex);
    out = s_requests;
}

