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

#include <osgEarth/NetworkMonitor>
#include <osgEarth/Threading>
#include <osgDB/fstream>
#include <iomanip>

using namespace osgEarth;

namespace
{
    static NetworkMonitor::Requests s_requests;
    static NetworkMonitor::URICount s_counts;
    osgEarth::Threading::ReadWriteMutex s_requestsMutex("NetworkMonitor(OE)");
    static unsigned long s_requestId = 0;
    static bool s_enabled = false;
    static std::unordered_map<unsigned int, std::string> s_requestLayer;
}

#define LC "[NetworkMonitor] "

unsigned long NetworkMonitor::begin(const std::string& uri, const std::string& status, const std::string& type)
{
    if (s_enabled)
    {
        osgEarth::Threading::ScopedWriteLock lock(s_requestsMutex);
        Request req(uri, status);
        req.layer = s_requestLayer[osgEarth::Threading::getCurrentThreadId()];
        req.type = type;
        req.count = ++s_counts.insert(std::make_pair(uri, 0u)).first->second;

        unsigned long id = s_requestId++;
        s_requests[id] = req;
        return id;
    }
    return 0;
}

void NetworkMonitor::end(unsigned long handle, const std::string& status)
{
    if (s_enabled)
    {
        osgEarth::Threading::ScopedWriteLock lock(s_requestsMutex);
        NetworkMonitor::Requests::iterator req = s_requests.find(handle);
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
}

void NetworkMonitor::getRequests(Requests& out)
{
    osgEarth::Threading::ScopedReadLock lock(s_requestsMutex);
    out = s_requests;
}

bool NetworkMonitor::getEnabled()
{
    return s_enabled;
}

void NetworkMonitor::setEnabled(bool enabled)
{
    s_enabled = enabled;
}

void NetworkMonitor::clear()
{
    osgEarth::Threading::ScopedWriteLock lock(s_requestsMutex);
    s_requests.clear();
    s_counts.clear();
}

void NetworkMonitor::saveCSV(Requests& requests, const std::string& filename)
{
    std::ofstream out(filename.c_str());
    out << "URI, Duration, Start, End, Layer, Type, Status" << std::endl;

    if (!requests.empty())
    {
        out << std::fixed << std::setprecision(2);
        double startTime = requests.begin()->second.startTime;
        for (Requests::iterator itr = requests.begin(); itr != requests.end(); ++itr)
        {
            double startMS = osg::Timer::instance()->delta_m(startTime, itr->second.startTime);
            double endMS = 0.0;
            if (itr->second.isComplete)
            {
                endMS = osg::Timer::instance()->delta_m(startTime, itr->second.endTime);
            }
            else
            {
                endMS = osg::Timer::instance()->delta_m(startTime, osg::Timer::instance()->tick());
            }

            out << itr->second.uri << ", "
                << itr->second.getDuration() << ", "
                << startMS << ", "
                << endMS << ", "
                << itr->second.layer << ", "
                << itr->second.type << ", "
                << itr->second.status
                << std::endl;
        }
    }
    out.close();
}

void NetworkMonitor::setRequestLayer(const std::string& name)
{
    osgEarth::Threading::ScopedWriteLock lock(s_requestsMutex);
    s_requestLayer[osgEarth::Threading::getCurrentThreadId()] = name;
}

std::string NetworkMonitor::getRequestLayer()
{
    osgEarth::Threading::ScopedReadLock lock(s_requestsMutex);
    return s_requestLayer[osgEarth::Threading::getCurrentThreadId()];
}

