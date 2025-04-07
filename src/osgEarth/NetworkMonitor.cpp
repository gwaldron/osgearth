/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */

#include <osgEarth/NetworkMonitor>
#include <osgEarth/Threading>
#include <osgDB/fstream>
#include <iomanip>
#include <thread>

using namespace osgEarth;

namespace
{
    NetworkMonitor::Requests s_requests;
    NetworkMonitor::URICount s_counts;
    osgEarth::Threading::ReadWriteMutex s_requestsMutex;
    unsigned long s_requestId = 0;
    bool s_enabled = false;
    std::unordered_map<std::thread::id, std::string> s_requestLayer;
}

#define LC "[NetworkMonitor] "

unsigned long NetworkMonitor::begin(const std::string& uri, const std::string& status, const std::string& type)
{
    if (s_enabled)
    {
        osgEarth::Threading::ScopedWriteLock lock(s_requestsMutex);
        Request req(uri, status);
        req.layer = s_requestLayer[std::this_thread::get_id()];
        req.type = type;
        req.count = ++s_counts.insert(std::make_pair(uri, 0u)).first->second;

        unsigned long id = s_requestId++;
        s_requests[id] = req;
        return id;
    }
    return 0;
}

void NetworkMonitor::end(unsigned long handle, const std::string& status, const std::string& detail)
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
        r.detail = detail;
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
    osgEarth::Threading::ScopedWriteLock unique_lock(s_requestsMutex);
    s_requestLayer[std::this_thread::get_id()] = name;
}

std::string NetworkMonitor::getRequestLayer()
{
    osgEarth::Threading::ScopedReadLock shared_lock (s_requestsMutex);
    return s_requestLayer[std::this_thread::get_id()];
}

