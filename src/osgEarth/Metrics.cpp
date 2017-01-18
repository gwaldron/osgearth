/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
#include <osgEarth/Metrics>
#include <osgEarth/ThreadingUtils>

using namespace osgEarth;


namespace
{
    static osg::ref_ptr< MetricsBackend > s_metrics_backend;

    class MetricsStartup
    {
    public:
        MetricsStartup()
        {
            const char* metricsFile = ::getenv("OSGEARTH_METRICS_FILE");
            if (metricsFile)
            {
                Metrics::setMetricsBackend(new ChromeMetricsBackend(std::string(metricsFile)));
            }
        }

        ~MetricsStartup()
        {
            Metrics::setMetricsBackend(0);
        }
    };

    static MetricsStartup s_metricsStartup;
}

void Metrics::begin(const std::string& name, const Config& args)
{
    if (s_metrics_backend)
    {
        s_metrics_backend->begin(name, args);
    }
}

void Metrics::end(const std::string& name)
{
    if (s_metrics_backend)
    {
        s_metrics_backend->end(name);
    }
}

void Metrics::counter(const std::string& name, double value)
{
    if (s_metrics_backend)
    {
        s_metrics_backend->counter(name, value);
    }
}

void Metrics::setMetricsBackend(MetricsBackend* backend)
{
    s_metrics_backend = backend;
}

ChromeMetricsBackend::ChromeMetricsBackend(const std::string& filename):
_firstEvent(true)
{
    _metricsFile.open(filename.c_str(), std::ios::out);
    _metricsFile << "[";
}

ChromeMetricsBackend::~ChromeMetricsBackend()
{
     OpenThreads::ScopedLock< OpenThreads::Mutex > lk(_mutex);
    _metricsFile << "]";
    _metricsFile.close();
}

void ChromeMetricsBackend::begin(const std::string& name, const Config& args)
{
    OpenThreads::ScopedLock< OpenThreads::Mutex > lk(_mutex);
    if (_firstEvent)
    {
        _firstEvent = false;
    }
    else
    {
        _metricsFile << "," << std::endl;
    }    
    _metricsFile << "{"
        << "\"cat\": \"" << "" << "\","
        << "\"pid\": \"" << 0 << "\","
        << "\"tid\": \"" << osgEarth::Threading::getCurrentThreadId() << "\","
        << "\"ts\": \""  << osg::Timer::instance()->time_u() << "\","
        << "\"ph\": \"B\","
        << "\"name\": \""  << name << "\"";

    if (!args.empty())
    {
        _metricsFile << "," << std::endl << " \"args\": {";
        bool first = true;
        for( ConfigSet::const_iterator i = args.children().begin(); i != args.children().end(); ++i ) {
            if (first)
            {
                first = !first;
            }
            else
            {
                _metricsFile << "," << std::endl;
            }
            _metricsFile << "\"" << i->key() << "\" : \"" << i->value() << "\"";
        }
        _metricsFile << "}";
    }


        _metricsFile << "}";
}

void ChromeMetricsBackend::end(const std::string& name)
{
    OpenThreads::ScopedLock< OpenThreads::Mutex > lk(_mutex);
    if (_firstEvent)
    {
        _firstEvent = false;
    }
    else
    {
        _metricsFile << "," << std::endl;
    } 

    _metricsFile << "{"
        << "\"cat\": \"" << "" << "\","
        << "\"pid\": \"" << 0 << "\","
        << "\"tid\": \"" << osgEarth::Threading::getCurrentThreadId() << "\","
        << "\"ts\": \""  << osg::Timer::instance()->time_u() << "\","
        << "\"ph\": \"E\","
        << "\"name\": \""  << name << "\""
        << "}";
}

void ChromeMetricsBackend::counter(const std::string& name, double value)
{
    OpenThreads::ScopedLock< OpenThreads::Mutex > lk(_mutex);
    if (_firstEvent)
    {
        _firstEvent = false;
    }
    else
    {
        _metricsFile << "," << std::endl;
    } 

    _metricsFile << "{"
        << "\"cat\": \"" << "" << "\","
        << "\"pid\": \"" << 0 << "\","
        << "\"tid\": \"" << osgEarth::Threading::getCurrentThreadId() << "\","
        << "\"ts\": \""  << osg::Timer::instance()->time_u() << "\","
        << "\"ph\": \"C\","
        << "\"name\": \""  << name << "\","
        << "\"args\" : {"
        << "    \"" << name << "\": " << value
        << "}"
        << "}";
}



ScopedMetric::ScopedMetric(const std::string& name, const Config& args) :
_name(name)
{
    Metrics::begin(_name, args);
}

ScopedMetric::~ScopedMetric()
{
    Metrics::end(_name);
}

