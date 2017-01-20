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
#include <osgEarth/Memory>
#include <osgViewer/Viewer>
#include <cstdarg>

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
    if (s_metrics_backend.valid())
    {
        s_metrics_backend->begin(name, args);
    }
}

Config Metrics::encodeArgs(unsigned count, ...)
{
    Config conf;

    va_list args;
    va_start( args, count );    

    for (unsigned int i = 0; i < count; i++)
    {
        char* key = va_arg(args, char*);
        char* value = va_arg(args, char*);
        conf.add( key, value );
    }

    va_end(args);

    return conf;
}

void Metrics::begin(const std::string& name, unsigned int argCount, ...)
{
    if (!s_metrics_backend.valid())
        return;

    Config conf;

    va_list args;
    va_start( args, argCount );    

    for (unsigned int i = 0; i < argCount; i++)
    {
        char* key = va_arg(args, char*);
        char* value = va_arg(args, char*);
        conf.add( key, value );
    }

    va_end(args);

    begin(name, conf);
}

void Metrics::end(const std::string& name, unsigned int argCount, ...)
{
    if (!s_metrics_backend.valid())
        return;

    Config conf;

    va_list args;
    va_start( args, argCount );    

    for (unsigned int i = 0; i < argCount; i++)
    {
        char* key = va_arg(args, char*);
        char* value = va_arg(args, char*);
        conf.add( key, value );
    }

    va_end(args);

    end(name, conf);
}

void Metrics::end(const std::string& name, const Config& args)
{
    if (s_metrics_backend.valid())
    {
        s_metrics_backend->end(name, args);
    }
}

void Metrics::counter(const std::string& graph,const std::string& name0, double value0)
{
    Metrics::counter(graph, name0, value0,
                            "", 0.0,
                            "", 0.0);
}

void Metrics::counter(const std::string& graph,const std::string& name0, double value0,
                                        const std::string& name1, double value1)
{
    Metrics::counter(graph, name0, value0,
                            name1, value1,
                            "", 0.0);
}


void Metrics::counter(const std::string& graph,const std::string& name0, double value0,
                                        const std::string& name1, double value1,
                                        const std::string& name2, double value2)
{
    if (s_metrics_backend.valid())
    {
        s_metrics_backend->counter(graph, name0, value0,
                                          name1, value1,
                                          name2, value2);
    }
}

MetricsBackend* Metrics::getMetricsBackend()
{
    return s_metrics_backend.get();
}

void Metrics::setMetricsBackend(MetricsBackend* backend)
{
    s_metrics_backend = backend;
}

bool Metrics::enabled()
{
    return getMetricsBackend() != NULL;
}

int Metrics::run(osgViewer::Viewer& viewer)
{
    if (Metrics::enabled())
    {
        if (!viewer.isRealized())
        {
            viewer.realize();
        }

        double totalFrameTime = 0.0;
        // Report memory and fps every 10 frames.
        unsigned int reportEvery = 10;

        while (!viewer.done())
        {
            osg::Timer_t frameStart = osg::Timer::instance()->tick();

            METRIC_SCOPED("frame");

            viewer.advance();

            {
                METRIC_SCOPED("event");
                viewer.eventTraversal();
            }

            {
                METRIC_SCOPED("update");
                viewer.updateTraversal();
            }

            {
                METRIC_SCOPED("cull/draw");
                viewer.renderingTraversals();
            }

            // Record the frame time.
            osg::Timer_t frameEnd = osg::Timer::instance()->tick();
            double frameTime = osg::Timer::instance()->delta_s(frameStart, frameEnd);
            totalFrameTime += frameTime;

            // Report memory and fps periodically. periodically.
            if (viewer.getFrameStamp()->getFrameNumber() % reportEvery == 0)
            {
                double fps = 1.0 / (totalFrameTime / (double)reportEvery);
                totalFrameTime = 0.0;

                // Only report the metrics if they are enabled to avoid computing the memory.
                if (Metrics::enabled())
                {
                    Metrics::counter("WorkingSet", "WorkingSet", Memory::getProcessPhysicalUsage() / 1048576);
                    Metrics::counter("PrivateBytes", "PrivateBytes", Memory::getProcessPrivateUsage() / 1048576);
                    Metrics::counter("PeakPrivateBytes", "PeakPrivateBytes", Memory::getProcessPeakPrivateUsage() / 1048576);
                    Metrics::counter("FPS", "FPS", fps);
                }
            }
        }

        return 0;
    }

    else
    {
        return viewer.run();
    }
}



ChromeMetricsBackend::ChromeMetricsBackend(const std::string& filename):
_firstEvent(true)
{
    _startTime = osg::Timer::instance()->tick();
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
    osg::Timer_t now = osg::Timer::instance()->tick();

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
        << "\"ts\": \""  << std::setprecision(9) << osg::Timer::instance()->delta_u(_startTime, now) << "\","
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

void ChromeMetricsBackend::end(const std::string& name, const Config& args)
{
    osg::Timer_t now = osg::Timer::instance()->tick();

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
        << "\"ts\": \""  << std::setprecision(9) << osg::Timer::instance()->delta_u(_startTime, now) << "\","
        << "\"ph\": \"E\","
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

void ChromeMetricsBackend::counter(const std::string& graph,
                             const std::string& name0, double value0,
                             const std::string& name1, double value1,
                             const std::string& name2, double value2)
{
    osg::Timer_t now = osg::Timer::instance()->tick();

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
        << "\"ts\": \""  << std::setprecision(9) << osg::Timer::instance()->delta_u(_startTime, now) << "\","
        << "\"ph\": \"C\","
        << "\"name\": \""  << graph << "\","
        << "\"args\" : {";

        if (!name0.empty())
        {
            _metricsFile << "    \"" << name0 << "\": " << std::setprecision(9) << value0;
        }

        if (!name1.empty())
        {
            _metricsFile << ",    \"" << name1 << "\": " << std::setprecision(9) <<value1;
        }

        if (!name2.empty())
        {
            _metricsFile << ",    \"" << name2 << "\": " << std::setprecision(9) <<value2;
        }
        _metricsFile << "}}";
}



ScopedMetric::ScopedMetric(const std::string& name) :
_name(name)
{
    static Config s_emptyConfig;
    Metrics::begin(_name, s_emptyConfig);
}

ScopedMetric::ScopedMetric(const std::string& name, const Config& args) :
_name(name)
{
    Metrics::begin(_name, args);
}

ScopedMetric::ScopedMetric(const std::string& name, int argCount, ...) :
_name(name)
{
    if (!s_metrics_backend) return;

    Config conf;

    va_list args;
    va_start( args, argCount );    

    for (unsigned int i = 0; i < argCount; i++)
    {
        char* key = va_arg(args, char*);
        char* value = va_arg(args, char*);
        conf.add( key, value );
    }

    va_end(args);

    Metrics::begin(_name, conf);
}

ScopedMetric::~ScopedMetric()
{
    Metrics::end(_name);
}

