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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#ifndef OSGEARTH_METRICS_H
#define OSGEARTH_METRICS_H 1

#include <osgEarth/Common>
#include <osgEarth/Config>
#include <iostream>
#include <osgDB/fstream>

// forward
namespace osgViewer {
    class Viewer;
}

namespace osgEarth
{
    /**
     * Interface for a class that handles collecting metrics.
     */
    class OSGEARTH_EXPORT MetricsBackend : public osg::Referenced
    {
    public:
        /**
         * Begins an event.
         * @param name
         *        The name of the event
         * @param args
         *        The arguments to the event.
         */
        virtual void begin(const std::string& name, const Config& args =Config()) = 0;

        /**
         * Ends an event
         * @param name
         *        The name of the event.
         * @param args
         *        The arguments to the event.
         */
        virtual void end(const std::string& name, const Config& args =Config()) = 0;

        /**
         * A counter event
         * @param graph
         *        The graph to display the counters on.
         * @param name0
         *        The name of the first counter.
         * @param value0
         *        The value of the first counter.
         * @param name1
         *        The name of the second counter.
         * @param value1
         *        The value of the second counter.
         * @param name2
         *        The name of the third counter.
         * @param value2
         *        The value of the third counter.
         */
        virtual void counter(const std::string& graph,
                             const std::string& name0, double value0,
                             const std::string& name1, double value1,
                             const std::string& name2, double value2) = 0;
    };

    /**
     * A MetricsProvider that uses the chrome://tracing format as described here: http://www.gamasutra.com/view/news/176420/Indepth_Using_Chrometracing_to_view_your_inline_profiling_data.php
     * https://docs.google.com/document/d/1CvAClvFfyA5R-PhYUmn5OOQtYMH4h6I0nSsKchNAySU/edit?pli=1#
     */
    class OSGEARTH_EXPORT ChromeMetricsBackend : public MetricsBackend
    {
    public:
        ChromeMetricsBackend(const std::string& filename);
        ~ChromeMetricsBackend();

        virtual void begin(const std::string& name, const Config& args =Config());
        virtual void end(const std::string& name, const Config& args =Config());
        

        /**
         * A counter event
         * @param graph
         *        The graph to display the counters on.
         * @param name0
         *        The name of the first counter.
         * @param value0
         *        The value of the first counter.
         * @param name1
         *        The name of the second counter.
         * @param value1
         *        The value of the second counter.
         * @param name2
         *        The name of the third counter.
         * @param value2
         *        The value of the third counter.
         */
        virtual void counter(const std::string& graph,
                             const std::string& name0, double value0,
                             const std::string& name1, double value1,
                             const std::string& name2, double value2);

    protected:
        std::ofstream _metricsFile;
        OpenThreads::Mutex _mutex;    
        bool _firstEvent;
        osg::Timer_t _startTime;
    };

    class OSGEARTH_EXPORT Metrics
    {
    public:
        /**
         * Begins an event.
         * @param name
         *        The name of the event
         * @param args
         *        The arguments to the event.
         */
        static void begin(const std::string& name, const Config& args =Config());


        /**
         * Begins an event with a variable number of arguments
         * @param name
         *        The name of the event
         * @param argCount
         *        The number of argument pairs (key, value) that will 
         */
        static void begin(const std::string& name, unsigned int argCount, ...);

        /**
         * Ends an event with a variable number of arguments
         * @param name
         *        The name of the event
         * @param argCount
         *        The number of argument pairs (key, value) that will 

         */
        static void end(const std::string& name, unsigned int argCount, ...);

        /**
         * Ends an event
         * @param name
         *        The name of the event.
         * @param args
         *        The arguments to the event.
         */
        static void end(const std::string& name,  const Config& args = Config());

        /**
         * A counter event
         * @param graph
         *        The graph to display the counters on.
         * @param name0
         *        The name of the first counter.
         * @param value0
         *        The value of the first counter.         
         */
        static void counter(const std::string& graph,const std::string& name0, double value0);
           
        /**
         * A counter event
         * @param graph
         *        The graph to display the counters on.
         * @param name0
         *        The name of the first counter.
         * @param value0
         *        The value of the first counter.
         * @param name1
         *        The name of the second counter.
         * @param value1
         *        The value of the second counter.
         */
        static void counter(const std::string& graph,const std::string& name0, double value0,
                                                     const std::string& name1, double value1);

        /**
         * A counter event
         * @param graph
         *        The graph to display the counters on.
         * @param name0
         *        The name of the first counter.
         * @param value0
         *        The value of the first counter.
         * @param name1
         *        The name of the second counter.
         * @param value1
         *        The value of the second counter.
         * @param name2
         *        The name of the third counter.
         * @param value2
         *        The value of the third counter.
         */
        static void counter(const std::string& graph,const std::string& name0, double value0,
                                                     const std::string& name1, double value1,
                                                     const std::string& name2, double value2);

        /**
         * Gets the metrics backend.
         */
        static MetricsBackend* getMetricsBackend();

        /*
         * Whether metrics collection is enabled.  Equivalent to checking getMetricsBackend != NULL;
         */
        static bool enabled();

        /**
         * Sets the MetricsBackend
         */
        static void setMetricsBackend( MetricsBackend* backend );

        /**
         * Encodes varargs into a Config object for serialization by the metrics backend
         */
        static Config encodeArgs(unsigned count, ...);

        /**
         * Convenience function to run the OSG frame loop with metrics.
         */
        static int run(osgViewer::Viewer& viewer);
    };

    /**
     * Utility that lets you start and stop metrics with a single line of code
     * @example
     * void myFunction() {
     *     ScopedMetric("myFunction");
     *     ...do some work
     * }
     */
    class OSGEARTH_EXPORT ScopedMetric
    {
    public:
        ScopedMetric(const std::string& name);
        ScopedMetric(const std::string& name, const Config& args);
        ScopedMetric(const std::string& name, int argCount, ...);
        ~ScopedMetric();
        std::string _name;
    };

#define METRIC_BEGIN(...) if (osgEarth::Metrics::enabled()) osgEarth::Metrics::begin(__VA_ARGS__)

#define METRIC_END(...)   if (osgEarth::Metrics::enabled()) osgEarth::Metrics::end(__VA_ARGS__)
    
#define METRIC_SCOPED(NAME) \
    osgEarth::ScopedMetric scoped_metric__(NAME) 

#define METRIC_SCOPED_EX(NAME, COUNT, ...) \
    osgEarth::ScopedMetric scoped_metric__(NAME, osgEarth::Metrics::enabled() ? osgEarth::Metrics::encodeArgs(COUNT, __VA_ARGS__) : osgEarth::Config())
};

#endif
