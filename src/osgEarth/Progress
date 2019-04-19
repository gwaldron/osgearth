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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#ifndef OSGEARTH_PROGRESS_H
#define OSGEARTH_PROGRESS_H 1

#include <osgEarth/Common>
#include <osgEarth/Containers>

namespace osgEarth
{
    /**
    * ProgressCallback is a general purpose interface for functions that need to report progress
    * or handle cancelation of a task.
    */
    class OSGEARTH_EXPORT ProgressCallback : public osg::Referenced
    {
    public:
        /**
        * Creates a new ProgressCallback
        */
        ProgressCallback();
        virtual ~ProgressCallback() { }

        /**
         * Report an error and set the canceled flag to true.
         */
        virtual void reportError(const std::string& msg);

        /**
        * Callback function that will be called.
        * @param current
        *        The amount of work done in the current stage
        * @param total
        *        The total amount of work to be done in the current stage
        * @param stage
        *        Stage of the operation we're currently in
        * @param totalStages
        *        Total number of stages in the operation
        * @param msg
        *        Description of what is being done. Useful when total is unknown.
        * @param returns
        *        Returns true if the current task should be cancelled, false otherwise.
        */
        virtual bool reportProgress(
            double             current, 
            double             total, 
            unsigned           currentStage,
            unsigned           totalStages,
            const std::string& msg );

        /**
         * Convenience functions
         */
        bool reportProgress( double current, double total, const std::string& msg ) {
            return reportProgress(current, total, 0, 1, msg );
        }
        bool reportProgress( double current, double total ) {
            return reportProgress(current, total, 0, 1, "" );
        }

        /**
         * called when the process starts
         */
        virtual void onStarted() { }

        /**
         * called when the process completed 
         */
        virtual void onCompleted() { }

        /**
         * Sets the cancelation flag
         */
        virtual void cancel() { _canceled = true; }

        /**
         * Whether the associated task was canceled for some reason.
         * Cancelation is NOT an error condition. It means that either
         * the results of the task are no longer required, or that a
         * recoverable problem occurred and the task is eligible to be
         * tried again later (e.g., HTTP timeout)
         */
        virtual bool isCanceled() { return _canceled; }

        /**
         * Status/error message
         */
        std::string& message() { return _message; }
        const std::string& message() const { return _message; }

        /**
         * Resets the canceled flag.
         */
        void reset() { _canceled = false; }

        /**
         * Access user stats
         */
        typedef fast_map<std::string,double> Stats;
        Stats& stats() { return _stats; }
        double& stats(const std::string& name);
        bool& collectStats() { return _collectStats; } 
        const bool& collectStats() const { return _collectStats; }        

    protected:
        std::string       _message;
        mutable  bool     _canceled;
        mutable  Stats    _stats;
        mutable  bool     _collectStats;
    };


    /**
    * ConsoleProgressCallback is a simple ProgressCallback that reports progress to the console
    */
    class OSGEARTH_EXPORT ConsoleProgressCallback : public ProgressCallback
    {
    public:
        /**
        * Creates a new ConsoleProgressCallback
        */
        ConsoleProgressCallback();
        virtual ~ConsoleProgressCallback() { }

        virtual void reportError(const std::string& msg);

        /**
        * Callback function that will be called.
        * @param current
        *        The amount of work done in the current stage
        * @param total
        *        The total amount of work to be done in the current stage
        * @param stage
        *        Stage of the operation we're currently in
        * @param totalStages
        *        Total number of stages in the operation
        * @param msg
        *        Description of what is being done. Useful when total is unknown.
        * @param returns
        *        Returns true if the current task should be cancelled, false otherwise.
        */
        virtual bool reportProgress(
            double             current, 
            double             total, 
            unsigned           currentStage,
            unsigned           totalStages,
            const std::string& msg );
    };
}

#endif
