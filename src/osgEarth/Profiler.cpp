/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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

#include <osgEarth/Profiler>

#include <map>
#include <osg/Timer>

using namespace osgEarth;

typedef std::map< std::string, osg::Timer_t > StartTimeMap;
static StartTimeMap S_START_TIMES;

typedef std::map< std::string, double > ElapsedTimeMap;
static ElapsedTimeMap S_ELAPSED_TIMES;

typedef std::map< std::string, unsigned int > CallMap;
static CallMap S_CALL_COUNT;

void Profiler::start(const std::string& name)
{
    S_START_TIMES[name] = osg::Timer::instance()->tick();
}

void Profiler::end(const std::string& name)
{
    osg::Timer_t end = osg::Timer::instance()->tick();
    StartTimeMap::iterator startItr = S_START_TIMES.find(name);
    if (startItr == S_START_TIMES.end())
    {
        OE_WARN << "Can't find start time " << name << std::endl;
        return;
    }

    double dt = osg::Timer::instance()->delta_s(startItr->second, end);
    ElapsedTimeMap::iterator elapsedItr = S_ELAPSED_TIMES.find(name);
    if (elapsedItr == S_ELAPSED_TIMES.end())
    {
        S_ELAPSED_TIMES[name] = 0.0;
    }
    S_ELAPSED_TIMES[name] += dt;

    // Increment the number of calls for the task
    CallMap::iterator callItr = S_CALL_COUNT.find(name);
    if (callItr == S_CALL_COUNT.end())
    {
        S_CALL_COUNT[name] = 0;
    }
    S_CALL_COUNT[name] += 1;

}

void Profiler::dump()
{
    for (ElapsedTimeMap::iterator itr = S_ELAPSED_TIMES.begin(); itr != S_ELAPSED_TIMES.end(); ++itr)
    {
        OE_NOTICE << itr->first << ": calls=" << S_CALL_COUNT[itr->first] << "  time=" << itr->second << "s" << std::endl;
    }
}