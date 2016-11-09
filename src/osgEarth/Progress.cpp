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

#include <osgEarth/Progress>
#include <osgEarth/Notify>

using namespace osgEarth;

ProgressCallback::ProgressCallback() :
osg::Referenced( true ),
_canceled      ( false ),
_failed        ( false ),
_needsRetry    ( false ),
_collectStats  ( false )
{
    //NOP
}

void ProgressCallback::reportError(const std::string& msg)
{
    _message = msg;
    _failed = true;
}

bool ProgressCallback::reportProgress(double             current,
                                      double             total,
                                      unsigned           stage,
                                      unsigned           numStages,
                                      const std::string& msg )
{
    return false;
}

double& ProgressCallback::stats(const std::string& key)
{
    Stats::iterator i = _stats.find(key);
    if ( i == _stats.end() )
    {
        double& value = _stats[key];
        value = 0.0;
        return value;
    }
    return i->second;
}

/******************************************************************************/
ConsoleProgressCallback::ConsoleProgressCallback() :
ProgressCallback()
{
    //NOP
}

void
ConsoleProgressCallback::reportError(const std::string& msg)
{
    ProgressCallback::reportError(msg);
    OE_NOTICE << "Error: " << msg << std::endl;
}

bool
ConsoleProgressCallback::reportProgress(double current, double total, 
                                        unsigned stage, unsigned numStages,
                                        const std::string& msg)
{
    if (total > 0)
    {
        double percentComplete = (current / total) * 100.0;
        OE_NOTICE 
            << "Stage " << (stage+1) << "/" << numStages 
            << "; completed " << percentComplete << "% " << current << " of " << total 
            << std::endl;
    }
    else
    {
        OE_NOTICE << msg << std::endl;
    }
    return false;
}
