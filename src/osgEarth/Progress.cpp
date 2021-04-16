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

#include <osgEarth/Progress>
#include <osgDB/DatabasePager>

using namespace osgEarth;

ProgressCallback::ProgressCallback() :
    _canceled(false),
    _retryDelay_s(0.0f),
    _cancelable(nullptr)
{
    //NOP
}

ProgressCallback::ProgressCallback(Cancelable* cancelable) :
    _canceled(false),
    _retryDelay_s(0.0f),
    _cancelable(cancelable)
{
    //NOP
}

void
ProgressCallback::cancel()
{
    _canceled = true;
}

void
ProgressCallback::reset()
{
    _canceled = false;
}

bool
ProgressCallback::isCanceled() const
{
    if (!_canceled)
    {
        if ((shouldCancel()) ||
            (_cancelable && _cancelable->isCanceled()))
        {
            _canceled = true;
        }
    }
    return _canceled;
}

void ProgressCallback::reportError(const std::string& msg)
{
    _message = msg;
}

bool ProgressCallback::reportProgress(double             current,
                                      double             total,
                                      unsigned           stage,
                                      unsigned           numStages,
                                      const std::string& msg )
{
    return false;
}

/******************************************************************************/

DatabasePagerProgressCallback::DatabasePagerProgressCallback()
{
    // if this is a pager thread, get a handle on it:

    // TODO: figure out a way to do this WITHOUT OpenThreads, since this
    // is the ONLY reason for the dependency.
    _pagerThread = dynamic_cast<osgDB::DatabasePager::DatabaseThread*>(
        OpenThreads::Thread::CurrentThread());
}

bool
DatabasePagerProgressCallback::shouldCancel() const
{
    return 
        (ProgressCallback::shouldCancel()) ||
        (_pagerThread != NULL && static_cast<osgDB::DatabasePager::DatabaseThread*>(_pagerThread)->getDone());
}
