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

ProgressCallback::ProgressCallback(Cancelable* cancelable, std::function<bool()> predicate) :
    _canceled(false),
    _retryDelay_s(0.0f),
    _cancelable(cancelable),
    _cancelPredicate(predicate)
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
ProgressCallback::canceled() const
{
    if (!_canceled)
    {
        if ((shouldCancel()) ||
            (_cancelable && _cancelable->canceled()) ||
            (_cancelPredicate && _cancelPredicate()))
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

