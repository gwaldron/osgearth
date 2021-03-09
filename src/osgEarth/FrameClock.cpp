/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include <osgEarth/FrameClock>

using namespace osgEarth;

FrameClock::FrameClock() :
    _frame(0u),
    _newframe(false)
{
    _zero = std::chrono::steady_clock::now();
    _tick = _zero;
}

double
FrameClock::getTime() const
{
    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(_tick - _zero);
    return 0.001 * (double)(diff.count());
}

unsigned
FrameClock::getFrame() const
{
    return _frame;
}

void
FrameClock::cull()
{
    _newframe.exchange(true);
}

bool
FrameClock::update()
{
    // this block will only execute if cull() was called since
    // the previous call to update, guaranteeing that the frame
    // number will only increment once per cull/update pair.
    if (_newframe.exchange(false) == true)
    {
        _tick = std::chrono::steady_clock::now();
        ++_frame;
        return true;
    }
    return false;
}