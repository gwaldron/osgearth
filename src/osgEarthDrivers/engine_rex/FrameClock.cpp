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
#include "FrameClock"

using namespace osgEarth::REX;

FrameClock::FrameClock() :
    _updateFrame(0u),
    _cullFrame(0u)
{
    _timer = osg::Timer::instance();
    _zero = _timer->tick();
    _tick = _zero;
}

double
FrameClock::getTime() const
{
    return _timer->delta_s(_zero, _tick);
}

unsigned
FrameClock::getFrame() const
{
    return _updateFrame;
}

bool
FrameClock::update()
{
    if (_updateFrame == _cullFrame)
    {
        _tick = _timer->tick();
        ++_updateFrame;
        return true;
    }
    return false;
}

void
FrameClock::cull()
{
    _cullFrame = _updateFrame;
}
