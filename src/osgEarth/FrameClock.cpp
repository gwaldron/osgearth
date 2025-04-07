/* osgEarth
 * Copyright 2008-2014 Pelican Mapping
 * MIT License
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