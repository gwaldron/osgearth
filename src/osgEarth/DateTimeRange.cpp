/* osgEarth
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/
#include <osgEarth/DateTimeRange>

using namespace osgEarth;

void DateTimeRange::expandBy(const DateTime& other)
{
    if (!_begin.isSet() || _begin->asTimeStamp() > other.asTimeStamp())
    {
        _begin = other;
    }

    if (!_end.isSet() || _end->asTimeStamp() < other.asTimeStamp())
    {
        _end = other;
    }
}

void DateTimeRange::expandBy(const DateTimeRange& other)
{
    if (other.begin().isSet())
    {
        expandBy(other.begin().get());
    }

    if (other.end().isSet())
    {
        expandBy(other.end().get());
    }
}

bool DateTimeRange::intersects(const DateTime& other) const
{
    return ((!_begin.isSet() || _begin->asTimeStamp() <= other.asTimeStamp()) &&
            (!_end.isSet() || _end->asTimeStamp() >= other.asTimeStamp()));
}

bool DateTimeRange::intersects(const DateTimeRange& other) const
{
    if (!_begin.isSet() && !_end.isSet()) //infinite range
    {
        return true;
    }

    if (!other.begin().isSet() && !other.end().isSet()) //other is infinite range
    {
        return true;
    }

    if (other.begin().isSet() && intersects(other.begin().get()))
    {
        return true;
    }

    if (other.end().isSet() && intersects(other.end().get()))
    {
        return true;
    }

    if (_begin.isSet() && other.intersects(_begin.get()))
    {
        return true;
    }

    if (_end.isSet() && other.intersects(_end.get()))
    {
        return true;
    }

    return false;
}