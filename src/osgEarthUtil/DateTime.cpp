/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
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
#include <osgEarthUtil/DateTime>
#include <string.h>
using namespace osgEarth::Util;

DateTime::DateTime()
{
    ::time( &_time_t );
    _tm = *::gmtime(&_time_t);
}

DateTime::DateTime(const ::time_t& utc)
{
    _time_t = utc;
    _tm = *::gmtime( &utc );
}

DateTime::DateTime(const ::tm& tm)
: _tm( tm )
{
    _time_t = ::mktime( &_tm );
}

DateTime::DateTime(int year, int month, int day, double hour)
{
    _tm.tm_year = year - 1900;
    _tm.tm_mon  = month - 1;
    _tm.tm_mday = day;

    double hour_whole = ::floor(hour);
    _tm.tm_hour = (int)hour_whole;
    double frac = hour - (double)_tm.tm_hour;
    double min = frac*60.0;
    _tm.tm_min = (int)::floor(min);
    frac = min - (double)_tm.tm_min;
    _tm.tm_sec = (int)(frac*60.0);
}

DateTime::DateTime(const DateTime& rhs) :
_tm    ( rhs._tm ),
_time_t( rhs._time_t )
{
    //nop
}

int 
DateTime::year() const 
{ 
    return _tm.tm_year + 1900;
}

int
DateTime::month() const
{
    return _tm.tm_mon + 1;
}

int
DateTime::day() const
{
    return _tm.tm_mday;
}

double
DateTime::hours() const
{
    return (double)_tm.tm_hour + ((double)_tm.tm_min)/60. + ((double)_tm.tm_sec)/3600.;
}
