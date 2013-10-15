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
#include <osgEarth/DateTime>
#include <osgEarth/StringUtils>
#include <math.h>
#include <iomanip>

using namespace osgEarth;

namespace
{
    // from RFC 1123, RFC 850

    const char* rfc_wkday[7] = {
        "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"
    };

    const char* rfc_weekday[7] = {
        "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"
    };

    const char* rfc_month[12] = {
        "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };
}

//------------------------------------------------------------------------

DateTime::DateTime()
{
    ::time( &_time_t );
    tm* temp = ::gmtime( &_time_t );
    if ( temp ) _tm = *temp;
    else memset( &_tm, 0, sizeof(tm) );
}

DateTime::DateTime(TimeStamp utc)
{
    _time_t = utc;
    tm* temp = ::gmtime( &_time_t );
    if ( temp ) _tm = *temp;
    else memset( &_tm, 0, sizeof(tm) );
}

DateTime::DateTime(const ::tm& in_tm)
{
    tm temptm = in_tm;
    _time_t = ::mktime( &temptm );
    tm* temp = ::gmtime( &_time_t );
    if ( temp ) _tm = *temp;
    else memset( &_tm, 0, sizeof(tm) );
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

    // now go to time_t, and back to tm, to populate the rest of the fields.
    _time_t =  ::mktime( &_tm );
    tm* temp = ::gmtime( &_time_t );
    if ( temp ) _tm = *temp;
    else memset( &_tm, 0, sizeof(tm) );
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

const std::string
DateTime::asRFC1123() const
{
    return Stringify()
        << rfc_wkday[_tm.tm_wday] << ", "
        << std::setfill('0') << std::setw(2) << _tm.tm_mday << ' '
        << rfc_month[_tm.tm_mon] << ' '
        << std::setw(4) << (1900 + _tm.tm_year) << ' '
        << std::setw(2) << _tm.tm_hour << ':'
        << std::setw(2) << _tm.tm_min << ':'
        << std::setw(2) << _tm.tm_sec << ' '
        << "GMT";
}
