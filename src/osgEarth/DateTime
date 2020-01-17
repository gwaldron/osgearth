/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#ifndef OSGEARTH_DATE_TIME_H
#define OSGEARTH_DATE_TIME_H

#include <osgEarth/Common>
#include <ctime>
#include <cstring>

namespace osgEarth
{
    /** Basic timestamp (seconds from the 1970 epoch) */
    typedef ::time_t TimeStamp;

    /** Time span (in seconds) */
    typedef long TimeSpan;

    /**
     * General-purpose UTC date/time object.
     * One second resolution, GMT time zone.
     */
    class OSGEARTH_EXPORT DateTime
    {
    public:
        /** DateTime representing "now" */
        DateTime();

        /** DateTime copy */
        DateTime(const DateTime& rhs);

        /** DateTime from a tm (in the local time zone) */
        DateTime(const ::tm& tm);

        /** DateTime from UTC seconds since the epoch */
        DateTime(TimeStamp utc);

        /** DateTime from year, month [1-12], date [1-31], hours [0-24) */
        DateTime(int year, int month, int day, double hours);

        /** DateTime from year and fractional day-of-year [1..365] */
        DateTime(int year, double dayOfYear);

        /** DateTime from an ISO 8601 string */
        DateTime(const std::string& iso8601);

        /** As a date/time string in RFC 1123 format (e.g., HTTP) */
        const std::string asRFC1123() const;

        /** As a date/time string in ISO 8601 format (lexigraphic order). */
        const std::string asISO8601() const;

        /** As a date/time string in compact ISO 8601 format (lexigraphic
          * order with no delimiters). */
        const std::string asCompactISO8601() const;

        /** Julian day (fractional) corresponding to this DateTime */
        double getJulianDay() const;

        /** Seconds since Jan 1, 1970 00:00 UTC */
        TimeStamp asTimeStamp() const { return _time_t; }

        /** Adds hours to return a new DateTime */
        DateTime operator + (double hours) const;

    public:
        int    year()  const;
        int    month() const;
        int    day()   const;
        double hours() const;

    protected:
        ::tm     _tm;
        ::time_t _time_t;

    private:
        // since timegm is not cross-platform
        ::time_t timegm(const ::tm* tm) const;
    };

} // namespace osgEarth

#endif // OSGEARTH_DATE_TIME_H
