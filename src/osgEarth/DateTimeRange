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
#ifndef OSGEARTH_DATE_TIME_RANGE_H
#define OSGEARTH_DATE_TIME_RANGE_H

#include <osgEarth/DateTime>
#include <osgEarth/optional>

namespace osgEarth
{
    /**
     * Two DateTime objects representing a temporal range.
     */
    class OSGEARTH_EXPORT DateTimeRange
    {
    public:
        //! Start of date/time range
        optional<DateTime>& begin() { return _begin; }
        //! Start of date/time range
        const optional<DateTime>& begin() const { return _begin; }

        //! End of date/time range
        optional<DateTime>& end() { return _end; }
        //! End of date/time range
        const optional<DateTime>& end() const { return _end; }

        //! Expand the range to include a date/time
        void expandBy(const DateTime& other);

        //! Expand the range to include another date/time range
        void expandBy(const DateTimeRange& other);

        //! True is the given date/time falls within this range
        bool intersects(const DateTime& other) const;

        //! True is the given range falls within this range
        bool intersects(const DateTimeRange& other) const;

    private:
        optional<DateTime> _begin;
        optional<DateTime> _end;
    };

} // namespace osgEarth

#endif // OSGEARTH_DATE_TIME_RANGE_H
