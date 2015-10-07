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