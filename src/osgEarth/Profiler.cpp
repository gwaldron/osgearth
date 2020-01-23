
/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2018 Pelican Mapping
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

#include <osgEarth/Profiler>

using namespace osgEarth;

Zone::Zone(const std::string& name) :
    _name(name),
    _isComplete(false),
    _startTime(_startTime = osg::Timer::instance()->time_m())
{
}

void Zone::complete()
{
    _isComplete = true;
    _endTime = osg::Timer::instance()->time_m();
}

double Zone::duration() const
{
    if (_isComplete)
    {
        return _endTime - _startTime;
    }
    else
    {
        return osg::Timer::instance()->time_m() - _startTime;
    }
}



ThreadZones::ThreadZones() :
    _currentZone(0)
{
}

void ThreadZones::begin(const std::string& name)
{
    Zone* prevZone = _currentZone;
    // If we have a current zone, push this zone onto it's children list.
    if (_currentZone)
    {
        _currentZone->_children.push_back(Zone(name));
        _currentZone = &_currentZone->_children.back();
    }
    else
    {
        // Otherwise stick it on the main zones list.
        _zones.push_back(Zone(name));
        _currentZone = &_zones.back();
    }
    _currentZone->_parent = prevZone;
}

void ThreadZones::end()
{
    if (!_currentZone)
    {
        OE_NOTICE << "Calling end before calling begin..." << std::endl;
        return;
    }

    _currentZone->complete();
    _currentZone = _currentZone->_parent;
}

static Profiler& getProfiler()
{
    static Profiler profiler;
    return profiler;
}

// I think we only need to lock a mutex here....
void Profiler::begin(const std::string& name)
{
    Profiler& profiler = getProfiler();
    Threading::ScopedMutexLock lock(profiler._mutex);
    ThreadZones& zones = profiler._threadZones[osgEarth::Threading::getCurrentThreadId()];
    zones.begin(name);
}

void Profiler::zoneText(const std::string& text)
{
    Profiler& profiler = getProfiler();
    Threading::ScopedMutexLock lock(profiler._mutex);
    ThreadZones& zones = profiler._threadZones[osgEarth::Threading::getCurrentThreadId()];
    if (!zones._currentZone)
    {
        OE_FATAL << "Called zone text outside of active zone" << std::endl;
        return;
    }
    zones._currentZone->_zoneText.push_back(text);
}

void Profiler::end()
{
    Profiler& profiler = getProfiler();
    Threading::ScopedMutexLock lock(profiler._mutex);
    ThreadZones& zones = profiler._threadZones[osgEarth::Threading::getCurrentThreadId()];
    zones.end();
}

void Profiler::getZones(std::map< unsigned int, ThreadZones >& zones)
{
    Profiler& profiler = getProfiler();
    Threading::ScopedMutexLock lock(profiler._mutex);
    zones = profiler._threadZones;
}