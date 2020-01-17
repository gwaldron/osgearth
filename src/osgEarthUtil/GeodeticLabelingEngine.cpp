/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
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
#include "GeodeticLabelingEngine"

#define LC "[GeodeticLabelingEngine] "

using namespace osgEarth;
using namespace osgEarth::Util;

//........................................................................

GeodeticLabelingEngine::GeodeticLabelingEngine(const SpatialReference* srs) :
GraticuleLabelingEngine(srs),
_resolution(10.0 / 180.0)
{
    _formatter = new LatLongFormatter(osgEarth::Util::LatLongFormatter::FORMAT_DEGREES_MINUTES_SECONDS_TERSE, LatLongFormatter::USE_SYMBOLS | LatLongFormatter::USE_PREFIXES);
}

double GeodeticLabelingEngine::getResolution() const
{
    return _resolution;
}

void GeodeticLabelingEngine::setResolution(double resolution)
{
    _resolution = resolution;
}

std::string
GeodeticLabelingEngine::getText(const GeoPoint& location, bool lat)
{
    double value = lat ? location.y() : location.x();
    return _formatter->format(value, lat);
}

bool
GeodeticLabelingEngine::updateLabels(const osg::Vec3d& LL_world, osg::Vec3d& UL_world, osg::Vec3d& LR_world, ClipSpace& window, CameraData& data)
{    
    SpatialReference* wgs84 = SpatialReference::create("wgs84");

    GeoPoint ll, ul, lr;
    ll.fromWorld(wgs84, LL_world);
    ul.fromWorld(wgs84, UL_world);
    lr.fromWorld(wgs84, LR_world);

    double resDegrees = _resolution * 180.0;

    double minLon = osg::minimum(ll.x(), ul.x());
    double maxLon = lr.x();
    // Handle the case where the the extent crosses the dateline.
    if (maxLon < minLon)
    {
        maxLon += 360.0;
    }

    double minLat = osg::minimum(osg::minimum(ll.y(), ul.y()), lr.y());
    double maxLat = osg::maximum(osg::maximum(ll.y(), ul.y()), lr.y());

    int minLonIndex = floor(((minLon + 180.0) / resDegrees));
    int maxLonIndex = ceil(((maxLon + 180.0) / resDegrees));

    int minLatIndex = floor(((minLat + 90) / resDegrees));
    int maxLatIndex = ceil(((maxLat + 90) / resDegrees));

    // Generate horizontal labels
    unsigned int xi = 0;
    for (int i = minLonIndex; i <= maxLonIndex; i++)
    {
        GeoPoint p(wgs84, -180.0 + (double)i * resDegrees, minLat, 0, ALTMODE_ABSOLUTE);
        std::string text = getText(p, false);
        window.clampToBottom(p); // also xforms to geographic
        data.xLabels[xi]->setPosition(p);
        data.xLabels[xi]->setText(text);
        data.xLabels[xi]->setNodeMask(~0);
        xi++;
        if (xi >= data.xLabels.size()) break;
    }

    unsigned int yi = 0;
    for (int i = minLatIndex; i <= maxLatIndex; i++)
    {
        GeoPoint p(wgs84, minLon, -90.0 + (double)i * resDegrees, 0, ALTMODE_ABSOLUTE);
        std::string text = getText(p, true);
        window.clampToLeft(p); // also xforms to geographic
        data.yLabels[yi]->setPosition(p);
        data.yLabels[yi]->setText(text);
        data.yLabels[yi]->setNodeMask(~0);
        yi++;
        if (yi >= data.yLabels.size()) break;
    }

    return true;
}
