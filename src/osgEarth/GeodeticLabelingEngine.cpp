/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include "GeodeticLabelingEngine"

#define LC "[GeodeticLabelingEngine] "

using namespace osgEarth;
using namespace osgEarth::Util;
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
    return _formatter->format(Angle(value, location.getSRS()->getUnits()), lat);
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
        GeoPoint eye(wgs84, -180.0 + (double)i * resDegrees, minLat+0.1, 0, ALTMODE_ABSOLUTE);
        window.clampToBottom(p, eye); // also xforms to geographic
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
        GeoPoint eye(wgs84, minLon+0.01, -90.0 + (double)i * resDegrees, 0, ALTMODE_ABSOLUTE);
        window.clampToLeft(p, eye); // also xforms to geographic
        data.yLabels[yi]->setPosition(p);
        data.yLabels[yi]->setText(text);
        data.yLabels[yi]->setNodeMask(~0);
        yi++;
        if (yi >= data.yLabels.size()) break;
    }

    return true;
}
