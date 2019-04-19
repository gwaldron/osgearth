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
#include "UTMLabelingEngine"

#define LC "[UTMLabelingEngine] "

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{   
    // Information for a single UTM zone. The labeling engine supports
    // two UTM zones (left and right) at a time.
    struct UTMZone
    {
        osg::ref_ptr<const SpatialReference> utmSRS;
        GeoPoint UL_geo, LL_geo, LR_geo;
        GeoPoint UL_utm, LL_utm, LR_utm;
    };    
}

//........................................................................

UTMLabelingEngine::UTMLabelingEngine(const SpatialReference* srs) :
GraticuleLabelingEngine(srs),
_maxRes(1.0)
{    
}

void
UTMLabelingEngine::setMaxResolution(double value)
{
    _maxRes = osg::maximum(value, 1.0);
    OE_INFO << LC << "Max resolution = " << _maxRes << std::endl;
}

bool
UTMLabelingEngine::updateLabels(const osg::Vec3d& LL_world, osg::Vec3d& UL_world, osg::Vec3d& LR_world, ClipSpace& window, CameraData& data)
{
    if (_maxRes > 10000.0)
        return false;

    // Split the view into (at most) 2 UTM zones. 
    UTMZone left, right;

    left.LL_geo.fromWorld(_srs.get(), LL_world);
    left.utmSRS = _srs->createUTMFromLonLat(left.LL_geo.x(), left.LL_geo.y());
    if (left.utmSRS.valid() == false)
        return false;

    left.UL_geo.fromWorld(_srs.get(), UL_world);

    right.LR_geo.fromWorld(_srs.get(), LR_world);
    right.utmSRS = _srs->createUTMFromLonLat(right.LR_geo.x(), right.LR_geo.y());
    if (right.utmSRS.valid() == false)
        return false;

    bool split = left.utmSRS->isHorizEquivalentTo(right.utmSRS.get()) == false;

    if (split)
    {
        // Calculate the longitude of the on-screen zone boundary and fill in the UTMZone values.
        double splitLon = (::floor(left.LL_geo.x() / 6.0) + 1.0) * 6.0;
        left.LR_geo.set(_srs.get(), splitLon, left.LL_geo.y(), 0, ALTMODE_ABSOLUTE);
        right.LL_geo = left.LR_geo;
        right.UL_geo.set(_srs.get(), splitLon, left.UL_geo.y(), 0, ALTMODE_ABSOLUTE);
    }
    else
    {
        left.LR_geo = right.LR_geo;
    }

    left.LR_utm = left.LR_geo.transform(left.utmSRS.get());
    left.LL_utm = left.LL_geo.transform(left.utmSRS.get());
    left.UL_utm = left.UL_geo.transform(left.utmSRS.get());

    if (left.LR_utm.isValid() == false ||
        left.LL_utm.isValid() == false ||
        left.UL_utm.isValid() == false)
    {
        OE_WARN << "Bail: left has invalid coords" << std::endl;
        return false;
    }

    if (split)
    {
        right.UL_utm = right.UL_geo.transform(right.utmSRS.get());
        right.LL_utm = right.LL_geo.transform(right.utmSRS.get());
        right.LR_utm = right.LR_geo.transform(right.utmSRS.get());

        //OE_NOTICE << "Right LL = " << right.LL_utm.toString() << std::endl;
        if (right.UL_utm.isValid() == false ||
            right.LL_utm.isValid() == false ||
            right.LR_utm.isValid() == false)
        {
            split = false;
        }
    }

    // Vertical extent of the frustum in meters:
    double utmDiff = left.LL_utm.distanceTo(left.UL_utm);

    // Determine the label interval based on the extent.
    // These numbers are from trial-and-error.
    double utmInterval;
    if (utmDiff > 150000) return false;
    else if (utmDiff > 18500) utmInterval = osg::maximum(10000.0, _maxRes);
    else if (utmDiff > 1750) utmInterval = osg::maximum(1000.0, _maxRes);
    else if (utmDiff > 170) utmInterval = osg::maximum(100.0, _maxRes);
    else utmInterval = osg::maximum(10.0, _maxRes);

    //OE_NOTICE << "utmDiff=" << utmDiff << ", utmInterval=" << utmInterval << std::endl;

    // Indices into the label pool
    unsigned xi = 0, yi = 0;

    // Finally, calculate all label positions and update them.
    // NOTE: It is safe to do this in the CULL traversal since all labels are
    // dynamic variance AND since all labels are children of this node.

    // LEFT zone:
    {
        // Quantize the start location(s) to the interval:
        double xStart = utmInterval * ::ceil(left.LL_utm.x() / utmInterval);

        unsigned numLabels = left.LL_utm.distanceTo(left.LR_utm) / utmInterval;
        if (numLabels < 2) numLabels = 2;

        // For now lets just draw 10 labels. Later we'll figure out the proper scale
        for (unsigned i = 0; i < numLabels && xi < data.xLabels.size(); ++i, ++xi)
        {
            double t = (double)i / (double)(numLabels - 1);
            double x = xStart + utmInterval * i;
            double y = left.LL_utm.y();
            GeoPoint p(left.utmSRS.get(), x, y, 0, ALTMODE_ABSOLUTE);
            int xx = ((int)x % 100000) / utmInterval;
            window.clampToBottom(p); // also xforms to geographic
            if (p.y() < 84.0 && p.y() > -80.0)
            {
                data.xLabels[xi]->setPosition(p);
                data.xLabels[xi]->setText(Stringify() << std::setprecision(8) << xx);
                data.xLabels[xi]->setNodeMask(~0);
            }
        }
        
        double yStart = utmInterval * ::ceil(left.LL_utm.y() / utmInterval);

        numLabels = left.LL_utm.distanceTo(left.UL_utm) / utmInterval;
        if (numLabels < 2) numLabels = 2;

        for (unsigned i = 0; i < numLabels && yi <  data.yLabels.size(); ++i, ++yi)
        {
            double t = (double)i / (double)(numLabels - 1);
            double x = left.LL_utm.x();
            double y = yStart + utmInterval * i;
            int yy = ((10000000 + (int)y) % 100000) / utmInterval;
            GeoPoint p(left.utmSRS.get(), x, y, 0, ALTMODE_ABSOLUTE);
            window.clampToLeft(p); // also xforms to geographic
            if (p.y() < 84.0 && p.y() > -80.0)
            {
                data.yLabels[yi]->setPosition(p);
                data.yLabels[yi]->setText(Stringify() << std::setprecision(8) << yy);
                data.yLabels[yi]->setNodeMask(~0);
            }
        }
    }

    // RIGHT zone, if we are split:
    if (split)
    {
        double xStart = utmInterval * ::ceil(right.LL_utm.x() / utmInterval);
        //double yStart = utmInterval * ::ceil(right.LL_utm.y() / utmInterval);

        unsigned numLabels = right.LL_utm.distanceTo(right.LR_utm) / utmInterval;
        if (numLabels < 2) numLabels = 2;

        for (unsigned i = 0; i < numLabels && xi <  data.xLabels.size(); ++i, ++xi)
        {
            double t = (double)i / (double)(numLabels - 1);
            double x = xStart + utmInterval * i;
            double y = right.LL_utm.y();
            GeoPoint p(right.utmSRS.get(), x, y, 0, ALTMODE_ABSOLUTE);
            int xx = ((int)x % 100000) / utmInterval;
            window.clampToBottom(p); // also xforms to geographic
            if (p.y() < 84.0 && p.y() > -80.0)
            {
                data.xLabels[xi]->setPosition(p);
                data.xLabels[xi]->setText(Stringify() << std::setprecision(8) << xx);
                data.xLabels[xi]->setNodeMask(~0);
            }
        }
    }

    return true;
}
