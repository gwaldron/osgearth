/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
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
#include <osgEarth/GeoData>
#include <osgEarth/TerrainEngineNode>
#include <osg/CoordinateSystemNode>

#define LC "[UTMLabelingEngine] "

using namespace osgEarth;
using namespace osgEarth::Util;

#define MAX_LABELS 60

namespace
{
    /**
     * Utility class to perform ellipsoid intersections
     * (Used by the UTMLabelingEngine)
     * TODO: At some point this class can graduate to the core if generally useful,
     * possibly extending osg::EllipsoidModel.
     */
    class EllipsoidIntersector
    {
    public:
        //! Construct a new ellipsoid intersector
        EllipsoidIntersector(const osg::EllipsoidModel* em)
        {
            _ellipsoidToUnitSphere.makeScale(
                1.0 / em->getRadiusEquator(),
                1.0 / em->getRadiusEquator(),
                1.0 / em->getRadiusPolar());

            _unitSphereToEllipsoid.makeScale(
                em->getRadiusEquator(),
                em->getRadiusEquator(),
                em->getRadiusPolar());
        }

        //! Interects a line (world space) with an ellipsoid.
        //! @param p0 First point on the line
        //! @param p1 Second point on the line
        //! @param out_world Output world coordinates of closest intersection
        bool intersectLine(const osg::Vec3d& p0_world, const osg::Vec3d& p1_world, osg::Vec3d& out_world)
        {
            double dist2 = 0.0;
            osg::Vec3d v;
            osg::Vec3d p0 = p0_world * _ellipsoidToUnitSphere;
            osg::Vec3d p1 = p1_world * _ellipsoidToUnitSphere;

            const double R = 1.0; // for unit sphere.

            // http://paulbourke.net/geometry/circlesphere/index.html#linesphere

            osg::Vec3d d = p1 - p0;

            double A = d * d;
            double B = 2.0 * (d * p0);
            double C = (p0 * p0) - R*R;

            // now solve the quadratic A + B*t + C*t^2 = 0.
            double D = B*B - 4.0*A*C;
            if (D > 0)
            {
                // two roots (line passes through sphere twice)
                // find the closer of the two.
                double sqrtD = sqrt(D);
                double t0 = (-B + sqrtD) / (2.0*A);
                double t1 = (-B - sqrtD) / (2.0*A);

                //seg; pick closest:
                if (fabs(t0) < fabs(t1))
                    v = d*t0;
                else
                    v = d*t1;
            }
            else if (D == 0.0)
            {
                // one root (line is tangent to sphere?)
                double t = -B / (2.0*A);
                v = d*t;
            }

            dist2 = v.length2();

            if (dist2 > 0.0)
            {
                out_world = (p0 + v) * _unitSphereToEllipsoid;
                return true;
            }
            else
            {
                // either no intersection, or the distance was not the max.
                return false;
            }
        }

    private:
        osg::Matrix _ellipsoidToUnitSphere;
        osg::Matrix _unitSphereToEllipsoid;
        osg::Matrix _clipToWorld;
    };


    /**
     * Utility class for perform operations in clip space
     * (Used by the UTMLabelingEngine)
     */
    class ClipSpace
    {
    public:
        osg::Matrix _worldToClip, _clipToWorld;

        ClipSpace(const osg::Matrix& MVP, const osg::Matrix& MVPinv)
            : _worldToClip(MVP),
            _clipToWorld(MVPinv)
        {
            //nop
        }

        // Moves the input point to the bottom edge of the viewport.
        void clampToBottom(GeoPoint& p)
        {
            p.transformInPlace(p.getSRS()->getGeographicSRS());
            osg::Vec3d world;
            p.toWorld(world);
            osg::Vec3d clip = world * _worldToClip;
            clip.y() = -1.0;
            world = clip * _clipToWorld;
            p.fromWorld(p.getSRS(), world);
        }

        // Moves the input point to left edge of the viewport.
        void clampToLeft(GeoPoint& p)
        {
            p.transformInPlace(p.getSRS()->getGeographicSRS());
            osg::Vec3d world;
            p.toWorld(world);
            osg::Vec3d clip = world * _worldToClip;
            clip.x() = -1.0;
            world = clip * _clipToWorld;
            p.fromWorld(p.getSRS(), world);
        }
    };


    // Information for a single UTM zone. The labeling engine supports
    // two UTM zones (left and right) at a time.
    struct UTMZone
    {
        osg::ref_ptr<const SpatialReference> utmSRS;
        GeoPoint UL_geo, LL_geo, LR_geo;
        GeoPoint UL_utm, LL_utm, LR_utm;
    };


    // Given a view matrix, return the heading of the camera relative to North;
    // this works for geocentric maps.
    // TODO: graduate to a utilities class somewhere in the core if generally useful
    double getCameraHeading(const osg::Matrix& VM)
    {
        osg::Matrixd VMinverse;
        VMinverse.invert(VM);

        osg::Vec3d N(0, 0, 6356752); // north pole, more or less
        osg::Vec3d b(-VM(0, 2), -VM(1, 2), -VM(2, 2)); // look vector
        osg::Vec3d E = osg::Vec3d(0, 0, 0)*VMinverse;
        osg::Vec3d u = E; u.normalize();

        // account for looking straight downish
        if (osg::equivalent(b*u, -1.0, 1e-4))
        {
            // up vec becomes the look vec.
            b = osg::Matrixd::transform3x3(VM, osg::Vec3f(0.0, 1.0, 0.0));
            b.normalize();
        }

        osg::Vec3d proj_d = b - u*(b*u);
        osg::Vec3d n = N - E;
        osg::Vec3d proj_n = n - u*(n*u);
        osg::Vec3d proj_e = proj_n^u;

        double cameraHeading = atan2(proj_e*proj_d, proj_n*proj_d);
        return cameraHeading;
    }
}

//........................................................................

UTMLabelingEngine::UTMLabelingEngine(const SpatialReference* srs) :
_maxRes(1.0)
{
    _srs = srs;

    // Set up the symbology for x-axis labels
    TextSymbol* xText = _xLabelStyle.getOrCreate<TextSymbol>();
    xText->alignment() = TextSymbol::ALIGN_CENTER_BOTTOM;
    xText->halo()->color().set(0, 0, 0, 1);
    xText->declutter() = false;

    // Set up the symbology for y-axis labels
    TextSymbol* yText = _yLabelStyle.getOrCreate<TextSymbol>();
    yText->alignment() = TextSymbol::ALIGN_LEFT_BOTTOM;
    yText->halo()->color().set(0, 0, 0, 1);
    yText->declutter() = false;
}

void
UTMLabelingEngine::setMaxResolution(double value)
{
    _maxRes = std::max(value, 1.0);
    OE_INFO << LC << "Max resolution = " << _maxRes << std::endl;
}

void
UTMLabelingEngine::AcceptCameraData::operator()(UTMLabelingEngine::CameraData& data)
{
    for (LabelNodeVector::iterator i = data.xLabels.begin(); i != data.xLabels.end(); ++i)
        i->get()->accept(_nv);

    for (LabelNodeVector::iterator i = data.yLabels.begin(); i != data.yLabels.end(); ++i)
        i->get()->accept(_nv);
}

void
UTMLabelingEngine::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
        if (cv)
        {
            // Find the data corresponding to this camera:
            CameraData& data = _cameraDataMap.get(cv->getCurrentCamera());
            bool visible = cullTraverse(*cv, data);
            if (visible)
            {
                // traverse all the labels for this camera:
                AcceptCameraData accept(nv);
                accept(data);
            }
        }
    }
    else
    {
        AcceptCameraData accept(nv);
        _cameraDataMap.forEach(accept);
    }
    
    osg::Group::traverse(nv);
}

bool
UTMLabelingEngine::cullTraverse(osgUtil::CullVisitor& nv, CameraData& data)
{
    osg::Camera* cam = nv.getCurrentCamera();

    // Don't draw the labels if we are too far from North-Up:
    double heading = getCameraHeading(cam->getViewMatrix());
    if (osg::RadiansToDegrees(fabs(heading)) > 7.0)
        return false;

    // Initialize the label pool for this camera if we have not done so:
    if (data.xLabels.empty())
    {
        for (unsigned i = 0; i < MAX_LABELS; ++i)
        {
            LabelNode* label = new LabelNode();
            label->setDynamic(true);
            label->setStyle(_xLabelStyle);
            label->setHorizonCulling(false);
            label->setOcclusionCulling(false);
            data.xLabels.push_back(label);
        }

        for (unsigned i = 0; i < MAX_LABELS; ++i)
        {
            LabelNode* label = new LabelNode();
            label->setDynamic(true);
            label->setStyle(_yLabelStyle);
            label->setHorizonCulling(false);
            label->setOcclusionCulling(false);
            data.yLabels.push_back(label);
        }
    }

    // Start out with all labels off. We will then turn back on the ones we use:
    for (unsigned i = 0; i < MAX_LABELS; ++i)
    {
        data.xLabels[i]->setNodeMask(0);
        data.yLabels[i]->setNodeMask(0);
    }

    if (_maxRes > 10000.0)
        return false;

    // Intersect the corners of the view frustum with the ellipsoid.
    // This will yeild the approximate geo-extent of the view.
    // TODO: graduate this to the core if generally useful - could be helpful
    // for displaying the extent of the current view.

    // Calculate the "clip to world" matrix = MVPinv.
    osg::Matrix MVP = (*nv.getModelViewMatrix()) * cam->getProjectionMatrix();
    osg::Matrix MVPinv;
    MVPinv.invert(MVP);

    EllipsoidIntersector ellipsoid(_srs->getEllipsoid());

    // For each corner, transform the clip coordinates at the near and far
    // planes into world space and intersect that line with the ellipsoid:
    osg::Vec3d p0, p1;

    // find the lower-left corner of the frustum:
    osg::Vec3d LL_world;
    p0 = osg::Vec3d(-1, -1, -1) * MVPinv;
    p1 = osg::Vec3d(-1, -1, +1) * MVPinv;
    bool LL_ok = ellipsoid.intersectLine(p0, p1, LL_world);   
    if (!LL_ok)
        return false;

    // find the upper-left corner of the frustum:
    osg::Vec3d UL_world;
    p0 = osg::Vec3d(-1, +1, -1) * MVPinv;
    p1 = osg::Vec3d(-1, +1, +1) * MVPinv;
    bool UL_ok = ellipsoid.intersectLine(p0, p1, UL_world);
    if (!UL_ok)
        return false;

    // find the lower-right corner of the frustum:
    osg::Vec3d LR_world;
    p0 = osg::Vec3d(+1, -1, -1) * MVPinv;
    p1 = osg::Vec3d(+1, -1, +1) * MVPinv;
    bool LR_ok = ellipsoid.intersectLine(p0, p1, LR_world);
    if (!LR_ok)
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
    else if (utmDiff > 18500) utmInterval = std::max(10000.0, _maxRes);
    else if (utmDiff > 1750) utmInterval = std::max(1000.0, _maxRes);
    else if (utmDiff > 170) utmInterval = std::max(100.0, _maxRes);
    else utmInterval = std::max(10.0, _maxRes);

    //OE_NOTICE << "utmDiff=" << utmDiff << ", utmInterval=" << utmInterval << std::endl;
    
    // Use this for clamping geopoints to the edges of the frustum:
    ClipSpace window(MVP, MVPinv); 

    // Indicies into the label pool
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
        for (unsigned i = 0; i < numLabels && xi < MAX_LABELS; ++i, ++xi)
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

        for (unsigned i = 0; i < numLabels && yi < MAX_LABELS; ++i, ++yi)
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

        for (unsigned i = 0; i < numLabels && xi < MAX_LABELS; ++i, ++xi)
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
