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
#include "GraticuleLabelingEngine"
#include <osgEarth/EllipsoidIntersector>

#define LC "[GraticuleLabelingEngine] "

using namespace osgEarth;
using namespace osgEarth::Util;

#define MAX_LABELS 60

namespace
{          
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

GraticuleLabelingEngine::GraticuleLabelingEngine(const SpatialReference* srs)
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

bool GraticuleLabelingEngine::getVisible(osg::Camera* camera)
{
    CameraData& data = _cameraDataMap.get(camera);
    return data.visible;
}

void
GraticuleLabelingEngine::UpdateLabelStyles::operator()(GraticuleLabelingEngine::CameraData& data)
{
    for(GraticuleLabelingEngine::LabelNodeVector::iterator i = data.xLabels.begin();
        i != data.xLabels.end();
        ++i)
    {
        i->get()->setStyle(*_xStyle);
    }

    for(GraticuleLabelingEngine::LabelNodeVector::iterator i = data.yLabels.begin();
        i != data.yLabels.end();
        ++i)
    {
        i->get()->setStyle(*_yStyle);
    }
}

void
GraticuleLabelingEngine::setStyle(const Style& style)
{
    setStyles(style, style);
}

void
GraticuleLabelingEngine::setStyles(const Style& xStyle, const Style& yStyle)
{
    _xLabelStyle = xStyle;
    _yLabelStyle = yStyle;

    UpdateLabelStyles update(_xLabelStyle, _yLabelStyle);
    _cameraDataMap.forEach(update);
}

void
GraticuleLabelingEngine::AcceptCameraData::operator()(GraticuleLabelingEngine::CameraData& data)
{
    for (LabelNodeVector::iterator i = data.xLabels.begin(); i != data.xLabels.end(); ++i)
        i->get()->accept(_nv);

    for (LabelNodeVector::iterator i = data.yLabels.begin(); i != data.yLabels.end(); ++i)
        i->get()->accept(_nv);
}

void
GraticuleLabelingEngine::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
        if (cv)
        {
            // Find the data corresponding to this camera:
            CameraData& data = _cameraDataMap.get(cv->getCurrentCamera());
            bool visible = cullTraverse(*cv, data);
            data.visible = visible;
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
GraticuleLabelingEngine::cullTraverse(osgUtil::CullVisitor& nv, CameraData& data)
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

    // Intersect the corners of the view frustum with the ellipsoid.
    // This will yield the approximate geo-extent of the view.
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

    // Use this for clamping geopoints to the edges of the frustum:
    ClipSpace window(MVP, MVPinv);

    return updateLabels(LL_world, UL_world, LR_world, window, data);
}

bool GraticuleLabelingEngine::updateLabels(const osg::Vec3d& LL_world, osg::Vec3d& UL_world, osg::Vec3d& LR_world, ClipSpace& clipSpace, CameraData& data)
{
    return true;
}
