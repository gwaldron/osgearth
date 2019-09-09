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
#ifndef OSGEARTHUTIL_AUTO_SCALE_CALLBACK
#define OSGEARTHUTIL_AUTO_SCALE_CALLBACK 1

#include <osgEarthUtil/Common>
#include <osg/NodeCallback>
#include <osg/Vec3d>

namespace osgEarth { namespace Util
{
    /**
     * Cull Callback that you can install on any node that implements
     * a setScale(osg::Vec3) method (like an osg::PositionAttitudeTransform)
     * and will automatically scale the model from model coords to pixels.
     *
     * A couple tips: If your object doesn't show up, it might be getting
     * small-feature-culled. Try disabling small feature calling in your
     * camera; or, you can call setCullingActive(false) on the transform
     * to which you add this callback.
     */
    template<typename T>
    class AutoScaleCallback : public osg::NodeCallback
    {
    public:

        AutoScaleCallback(
            double baseScale = 1.0,
            double minScale = 0.0,
            double maxScale = DBL_MAX) :
                _baseScale(baseScale),
                _minScale(minScale),
                _maxScale(maxScale) { }

    public: // osg::NodeCallback
        void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            T* transform = static_cast<T*>(node);
            osgUtil::CullVisitor* cs = static_cast<osgUtil::CullVisitor*>(nv);

            osg::Camera* cam = cs->getCurrentCamera();

            // If this is an RTT camera, we need to use it's "parent"
            // to calculate the proper scale factor.
            if (cam->isRenderToTextureCamera() &&
                cam->getView() &&
                cam->getView()->getCamera() &&
                cam->getView()->getCamera() != cam)
            {
                cam = cam->getView()->getCamera();
            }

            if (cam->getViewport())
            {
                // Reset the scale so we get a proper bound
                transform->setScale(osg::Vec3d(_baseScale,_baseScale, _baseScale));
                const osg::BoundingSphere& bs = node->getBound();
                osg::Matrix l2w = osg::computeLocalToWorld(nv->getNodePath());

                // transform centroid to VIEW space:
                osg::Vec3d centerView = l2w.getTrans() * cam->getViewMatrix();

                // Set X coordinate to the radius so we can use the resulting CLIP
                // distance to calculate meters per pixel:
                centerView.x() = bs.radius();

                // transform the CLIP space:
                osg::Vec3d centerClip = centerView * cam->getProjectionMatrix();

                // caluclate meters per pixel:
                double mpp = (centerClip.x()*0.5) * cam->getViewport()->width();

                // and the resulting scale we need to auto-scale.
                double scale = bs.radius() / mpp;

                if (scale < _minScale)
                    scale = _minScale;
                else if (scale>_maxScale)
                    scale = _maxScale;

                scale *= _baseScale;

                transform->setScale(osg::Vec3d(scale, scale, scale));
            }

            if (node->getCullingActive() == false)
            {
                node->setCullingActive(true);
            }

            traverse(node, nv);
        }

    private:
        double _baseScale;
        double _minScale;
		double _maxScale;
    };

} } // namespace

#endif //OSGEARTHUTIL_AUTO_SCALE_CALLBACK
