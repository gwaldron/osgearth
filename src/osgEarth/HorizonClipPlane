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

#ifndef OSGEARTH_HORIZON_CLIP_PLANE_H
#define OSGEARTH_HORIZON_CLIP_PLANE_H 1

#include <osgEarth/Common>
#include <osgEarth/Horizon>
#include <osgEarth/Containers>
#include <osg/NodeCallback>
#include <osg/CoordinateSystemNode>
#include <osg/Plane>

namespace osgEarth
{
    /**
     * Installs an OpenGL clip plane that will clip graphics
     * at the geocentric horizon. Usage:
     *
     *   HorizonClipPlane* hcp = new HorizonClipPlane();
     *   hcp->setClipPlaneNumber(0);
     *   hcp->installShaders(mapNode);
     *   mapNode->addCullCallback(hcp);
     *
     * Then, anywhere under that node that you want to apply clipping:
     *
     *   node->getOrCreateStateSet()->setMode(GL_CLIP_DISTANCE0, 1);
     */
    class OSGEARTH_EXPORT HorizonClipPlane : public osg::NodeCallback
    {
    public:
        //! Construct a new horizon clip plane with default (WGS84) ellipsoid
        HorizonClipPlane();

        //! Construct a new horizon clip plane with specific ellipsoid
        HorizonClipPlane(const osg::EllipsoidModel* em);

        //! Set the GL clip plane to use. Default is zero.
        void setClipPlaneNumber(unsigned n);
        unsigned getClipPlaneNumber() const { return _num; }

    public: // osg::NodeCallback

        void operator()(osg::Node* node, osg::NodeVisitor* nv);

    public: // osg::Object

        virtual void resizeGLObjectBuffers(unsigned maxSize);
        virtual void releaseGLObjects(osg::State* state) const;

    private:

        unsigned _num;
        osg::EllipsoidModel _ellipsoid;

        struct PerCameraData {
            osg::ref_ptr<Horizon> horizon;
            osg::ref_ptr<osg::StateSet> stateSet;
            osg::ref_ptr<osg::Uniform> uniform;
        };
        typedef PerObjectFastMap<osg::Camera*, PerCameraData> DataMap;
        DataMap data;

        struct ResizeFunctor : public DataMap::Functor {
            unsigned _s;
            ResizeFunctor(unsigned s) : _s(s) { }
            virtual void operator()(PerCameraData& data);
        };
        struct ReleaseFunctor : public DataMap::ConstFunctor {
            osg::State* _state;
            ReleaseFunctor(osg::State* state) : _state(state) { }
            virtual void operator()(const PerCameraData& data) const;
        };
    };

} // namespace osgEarth

#endif // OSGEARTH_HORIZON_CLIP_PLANE_H
