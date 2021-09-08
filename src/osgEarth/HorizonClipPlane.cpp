/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
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
#include <osgEarth/HorizonClipPlane>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Shaders>
#include <osgEarth/Utils>
#include <osgUtil/CullVisitor>

#undef LC
#define LC "[HorizonClipPlane] "

using namespace osgEarth;

namespace
{
    constexpr const char* clip_shader = R"(
        #version 330
        #pragma import_defines(OE_CLIPPLANE_NUM)

        // OSG built-in to transform from view to world
        uniform mat4 osg_ViewMatrixInverse;

        // clipping plane
        uniform vec4 oe_ClipPlane_plane;

        void oe_ClipPlane_vs(inout vec4 vertex_view)
        {
        #ifndef GL_ES
            gl_ClipDistance[OE_CLIPPLANE_NUM] = dot(osg_ViewMatrixInverse * vertex_view, oe_ClipPlane_plane);
        #endif
        }
    )";
}

HorizonClipPlane::HorizonClipPlane() :
    _num(0u)
{
    //nop
}

HorizonClipPlane::HorizonClipPlane(const Ellipsoid& em) :
    _ellipsoid(em),
    _num(0u),
    _data(OE_MUTEX_NAME)
{
    //nop
}

void
HorizonClipPlane::setClipPlaneNumber(unsigned num)
{
    _num = num;
}

void
HorizonClipPlane::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);
    PerCameraData& d = _data.get(cv->getCurrentCamera());

    if (!d.horizon.valid())
    {
        d.horizon = new Horizon(_ellipsoid);
        d.stateSet = new osg::StateSet();
        d.uniform = new osg::Uniform("oe_ClipPlane_plane", osg::Vec4f());
        d.stateSet->addUniform(d.uniform.get());
        d.stateSet->setDefine("OE_CLIPPLANE_NUM", Stringify() << getClipPlaneNumber());

        VirtualProgram* vp = VirtualProgram::getOrCreate(d.stateSet.get());
        vp->setName("HorizonClipPlane");
        vp->setFunction("oe_ClipPlane_vs", clip_shader, ShaderComp::LOCATION_VERTEX_VIEW, FLT_MAX);
    }

    // push this horizon on to the nodevisitor so modules can access it
    //d.horizon->put(*nv);
    ObjectStorage::set(nv, d.horizon.get());

    // update with current eyepoint
    if (d.horizon->setEye(nv->getViewPoint()))
    {
        // compute the horizon plane and update the clipping uniform
        osg::Plane horizonPlane;
        if (d.horizon->getPlane(horizonPlane))
        {
            d.uniform->set(osg::Vec4f(horizonPlane.asVec4()));
        }
    }
    
    cv->pushStateSet(d.stateSet.get());
    traverse(node, nv);
    cv->popStateSet();
}

void
HorizonClipPlane::ResizeFunctor::operator()(HorizonClipPlane::PerCameraData& data)
{
    if (data.horizon.valid())
        data.horizon->resizeGLObjectBuffers(_s);
    if (data.stateSet.valid())
        data.stateSet->resizeGLObjectBuffers(_s);
    if (data.uniform.valid())
        data.uniform->resizeGLObjectBuffers(_s);
}

void
HorizonClipPlane::resizeGLObjectBuffers(unsigned maxSize)
{
    ResizeFunctor f(maxSize);
    _data.forEach(f);
}

void
HorizonClipPlane::ReleaseFunctor::operator()(const HorizonClipPlane::PerCameraData& data) const
{
    if (data.horizon.valid())
        data.horizon->releaseGLObjects(_state);
    if (data.stateSet.valid())
        data.stateSet->releaseGLObjects(_state);
    if (data.uniform.valid())
        data.uniform->releaseGLObjects(_state);
}

void
HorizonClipPlane::releaseGLObjects(osg::State* state) const
{
    ReleaseFunctor f(state);
    _data.forEach(f);
}
