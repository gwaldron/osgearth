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
#include <osgEarthUtil/HorizonClipPlane>
#include <osgEarth/VirtualProgram>
#include <osgUtil/CullVisitor>
#include <cassert>

using namespace osgEarth;
using namespace osgEarth::Util;

HorizonClipPlane::HorizonClipPlane() :
_num(0u)
{
    //nop
}

void
HorizonClipPlane::setClipPlaneNumber(unsigned num)
{
    _num = num;
}

void
HorizonClipPlane::installShaders(osg::StateSet* stateSet)
{
    assert(stateSet != 0L);

    const char* shader =
        "#version " GLSL_VERSION_STR "\n"
        "#pragma import_defines(OE_CLIPPLANE_NUM) \n"
        "uniform mat4 osg_ViewMatrixInverse; \n"
        "uniform vec4 oe_clipping_plane; // world space \n"
        "void oe_clipplane_apply(inout vec4 vertex_view)\n"
        "{\n"
        "    gl_ClipDistance[OE_CLIPPLANE_NUM] = dot(osg_ViewMatrixInverse * vertex_view, oe_clipping_plane); \n"
        "}\n";

    VirtualProgram* vp = VirtualProgram::getOrCreate(stateSet);
    vp->setFunction("oe_clipplane_apply", shader, ShaderComp::LOCATION_VERTEX_VIEW, 0.99f);

    stateSet->setDefine("OE_CLIPPLANE_NUM", Stringify() << getClipPlaneNumber());
}

void
HorizonClipPlane::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);
    PerCameraData& d = data.get(cv->getCurrentCamera());
    
    osg::Plane horizonPlane;

    // is there a Horizon object in the visitor? If so use it
    Horizon* horizon = Horizon::get(*nv);
    if (horizon)
    {
        horizon->setEye(nv->getViewPoint());
        horizon->getPlane(horizonPlane);
    }

    // otherwise use a local one
    else
    {
        if (!d.horizon.valid())
            d.horizon = new Horizon();

        d.horizon->setEye(nv->getViewPoint());
        d.horizon->getPlane(horizonPlane);
    }
    
    if (!d.stateSet.valid())
        d.stateSet = new osg::StateSet();

    osg::Uniform* u = d.stateSet->getOrCreateUniform("oe_clipping_plane", osg::Uniform::FLOAT_VEC4);
    u->set(osg::Vec4f(horizonPlane.asVec4()));
    
    cv->pushStateSet(d.stateSet.get());
    traverse(node, nv);
    cv->popStateSet();
}
