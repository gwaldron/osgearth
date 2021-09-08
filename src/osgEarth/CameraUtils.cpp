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
#include <osgEarth/CameraUtils>
#include <osg/Camera>

#define LC "[CameraUtils] "

using namespace osgEarth;
using namespace osgEarth::Util;

void
CameraUtils::setIsShadowCamera(osg::Camera* camera)
{
    OE_SOFT_ASSERT_AND_RETURN(camera != nullptr, void());
    osg::StateSet* ss = camera->getOrCreateStateSet();
    ss->setDefine("OE_IS_SHADOW_CAMERA");
    ss->setDefine("OE_IS_DEPTH_CAMERA");
    setIsDepthCamera(camera);
}

bool
CameraUtils::isShadowCamera(const osg::Camera* camera)
{
    if (!camera) return false;
    if (!camera->isRenderToTextureCamera()) return false;
    const osg::StateSet* ss = camera->getStateSet();
    return ss && ss->getDefinePair("OE_IS_SHADOW_CAMERA") != nullptr;
}

void
CameraUtils::setIsDepthCamera(osg::Camera* camera)
{
    OE_SOFT_ASSERT_AND_RETURN(camera != nullptr, void());
    osg::StateSet* ss = camera->getOrCreateStateSet();
    ss->setDefine("OE_IS_DEPTH_CAMERA");
}

bool
CameraUtils::isDepthCamera(const osg::Camera* camera)
{
    if (!camera) return false;
    if (!camera->isRenderToTextureCamera()) return false;
    const osg::StateSet* ss = camera->getStateSet();
    return ss && ss->getDefinePair("OE_IS_DEPTH_CAMERA") != nullptr;
}

void
CameraUtils::setIsPickCamera(osg::Camera* camera)
{
    OE_SOFT_ASSERT_AND_RETURN(camera != nullptr, void());
    osg::StateSet* ss = camera->getOrCreateStateSet();
    ss->setDefine("OE_IS_PICK_CAMERA");
}

bool
CameraUtils::isPickCamera(const osg::Camera* camera)
{
    if (!camera) return false;
    if (!camera->isRenderToTextureCamera()) return false;
    const osg::StateSet* ss = camera->getStateSet();
    return ss && ss->getDefinePair("OE_IS_PICK_CAMERA") != nullptr;
}