/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include <osgEarth/CameraUtils>
#include <osg/Camera>
#include <osgEarth/Notify>

#define LC "[CameraUtils] "

using namespace osgEarth;
using namespace osgEarth::Util;

void
CameraUtils::setIsShadowCamera(osg::Camera* camera)
{
    OE_SOFT_ASSERT_AND_RETURN(camera != nullptr, void());
    osg::StateSet* ss = camera->getOrCreateStateSet();
    ss->setDefine("OE_IS_SHADOW_CAMERA");
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