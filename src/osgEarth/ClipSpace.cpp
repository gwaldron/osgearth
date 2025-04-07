/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgEarth/ClipSpace>

using namespace osgEarth;
using namespace osgEarth::Util;

ClipSpace::ClipSpace(const osg::Matrix& MVP, const osg::Matrix& MVPinv)
    : _worldToClip(MVP),
    _clipToWorld(MVPinv)
{
}

void ClipSpace::clampToBottom(GeoPoint& p)
{
    p.transformInPlace(p.getSRS()->getGeographicSRS());
    osg::Vec3d world;
    p.toWorld(world);
    osg::Vec3d clip = world * _worldToClip;
    clip.y() = -1.0;
    world = clip * _clipToWorld;
    p.fromWorld(p.getSRS(), world);
}

void ClipSpace::clampToLeft(GeoPoint& p)
{
    p.transformInPlace(p.getSRS()->getGeographicSRS());
    osg::Vec3d world;
    p.toWorld(world);
    osg::Vec3d clip = world * _worldToClip;
    clip.x() = -1.0;
    world = clip * _clipToWorld;
    p.fromWorld(p.getSRS(), world);
}

void ClipSpace::clampToBottom(GeoPoint& p, const GeoPoint& eye)
{
    p.transformInPlace(p.getSRS()->getGeographicSRS());
    osg::Vec3d world;
    p.toWorld(world);
    osg::Vec3d clip = world * _worldToClip;

    GeoPoint eyeGeo = eye.transform(p.getSRS()->getGeographicSRS());
    osg::Vec3d eyeWorld;
    eyeGeo.toWorld(eyeWorld);
    osg::Vec3d eyeClip = eyeWorld * _worldToClip; 

    osg::Vec3d vecClip = clip - eyeClip;
    vecClip.normalize();

    const osg::Vec3d planeNormal(0,+1,0);
    const osg::Vec3d planePoint(0,-1,0);

    double t = ((planePoint-eyeClip)*planeNormal) / (vecClip*planeNormal);
    clip = eyeClip + vecClip*t;

    //clip.x() = -1.0;
    world = clip * _clipToWorld;
    p.fromWorld(p.getSRS(), world);
}

void ClipSpace::clampToLeft(GeoPoint& p, const GeoPoint& eye)
{
    p.transformInPlace(p.getSRS()->getGeographicSRS());
    osg::Vec3d world;
    p.toWorld(world);
    osg::Vec3d clip = world * _worldToClip;

    GeoPoint eyeGeo = eye.transform(p.getSRS()->getGeographicSRS());
    osg::Vec3d eyeWorld;
    eyeGeo.toWorld(eyeWorld);
    osg::Vec3d eyeClip = eyeWorld * _worldToClip; 

    osg::Vec3d vecClip = clip - eyeClip;
    vecClip.normalize();

    const osg::Vec3d planeNormal(+1,0,0);
    const osg::Vec3d planePoint(-1,0,0);
    double t = ((planePoint-eyeClip)*planeNormal) / (vecClip*planeNormal);
    clip = eyeClip + vecClip*t;

    world = clip * _clipToWorld;
    p.fromWorld(p.getSRS(), world);
}
