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
