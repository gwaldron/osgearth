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

#include "GLSkyNode"
#include <osg/LightSource>

#include <osgEarth/VirtualProgram>
#include <osgEarth/SpatialReference>
#include <osgEarth/GeoData>
#include <osgEarth/Lighting>
#include <osgEarth/PhongLightingEffect>
#include <osgEarthUtil/Ephemeris>

#define LC "[GLSkyNode] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::GLSky;

//---------------------------------------------------------------------------

GLSkyNode::GLSkyNode(const GLSkyOptions& options) :
SkyNode ( options ),
_options( options )
{
    construct();
}

void
GLSkyNode::construct()
{
    _light = new LightGL3(0);
    _light->setDataVariance(_light->DYNAMIC);
    _light->setAmbient(osg::Vec4(0.1f, 0.1f, 0.1f, 1.0f));
    _light->setDiffuse(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    _light->setSpecular(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));

    if ( _options.ambient().isSet() )
    {
        float a = osg::clampBetween(_options.ambient().get(), 0.0f, 1.0f);
        _light->setAmbient(osg::Vec4(a, a, a, 1.0f));
    }

    // installs the main uniforms and the shaders that will light the subgraph (terrain).
    osg::StateSet* stateset = this->getOrCreateStateSet();

    _lighting = new PhongLightingEffect();
    _lighting->attach( stateset );

    // install the Sun as a lightsource.
    osg::LightSource* lightSource = new osg::LightSource();
    lightSource->setLight(_light.get());
    lightSource->setCullingActive(false);
    this->addChild( lightSource );
    lightSource->addCullCallback(new LightSourceGL3UniformGenerator());

    onSetDateTime();
}

GLSkyNode::~GLSkyNode()
{
    if ( _lighting.valid() )
        _lighting->detach();
}

void
GLSkyNode::onSetEphemeris()
{
    // trigger the date/time update.
    onSetDateTime();
}

void
GLSkyNode::onSetReferencePoint()
{
    onSetDateTime();
}

void
GLSkyNode::onSetDateTime()
{
    if ( !getSunLight() )
        return;

    CelestialBody sun = getEphemeris()->getSunPosition(getDateTime());   

    // If the user set a projected-map reference point, assume we are using
    // a projected map and set the sun position acordingly.
    if (getReferencePoint().isValid())
    {
        // pull the ref point:
        GeoPoint refpoint = getReferencePoint();

        // convert to lat/long:
        GeoPoint refLatLong;
        osg::ref_ptr<const SpatialReference> wgs84 = SpatialReference::get("wgs84");
        refpoint.transform(wgs84.get(), refLatLong);

        // Matrix to convert the ECEF sun position to the local tangent plane
        // centered on our reference point:
        osg::Matrixd world2local;
        refLatLong.createWorldToLocal(world2local);

        // convert the sun position:
        osg::Vec3d sunPosLocal = sun.geocentric * world2local;
        sunPosLocal.normalize();

        getSunLight()->setPosition( osg::Vec4(sunPosLocal, 0.0) );
    }

    else if (_options.coordinateSystem() == SkyOptions::COORDSYS_ECEF)
    {
        osg::Vec3d pos = sun.geocentric;
        pos.normalize();
        _light->setPosition(osg::Vec4(pos, 0.0)); // directional light
    }

    else if (_options.coordinateSystem() == SkyOptions::COORDSYS_ECI)
    {
        osg::Vec3d pos = sun.eci;
        pos.normalize();
        _light->setPosition(osg::Vec4(pos, 0.0)); // directional light
    }
}

void
GLSkyNode::attach( osg::View* view, int lightNum )
{
    if ( !view ) return;

    _light->setLightNum( lightNum );
    
    // install the light in the view (so other modules can access it, like shadowing)
    view->setLight(_light.get());

    // Tell the view not to automatically include a light.
    view->setLightingMode( osg::View::NO_LIGHT );

    // initial date/time setup.
    onSetDateTime();
}
