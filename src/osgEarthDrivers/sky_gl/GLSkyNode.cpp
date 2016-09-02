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

#include "GLSkyNode"
#include "GLSkyShaders"
#include <osgEarthUtil/Ephemeris>

#include <osgEarth/VirtualProgram>
#include <osgEarth/SpatialReference>
#include <osgEarth/GeoData>
#include <osgEarth/PhongLightingEffect>

#define LC "[GLSkyNode] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::GLSky;

//---------------------------------------------------------------------------

GLSkyNode::GLSkyNode(const Profile* profile) :
SkyNode()
{
    initialize(profile);
}

GLSkyNode::GLSkyNode(const Profile*      profile,
                     const GLSkyOptions& options) :
SkyNode ( options ),
_options( options )
{
    initialize(profile);
}

void
GLSkyNode::initialize(const Profile* profile)
{
    _profile = profile;
    _light = new osg::Light(0);
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
    _lighting->setCreateLightingUniform( false );
    _lighting->attach( stateset );

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
    if ( !getSunLight() || !_profile.valid() )
        return;

    const DateTime& dt = getDateTime();
    osg::Vec3d sunPosECEF = getEphemeris()->getSunPositionECEF( dt );

    if ( _profile->getSRS()->isGeographic() )
    {
        sunPosECEF.normalize();
        getSunLight()->setPosition( osg::Vec4(sunPosECEF, 0.0) );
    }
    else
    {
        // pull the ref point:
        GeoPoint refpoint = getReferencePoint();
        if ( !refpoint.isValid() )
        {
            // not found; use the center of the profile:
            _profile->getExtent().getCentroid(refpoint);
        }

        // convert to lat/long:
        GeoPoint refLatLong;
        refpoint.transform(_profile->getSRS()->getGeographicSRS(), refLatLong);

        // Matrix to convert the ECEF sun position to the local tangent plane
        // centered on our reference point:
        osg::Matrixd world2local;
        refLatLong.createWorldToLocal(world2local);

        // convert the sun position:
        osg::Vec3d sunPosLocal = sunPosECEF * world2local;
        sunPosLocal.normalize();

        getSunLight()->setPosition( osg::Vec4(sunPosLocal, 0.0) );
    }
}

void
GLSkyNode::onSetMinimumAmbient()
{
    // GLSky doesn't adjust the ambient lighting automatically, so just set it.
    _light->setAmbient( getMinimumAmbient() );
}

void
GLSkyNode::attach( osg::View* view, int lightNum )
{
    if ( !view ) return;

    _light->setLightNum( lightNum );
    view->setLight( _light.get() );
    view->setLightingMode( osg::View::SKY_LIGHT );

    onSetDateTime();
}
