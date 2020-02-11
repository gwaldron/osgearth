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
#include <osgEarth/Sky>
#include <osgEarth/MapNode>
#include <osgEarth/GLUtils>

using namespace osgEarth;

#undef  LC
#define LC "[SkyNode] "


SkyNode::SkyNode()
{
    baseInit(SkyOptions());
}

SkyNode::SkyNode(const SkyOptions& options)
{
    baseInit(options);
}

SkyNode::~SkyNode()
{
    //nop
}

void
SkyNode::baseInit(const SkyOptions& options)
{
    _ephemeris = new Ephemeris();
    _sunVisible = true;
    _moonVisible = true;
    _starsVisible = true;
    _atmosphereVisible = true;
    //_minimumAmbient.set(0.0f, 0.0f, 0.0f, 0.0f);

    setLighting( osg::StateAttribute::ON );

    if ( options.hours().isSet() )
    {
        float hours = osg::clampBetween(options.hours().get(), 0.0f, 24.0f);
        _dateTime = DateTime(_dateTime.year(), _dateTime.month(), _dateTime.day(), (double)hours);
        // (don't call setDateTime since we are called from the CTOR)
    }

    this->getOrCreateStateSet()->setDefine("OE_NUM_LIGHTS", "1");
}

void
SkyNode::setEphemeris(Ephemeris* ephemeris)
{
    // cannot be null.
    _ephemeris = ephemeris ? ephemeris : new Ephemeris();
    onSetEphemeris();
}

const Ephemeris*
SkyNode::getEphemeris() const
{
    return _ephemeris.get();
}

void
SkyNode::setDateTime(const DateTime& dt)
{
    _dateTime = dt;
    //OE_INFO << LC << "Time = " << dt.asRFC1123() << std::endl;
    onSetDateTime();
}

void
SkyNode::setReferencePoint(const GeoPoint& value)
{
    _refpoint = value;
    onSetReferencePoint();
}

void
SkyNode::setLighting(osg::StateAttribute::OverrideValue value)
{
    _lightingValue = value;

    if (value & osg::StateAttribute::INHERIT)
        GLUtils::remove(this->getStateSet(), GL_LIGHTING);
    else
        GLUtils::setLighting(this->getOrCreateStateSet(), value);
}

void
SkyNode::setSunVisible(bool value)
{
    _sunVisible = value;
    onSetSunVisible();
}

void
SkyNode::setMoonVisible(bool value)
{
    _moonVisible = value;
    onSetMoonVisible();
}

void
SkyNode::setStarsVisible(bool value)
{
    _starsVisible = value;
    onSetStarsVisible();
}

void
SkyNode::setAtmosphereVisible(bool value)
{
    _atmosphereVisible = value;
    onSetAtmosphereVisible();
}

//------------------------------------------------------------------------

#define SKY_OPTIONS_TAG "__osgEarth::Util::SkyOptions"

SkyNode*
SkyNode::create(const SkyOptions& options)
{
    std::string driverName = osgEarth::trim(options.getDriver());
    if ( driverName.empty() )
        driverName = "simple";

    std::string extensionName = std::string("sky_") + driverName;

    osg::ref_ptr<Extension> extension = Extension::create(extensionName, options);
    if ( !extension.valid() ) {
        OE_WARN << LC << "Failed to load extension for sky driver \"" << driverName << "\"\n";
        return 0L;
    }

    SkyNodeFactory* factory = extension->as<SkyNodeFactory>();
    if ( !factory ) {
        OE_WARN << LC << "Internal error; extension \"" << extensionName << "\" does not implement SkyNodeFactory\n";
        return 0L;
    }

    osg::ref_ptr<SkyNode> result = factory->createSkyNode();
    return result.release();
}

SkyNode*
SkyNode::create()
{
    SkyOptions options;
    return create(options);
}

SkyNode*
SkyNode::create(const std::string& driver)
{
    SkyOptions options;
    options.setDriver( driver );
    return create(options);
}


//------------------------------------------------------------------------

const SkyOptions&
SkyDriver::getSkyOptions(const osgDB::Options* options) const
{
    static SkyOptions s_default;
    const void* data = options->getPluginData(SKY_OPTIONS_TAG);
    return data ? *static_cast<const SkyOptions*>(data) : s_default;
}
