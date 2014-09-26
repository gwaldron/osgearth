/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include <osgEarthUtil/Sky>
#include <osgEarthUtil/Ephemeris>
#include <osgEarth/Registry>
#include <osgEarth/ShaderFactory>
#include <osgEarth/ShaderUtils>
#include <osgDB/ReadFile>

using namespace osgEarth;
using namespace osgEarth::Util;

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

    setLighting( osg::StateAttribute::ON );

    if ( options.hours().isSet() )
    {
        float hours = osg::clampBetween(options.hours().get(), 0.0f, 24.0f);
        _dateTime = DateTime(_dateTime.year(), _dateTime.month(), _dateTime.day(), (double)hours);
        // (don't call setDateTime since we are called from the CTOR)
    }
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
    _lightingUniform = Registry::shaderFactory()->createUniformForGLMode(
        GL_LIGHTING, value );

    this->getOrCreateStateSet()->addUniform( _lightingUniform.get() );
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
SkyNode::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        // update the light model uniforms.
        if ( _lightingUniformsHelper.valid() )
        {
            _lightingUniformsHelper->cullTraverse( this, &nv );
        }
    }
    osg::Group::traverse(nv);
}

//------------------------------------------------------------------------

#define MAPNODE_TAG     "__osgEarth::MapNode"
#define SKY_OPTIONS_TAG "__osgEarth::Util::SkyOptions"

SkyNode*
SkyNode::create(const SkyOptions& options,
                MapNode*          mapNode)
{
    SkyNode* result = 0L;

    std::string driverName = options.getDriver();
    if ( driverName.empty() )
        driverName = "simple";

    std::string driverExt = std::string(".osgearth_sky_") + driverName;

    osg::ref_ptr<osgDB::Options> rwopts = Registry::instance()->cloneOrCreateOptions();
    rwopts->setPluginData( MAPNODE_TAG, (void*)mapNode );
    rwopts->setPluginData( SKY_OPTIONS_TAG, (void*)&options );

    result = dynamic_cast<SkyNode*>( osgDB::readNodeFile( driverExt, rwopts.get() ) );
    if ( result )
    {
        OE_INFO << LC << "Loaded sky driver \"" << driverName << "\" OK." << std::endl;
    }
    else
    {
        OE_WARN << LC << "FAIL, unable to load sky driver for \"" << driverName << "\"" << std::endl;
    }

    return result;
}

SkyNode*
SkyNode::create(MapNode* mapNode)
{
    SkyOptions options;
    return create(options, mapNode);
}

SkyNode*
SkyNode::create(const std::string& driver, MapNode* mapNode)
{
    SkyOptions options;
    options.setDriver( driver );
    return create( options, mapNode );
}


//------------------------------------------------------------------------

const SkyOptions&
SkyDriver::getSkyOptions(const osgDB::Options* options) const
{
    return *static_cast<const SkyOptions*>( options->getPluginData(SKY_OPTIONS_TAG) );
}


MapNode*
SkyDriver::getMapNode(const osgDB::Options* options) const
{
    return const_cast<MapNode*>(
        static_cast<const MapNode*>( options->getPluginData(MAPNODE_TAG) ) );
}
