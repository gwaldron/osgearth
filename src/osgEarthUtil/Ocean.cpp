/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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
#include <osgEarthUtil/Ocean>
#include <osgEarth/Registry>
#include <osgDB/ReadFile>

using namespace osgEarth;
using namespace osgEarth::Util;

#undef  LC
#define LC "[Ocean] "

//------------------------------------------------------------------------

OceanOptions::OceanOptions(const ConfigOptions& options) :
DriverConfigOptions( options ),
_maxAltitude( 250000.0 )
{
    fromConfig(_conf);
}

void
OceanOptions::fromConfig( const Config& conf )
{
    conf.getIfSet( "max_altitude", _maxAltitude );
}

void
OceanOptions::mergeConfig( const Config& conf )
{
    DriverConfigOptions::mergeConfig( conf );
    fromConfig( conf );
}

Config
OceanOptions::getConfig() const
{
    Config conf = DriverConfigOptions::getConfig();
    conf.addIfSet( "max_altitude", _maxAltitude );
    return conf;
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[OceanNode] "

OceanNode::OceanNode() :
_seaLevel( 0.0f )
{
    //nop
}

OceanNode::~OceanNode()
{
    //nop
}

void
OceanNode::setSeaLevel(float value)
{
    _seaLevel = value;
    onSetSeaLevel();
}

//------------------------------------------------------------------------

#define MAPNODE_TAG "__osgEarth::MapNode"
#define OPTIONS_TAG "__osgEarth::Util::OceanOptions"

OceanNode*
OceanNode::create(const OceanOptions& options,
                  MapNode*            mapNode)
{
    OceanNode* result = 0L;

    std::string driver = options.getDriver();
    if ( driver.empty() )
        driver = "simple";

    std::string driverExt = std::string(".osgearth_ocean_") + driver;

    osg::ref_ptr<osgDB::Options> rwopts = Registry::instance()->cloneOrCreateOptions();
    rwopts->setPluginData( MAPNODE_TAG, (void*)mapNode );
    rwopts->setPluginData( OPTIONS_TAG, (void*)&options );

    result = dynamic_cast<OceanNode*>( osgDB::readNodeFile( driverExt, rwopts.get() ) );
    if ( result )
    {
        OE_INFO << LC << "Loaded ocean driver \"" << driver << "\" OK." << std::endl;
    }
    else
    {
        OE_WARN << LC << "FAIL, unable to load ocean driver \"" << driver << "\"" << std::endl;
    }

    return result;
}

OceanNode*
OceanNode::create(MapNode* mapNode)
{
    OceanOptions options;
    return create(options, mapNode);
}

//------------------------------------------------------------------------

const OceanOptions&
OceanDriver::getOceanOptions(const osgDB::Options* options) const
{
    return *static_cast<const OceanOptions*>( options->getPluginData(OPTIONS_TAG) );
}


MapNode*
OceanDriver::getMapNode(const osgDB::Options* options) const
{
    return const_cast<MapNode*>(
        static_cast<const MapNode*>(
            options->getPluginData(MAPNODE_TAG) ) );
}
