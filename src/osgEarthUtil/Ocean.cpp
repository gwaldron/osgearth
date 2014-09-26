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
#include <osgEarthUtil/Ocean>
#include <osgEarth/Registry>
#include <osgEarth/CullingUtils>
#include <osgEarth/MapNode>
#include <osgDB/ReadFile>

using namespace osgEarth;
using namespace osgEarth::Util;

#undef  LC
#define LC "[Ocean] "

//------------------------------------------------------------------------

OceanOptions::OceanOptions(const ConfigOptions& options) :
DriverConfigOptions( options ),
_maxAltitude       ( 250000.0 )
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

OceanNode::OceanNode(const OceanOptions& options) :
_options ( options ),
_seaLevel( 0.0f )
{
    //NOP
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

void
OceanNode::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.CULL_VISITOR && _srs.valid() )
    {
        osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
        if ( cv->getCurrentCamera() )
        {
            // find the current altitude:
            osg::Vec3d eye = osg::Vec3d(0,0,0) * cv->getCurrentCamera()->getInverseViewMatrix();
            osg::Vec3d local;
            double altitude;
            _srs->transformFromWorld(eye, local, &altitude);

            // check against max altitude:
            if ( _options.maxAltitude().isSet() && altitude > *_options.maxAltitude() )
                return;
            
            // Set the near clip plane to account for an ocean sphere.
            // First, adjust for the sea level offset:
            altitude -= (double)getSeaLevel();

            // clamp the absolute value so it will work above or below sea level
            // and so we don't attempt to set the near clip below 1:
            altitude = std::max( ::fabs(altitude), 1.0 );

            // we don't want the ocean participating in the N/F calculation:
            osg::CullSettings::ComputeNearFarMode mode = cv->getComputeNearFarMode();
            cv->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );

            // visit the ocean:
            osg::Group::traverse( nv );

            cv->setComputeNearFarMode( mode );

            // just use the height above (or below) the ocean as the near clip
            // plane distance. Close enough and errs on the safe side.
            double oldNear = cv->getCalculatedNearPlane();

            double newNear = std::min( oldNear, altitude );
            if ( newNear < oldNear )
            {
                cv->setCalculatedNearPlane( newNear );
            }

            return;
        }
    }
    osg::Group::traverse( nv );
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
    {
        OE_INFO << LC << "No driver in options; defaulting to \"simple\"." << std::endl;
        OE_INFO << LC << options.getConfig().toJSON(true) << std::endl;
        driver = "simple";
    }

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
