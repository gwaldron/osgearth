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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include "SplatExtension"
#include "SplatCatalog"
#include "SplatCoverageLegend"
#include "SplatTerrainEffect"

#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/XmlUtils>

using namespace osgEarth;
using namespace osgEarth::Splat;

#define LC "[SplatExtension] "

//.........................................................................

REGISTER_OSGEARTH_EXTENSION(osgearth_splat, SplatExtension);


SplatExtension::SplatExtension()
{
    //nop
}

SplatExtension::SplatExtension(const SplatOptions& options) :
SplatOptions( options )
{
    //nop
}

SplatExtension::~SplatExtension()
{
    //nop
}

void
SplatExtension::setDBOptions(const osgDB::Options* dbOptions)
{
    _dbo = dbOptions;
}

bool
SplatExtension::connect(MapNode* mapNode)
{
    if ( !mapNode )
    {
        OE_WARN << LC << "Illegal: MapNode cannot be null." << std::endl;
        return false;
    }

    OE_INFO << LC << "Connecting to MapNode.\n";

    // Coverage source data
    osg::ref_ptr<Coverage> myCoverage;
    if ( coverage().isSet() )
    {
        myCoverage = new Coverage();
        if ( !myCoverage->configure( coverage().get(), mapNode->getMap(), _dbo.get() ) )
        {
            OE_WARN << LC << "Coverage is not properly configured; land cover disabled.\n";
            return false;
        }
    }

    bool enableSurfaceEffect = false;
    bool enableLandCoverEffect = false;

    // Zone definitions
    Zones myZones;
    for(int i=0; i<zones().size(); ++i)
    {
        osg::ref_ptr<Zone> zone = new Zone();
        if ( zone->configure(zones().at(i), mapNode->getMap(), _dbo.get()) )
        {
            myZones.push_back( zone.get() );

            if ( zone->getSurface() != 0L )
            {
                enableSurfaceEffect = true;
            }

            if ( zone->getLandCover() != 0L )
            {
                enableLandCoverEffect = true;
            }
        }
    }

    if ( enableSurfaceEffect )
    {
        OE_INFO << LC << "Enabling the surface splatting effect\n";
        _splatEffect = new SplatTerrainEffect();
        _splatEffect->setDBOptions( _dbo.get() );
        _splatEffect->setZones( myZones );
        _splatEffect->setCoverage( myCoverage.get() );

        mapNode->getTerrainEngine()->addEffect( _splatEffect.get() );
    }

    if ( enableLandCoverEffect )
    {
        OE_INFO << LC << "Enabling the land cover effect\n";
        _landCoverEffect = new LandCoverTerrainEffect();
        _landCoverEffect->setDBOptions( _dbo.get() );
        _landCoverEffect->setZones( myZones );
        _landCoverEffect->setCoverage( myCoverage.get() );
        
        mapNode->getTerrainEngine()->addEffect( _landCoverEffect.get() );
    }

    // Install the zone switcher; this will select the best zone based on
    // the camera position.
    _zoneSwitcher = new ZoneSwitcher(myZones);
    mapNode->getTerrainEngine()->addCullCallback( _zoneSwitcher.get() );

    return true;
}

bool
SplatExtension::disconnect(MapNode* mapNode)
{
    if ( mapNode )
    {
        if ( _splatEffect.valid() )
        {
            mapNode->getTerrainEngine()->removeEffect( _splatEffect.get() );
            _splatEffect = 0L;
        }

        if ( _landCoverEffect.valid() )
        {
            mapNode->getTerrainEngine()->removeEffect( _landCoverEffect.get() );
            _landCoverEffect = 0L;
        }

        mapNode->getTerrainEngine()->removeCullCallback( _zoneSwitcher.get() );
    }

    return true;
}

bool
SplatExtension::connect(Control* control)
{
    ////TODO add a UI.
    //Container* container = dynamic_cast<Container*>(control);
    //if ( container )
    //{
    //    container->addControl( new LabelControl("Prodecural Terrain Extension Active") );
    //}
    return true;
}

bool
SplatExtension::disconnect(Control* control)
{
    // NOP
    return true;
}
