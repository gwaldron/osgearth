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
#if 0
#include "SplatExtension"
#include "SplatCatalog"
#include "SplatCoverageLegend"
#include "SplatLayerFactory"
#include "NoiseTextureFactory"

#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/XmlUtils>

using namespace osgEarth;
using namespace osgEarth::Splat;

#define LC "[SplatExtension] "

//.........................................................................

//REGISTER_OSGEARTH_EXTENSION(osgearth_splat, SplatExtension);


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
    bool enableGroundCoverEffect = false;

    // Zone definitions
    Zones myZones;
    for(int i=0; i<zones().size(); ++i)
    {
        osg::ref_ptr<Zone> zone = new Zone();
        if ( zone->configure(zones()[i], mapNode->getMap(), _dbo.get()) )
        {
            myZones.push_back( zone.get() );

            if ( zone->getSurface() != 0L )
            {
                enableSurfaceEffect = true;
            }

            if ( zone->getGroundCover() != 0L )
            {
                enableGroundCoverEffect = true;
            }
        }
    }

    // Install the noise texture that's used by both effects.
    if (enableSurfaceEffect || enableGroundCoverEffect)
    {
        osg::StateSet* terrainStateSet = mapNode->getTerrainEngine()->getOrCreateStateSet();

        // reserve a texture unit:
        if (mapNode->getTerrainEngine()->getResources()->reserveTextureImageUnit(_noiseTexUnit, "Splat Noise"))
        {
            NoiseTextureFactory noise;
            terrainStateSet->setTextureAttribute(_noiseTexUnit, noise.create(256u, 4u));
            terrainStateSet->addUniform(new osg::Uniform("oe_splat_noiseTex", _noiseTexUnit));
        }
    }

    if ( enableSurfaceEffect )
    {
        OE_INFO << LC << "Enabling the surface splatting effect\n";
        _splatLayerFactory = new SplatLayerFactory();
        _splatLayerFactory->setDBOptions( _dbo.get() );
        _splatLayerFactory->setZones( myZones );
        _splatLayerFactory->setCoverage( myCoverage.get() );
        _splatLayerFactory->install(mapNode);
    }

    if ( enableGroundCoverEffect )
    {
        OE_INFO << LC << "Enabling the land cover effect\n";
        _GroundCoverLayerFactory = new GroundCoverLayerFactory();
        _GroundCoverLayerFactory->setDBOptions( _dbo.get() );
        _GroundCoverLayerFactory->setZones( myZones );
        _GroundCoverLayerFactory->setCoverage( myCoverage.get() );
        _GroundCoverLayerFactory->install(mapNode);
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
        if ( _splatLayerFactory.valid() )
        {
            _splatLayerFactory->uninstall(mapNode);
            _splatLayerFactory = 0L;
        }

        if ( _GroundCoverLayerFactory.valid() )
        {
            _GroundCoverLayerFactory->uninstall(mapNode);
            _GroundCoverLayerFactory = 0L;
        }

        mapNode->getTerrainEngine()->removeCullCallback( _zoneSwitcher.get() );

        if (_noiseTexUnit >= 0)
        {
            mapNode->getTerrainEngine()->getResources()->releaseTextureImageUnit(_noiseTexUnit);
        }
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
#endif