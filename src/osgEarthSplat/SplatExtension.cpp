/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2015 Pelican Mapping
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
#include "BiomeRegion"
#include "SplatCoverageLegend"
#include "SplatTerrainEffect"
#include "LandCoverTerrainEffect"

#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/XmlUtils>

using namespace osgEarth;
using namespace osgEarth::Splat;

#define LC "[SplatExtension] "

//.........................................................................

namespace
{
}

//.........................................................................

SplatExtension::SplatExtension()
{
    //nop
}

SplatExtension::SplatExtension(const SplatOptions& options) :
_options( options )
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
    osg::ref_ptr<Coverage> coverage;
    if ( _options.coverage().isSet() )
    {
        coverage = new Coverage();
        if ( !coverage->configure( _options.coverage().get(), mapNode->getMap(), _dbo.get() ) )
        {
            OE_WARN << LC << "Coverage is not properly configured; land cover disabled.\n";
            return false;
        }
    }

    osg::ref_ptr<Surface> surface;
    if ( _options.surface().isSet() )
    {
        surface = new Surface();
        if ( !surface->configure( _options.surface().get(), mapNode->getMap(), _dbo.get() ) )
        {
            OE_WARN << LC << "Surface data is not properly configured; surface splatting disabled.\n";
            surface = 0L;
        }
    }

    osg::ref_ptr<LandCover> landCover;
    if ( _options.landCover().isSet() )
    {
        landCover = new LandCover();
        if ( !landCover->configure( _options.landCover().get(), _dbo.get() ) )
        {
            OE_WARN << LC << "Land cover is not properly configured; land cover disabled.\n";
            landCover = 0L;
        }
    }

    if ( surface.valid() )
    {
        _splatEffect = new SplatTerrainEffect();
        _splatEffect->setDBOptions( _dbo.get() );
        _splatEffect->setCoverage( coverage.get() );
        _splatEffect->setSurface( surface.get() );

        mapNode->getTerrainEngine()->addEffect( _splatEffect.get() );
    }

    if ( landCover.valid() )
    {
        _landCoverEffect = new LandCoverTerrainEffect();
        _landCoverEffect->setDBOptions( _dbo.get() );
        _landCoverEffect->setCoverage( coverage.get() );
        _landCoverEffect->setLandCover( landCover.get() );
        
        mapNode->getTerrainEngine()->addEffect( _landCoverEffect.get() );
    }

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
    }

    return true;
}

bool
SplatExtension::connect(Control* control)
{
    //TODO add a UI.
    Container* container = dynamic_cast<Container*>(control);
    if ( container )
    {
        container->addControl( new LabelControl("Prodecural Terrain Extension Active") );
    }
    return true;
}

bool
SplatExtension::disconnect(Control* control)
{
    // NOP
    return true;
}
