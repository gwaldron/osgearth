/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarth/TerrainEngineNode>
#include <osgDB/ReadFile>

using namespace osgEarth;

TerrainEngineNode::TerrainEngineNode( Map* map, const TerrainOptions& options ) :
_map( map ),
_terrainOptions( options )
{
    //nop
}

void
TerrainEngineNode::setVerticalScale( float value )
{
    _verticalScale = value;
    onVerticalScaleChanged();
}

void
TerrainEngineNode::setElevationSamplingRatio( float value )
{
    _elevationSamplingRatio = value;
    onElevationSamplingRatioChanged();
}

void
TerrainEngineNode::onMapProfileEstablished( const Profile* profile )
{
    // set up the CSN values
    _map->getProfile()->getSRS()->populateCoordinateSystemNode( this );
    
    // OSG's CSN likes a NULL ellipsoid to represent projected mode.
    if ( _map->getCoordinateSystemType() == Map::CSTYPE_PROJECTED )
        this->setEllipsoidModel( NULL );
}

//------------------------------------------------------------------------

#undef LC
#define LC "[TerrainEngineFactory] "

TerrainEngineNode*
TerrainEngineNodeFactory::create( const TerrainOptions& options )
{
    TerrainEngineNode* result = 0L;

    if ( !options.getDriver().empty() )
    {
        OE_INFO << LC << "Loading terrain engine from driver \"" << options.getDriver() << "\"" << std::endl;
    }

    std::string driverExt = std::string( ".osgearth_engine_" ) + options.getDriver();
    result = dynamic_cast<TerrainEngineNode*>( osgDB::readObjectFile( driverExt ) ); //, options ) );

    if ( !result )
    {
        OE_WARN << "WARNING: Failed to load terrain engine driver for \"" << options.getDriver() << "\"" << std::endl;
    }

    return result;
}

