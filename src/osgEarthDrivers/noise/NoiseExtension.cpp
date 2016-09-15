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
#include "NoiseExtension"
#include "NoiseTerrainEffect"
#include <osgEarth/TerrainEngineNode>

using namespace osgEarth;
using namespace osgEarth::Noise;

#define LC "[NoiseExtension] "


REGISTER_OSGEARTH_EXTENSION( osgearth_noise, NoiseExtension )


NoiseExtension::NoiseExtension()
{
    //nop
}

NoiseExtension::NoiseExtension(const NoiseOptions& options) :
NoiseOptions( options )
{
    //nop
}

NoiseExtension::~NoiseExtension()
{
    //nop
}

void
NoiseExtension::setDBOptions(const osgDB::Options* dbOptions)
{
    _dbOptions = dbOptions;
}

bool
NoiseExtension::connect(MapNode* mapNode)
{
    if ( !mapNode )
    {
        OE_WARN << LC << "Illegal: MapNode cannot be null." << std::endl;
        return false;
    }

    _effect = new NoiseTerrainEffect( *this );

    mapNode->getTerrainEngine()->addEffect( _effect.get() );
    
    return true;
}

bool
NoiseExtension::disconnect(MapNode* mapNode)
{
    if ( mapNode )
    {
        mapNode->getTerrainEngine()->removeEffect( _effect.get() );
    }
    _effect = 0L;
    return true;
}

