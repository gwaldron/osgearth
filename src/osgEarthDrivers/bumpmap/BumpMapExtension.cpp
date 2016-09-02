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
#include "BumpMapExtension"
#include "BumpMapTerrainEffect"

#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>

using namespace osgEarth;
using namespace osgEarth::BumpMap;

#define LC "[BumpMapExtension] "


REGISTER_OSGEARTH_EXTENSION(osgearth_bumpmap, BumpMapExtension);
REGISTER_OSGEARTH_EXTENSION(osgearth_bump_map, BumpMapExtension);


BumpMapExtension::BumpMapExtension()
{
    //nop
}

BumpMapExtension::BumpMapExtension(const BumpMapOptions& options) :
BumpMapOptions( options )
{
    //nop
}

BumpMapExtension::~BumpMapExtension()
{
    //nop
}

void
BumpMapExtension::setDBOptions(const osgDB::Options* dbOptions)
{
    _dbOptions = dbOptions;
}

bool
BumpMapExtension::connect(MapNode* mapNode)
{
    if ( !mapNode )
    {
        OE_WARN << LC << "Illegal: MapNode cannot be null." << std::endl;
        return false;
    }
    
    osg::ref_ptr<osg::Image> image = imageURI()->getImage( _dbOptions.get() );
    if ( !image.valid() )
    {
        OE_WARN << LC << "Failed; unable to load normal map image from "
            << imageURI()->full() << "\n";
        return false;
    }

    _effect = new BumpMapTerrainEffect( _dbOptions.get() );
    _effect->setBumpMapImage( image.get() );

    if (intensity().isSet())
        _effect->getIntensityUniform()->set( intensity().get() );

    if (scale().isSet())
        _effect->getScaleUniform()->set( scale().get() );

    if ( octaves().isSet() )
        _effect->setOctaves( octaves().get() );

    if ( baseLOD().isSet() )
        _effect->setBaseLOD( baseLOD().get() );

    mapNode->getTerrainEngine()->addEffect( _effect.get() );
    
    OE_INFO << LC << "Installed.\n";

    return true;
}

bool
BumpMapExtension::disconnect(MapNode* mapNode)
{
    if ( mapNode )
    {
        mapNode->getTerrainEngine()->removeEffect( _effect.get() );
    }
    _effect = 0L;
    return true;
}

