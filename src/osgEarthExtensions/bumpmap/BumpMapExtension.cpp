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
#include "BumpMapExtension"
#include "BumpMapTerrainEffect"

#include <osgEarth/MapNode>

using namespace osgEarth;
using namespace osgEarth::BumpMap;

#define LC "[BumpMapExtension] "


BumpMapExtension::BumpMapExtension()
{
    //nop
}

BumpMapExtension::BumpMapExtension(const BumpMapOptions& options) :
_options( options )
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
    
    osg::ref_ptr<osg::Image> image = _options.imageURI()->getImage( _dbOptions.get() );
    if ( !image.valid() )
    {
        OE_WARN << LC << "Failed; unable to load normal map image from "
            << _options.imageURI()->full() << "\n";
        return false;
    }

    _effect = new BumpMapTerrainEffect( _dbOptions.get() );
    _effect->setBumpMapImage( image.get() );

    if (_options.intensity().isSet())
        _effect->getIntensityUniform()->set( _options.intensity().get() );

    if (_options.scale().isSet())
        _effect->getScaleUniform()->set( _options.scale().get() );

    if ( _options.octaves().isSet() )
        _effect->setOctaves( _options.octaves().get() );

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

