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
#include "NormalMapExtension"
#include "NormalMapTerrainEffect"

#include <osgEarth/MapNode>

using namespace osgEarth;
using namespace osgEarth::Extensions::NormalMap;

#define LC "[NormalMapExtension] "


NormalMapExtension::NormalMapExtension()
{
    //nop
}

NormalMapExtension::NormalMapExtension(const NormalMapOptions& options) :
_options( options )
{
    //nop
}

NormalMapExtension::~NormalMapExtension()
{
    //nop
}

void
NormalMapExtension::setDBOptions(const osgDB::Options* dbOptions)
{
    _dbOptions = dbOptions;
}

bool
NormalMapExtension::connect(MapNode* mapNode)
{
    if ( !mapNode )
    {
        OE_WARN << LC << "Illegal: MapNode cannot be null." << std::endl;
        return false;
    }
    
    osg::ref_ptr<osg::Image> image = _options.imageURI()->getImage( _dbOptions.get() );
    if ( !image.valid() )
    {
        OE_WARN << LC << "Failed; unable to load normal map image\n";
        return false;
    }

    _effect = new NormalMapTerrainEffect( _dbOptions.get() );
    _effect->setNormalMapImage( image.get() );

    mapNode->getTerrainEngine()->addEffect( _effect.get() );

    return true;
}

bool
NormalMapExtension::disconnect(MapNode* mapNode)
{
    if ( mapNode )
    {
        mapNode->getTerrainEngine()->removeEffect( _effect.get() );
    }
    _effect = 0L;
    return true;
}

