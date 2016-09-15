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
#include "GraticuleExtension"
#include "GraticuleTerrainEffect"
#include "GraticuleNode"

#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgDB/FileNameUtils>

using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[GraticuleExtension] "

REGISTER_OSGEARTH_EXTENSION( osgearth_graticule, GraticuleExtension );


GraticuleExtension::GraticuleExtension()
{
    //nop
}

GraticuleExtension::GraticuleExtension(const GraticuleOptions& options) :
GraticuleOptions( options )
{
    //nop
}

GraticuleExtension::~GraticuleExtension()
{
    //nop
}

void
GraticuleExtension::setDBOptions(const osgDB::Options* dbOptions)
{
    _dbOptions = dbOptions;
}

bool
GraticuleExtension::connect(MapNode* mapNode)
{
    if ( !mapNode )
    {
        OE_WARN << LC << "Illegal: MapNode cannot be null." << std::endl;
        return false;
    }

   _node = new GraticuleNode(mapNode, *this);
    mapNode->addChild(_node.get());
    
    OE_INFO << LC << "Installed!\n";

    return true;
}

bool
GraticuleExtension::disconnect(MapNode* mapNode)
{
    if ( mapNode )
    {
        mapNode->removeChild(_node.get());
    }
    _node = 0L;
    return true;
}
