/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#include "BYOTerrainEngineNode"
#include "BYOTerrainEngineOptions"

#define LC "[BYOTerrainEngineNode] "

using namespace osgEarth_engine_byo;
using namespace osgEarth;
using namespace osgEarth::Drivers;

//------------------------------------------------------------------------

BYOTerrainEngineNode::BYOTerrainEngineNode() :
TerrainEngineNode( )
{
    //nop
}

BYOTerrainEngineNode::~BYOTerrainEngineNode()
{
    //nop
}

void
BYOTerrainEngineNode::preInitialize( const Map* map, const TerrainOptions& options )
{
    TerrainEngineNode::preInitialize( map, options );
    BYOTerrainEngineOptions myoptions(options);
    if ( myoptions.getNode() )
    {
        this->addChild( myoptions.getNode() );
    }
    else if ( myoptions.url().isSet() )
    {
        osg::Node* node = myoptions.url()->getNode();
        if ( node )
        {
            this->addChild( node );
        }
    }
}
