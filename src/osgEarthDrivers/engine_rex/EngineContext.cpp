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
#include "EngineContext"
#include "FileLocationCallback"
#include "SurfaceNodeFactory"

#include <osgEarth/GeoData>
#include <osgEarth/Registry>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Progress>
#include <osgEarth/Containers>
#include <osgEarth/CullingUtils>
#include <osgEarth/TerrainEngineNode>

#include <osgUtil/CullVisitor>

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[EngineContext] "

//..............................................................

namespace
{
    // Cull callback that uses a tight bounding box around the surface node.
    struct SurfaceNodeCullCallback : public osg::NodeCallback
    {
        void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);
            if ( !cv->isCulled(static_cast<SurfaceNode*>(node)->getBoundingBox()) )
                traverse(node, nv);
        }
    };
}

//..............................................................


EngineContext::EngineContext(const Map*                     map,
                             TerrainEngine*                 terrainEngine,
                             GeometryPool*                  geometryPool,
                             Loader*                        loader,
                             TileNodeRegistry*              liveTiles,
                             TileNodeRegistry*              deadTiles,
                             const RenderBindings&          renderBindings,
                             const RexTerrainEngineOptions& options) :
_frame         ( map ),
_terrainEngine ( terrainEngine ),
_geometryPool  ( geometryPool ),
_loader        ( loader ),
_liveTiles     ( liveTiles ),
_deadTiles     ( deadTiles ),
_renderBindings( renderBindings ),
_options       ( options )
{
    //NOP
}
