/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
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
#include "TileNodeRegistry"
#include <osgEarth/TraversalData>
#include <osgEarth/CullingUtils>
#include <osgEarth/Registry>

using namespace osgEarth::REX;
using namespace osgEarth;

#define LC "[EngineContext] "

//#define PROFILE 1

//..............................................................


EngineContext::EngineContext(const Map*                     map,
                             TerrainEngineNode*             terrainEngine,
                             GeometryPool*                  geometryPool,
                             Loader*                        loader,
                             TileNodeRegistry*              liveTiles,
                             const RenderBindings&          renderBindings,
                             const TerrainOptions&          options,
                             const SelectionInfo&           selectionInfo,
                             const FrameClock*              clock) :
_map           ( map ),
_terrainEngine ( terrainEngine ),
_geometryPool  ( geometryPool ),
_loader        ( loader ),
_liveTiles     ( liveTiles ),
_renderBindings( renderBindings ),
_options       ( options ),
_selectionInfo ( selectionInfo ),
_tick(0),
_tilesLastCull(0),
_clock(clock)
{
    _expirationRange2 = _options.minExpiryRange().get() * _options.minExpiryRange().get();
    _bboxCB = new ModifyBoundingBoxCallback(this);
}

osg::ref_ptr<const Map>
EngineContext::getMap() const
{
    osg::ref_ptr<const Map> map;
    _map.lock(map);
    return map;
}
