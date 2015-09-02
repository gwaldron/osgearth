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

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[EngineContext] "

//..............................................................


EngineContext::EngineContext(const Map*                     map,
                             TerrainEngine*                 terrainEngine,
                             GeometryPool*                  geometryPool,
                             Loader*                        loader,
                             TileNodeRegistry*              liveTiles,
                             TileNodeRegistry*              deadTiles,
                             const LandCoverBins*           landCoverBins,
                             const RenderBindings&          renderBindings,
                             const RexTerrainEngineOptions& options,
                             const SelectionInfo&           selectionInfo) :
_frame         ( map ),
_terrainEngine ( terrainEngine ),
_geometryPool  ( geometryPool ),
_loader        ( loader ),
_liveTiles     ( liveTiles ),
_deadTiles     ( deadTiles ),
_landCoverBins ( landCoverBins ),
_renderBindings( renderBindings ),
_options       ( options ),
_selectionInfo ( selectionInfo )
{
    //NOP
}

void
EngineContext::startCull()
{
    _tick = osg::Timer::instance()->tick();
    _tilesLastCull = _liveTiles->size();
}

double
EngineContext::getElapsedCullTime() const
{
    osg::Timer_t now = osg::Timer::instance()->tick();
    return osg::Timer::instance()->delta_s(_tick, now);
}

void
EngineContext::endCull()
{
    double tms = 1000.0 * getElapsedCullTime();
    int tileDelta = _liveTiles->size() - _tilesLastCull;
    if ( tileDelta != 0 )
    {
        OE_DEBUG << LC << "Cull time = " << tms << " ms" << ", tile delta = " << tileDelta << "; avt = " << tms/(double)tileDelta << " ms\n";
    }
}

bool
EngineContext::maxLiveTilesExceeded() const
{
    return _liveTiles->size() > _options.expirationThreshold().get();
}
