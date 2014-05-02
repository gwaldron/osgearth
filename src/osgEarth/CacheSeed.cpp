/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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

#include <osgEarth/CacheSeed>
#include <osgEarth/CacheEstimator>
#include <osgEarth/MapFrame>
#include <OpenThreads/ScopedLock>
#include <limits.h>

#define LC "[CacheSeed] "

using namespace osgEarth;
using namespace OpenThreads;

CacheSeed::CacheSeed():
_visitor(new TileVisitor())
{
}

TileVisitor* CacheSeed::getVisitor() const
{
    return _visitor;
}

void CacheSeed::setVisitor(TileVisitor* visitor)
{
    _visitor = visitor;
}

void CacheSeed::run( TerrainLayer* layer, Map* map )
{
    _visitor->setTileHandler( new CacheTileHandler( layer, map ) );
    _visitor->run( map->getProfile() );
}