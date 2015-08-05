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
#include "ProxySurfaceNode"
#include "GeometryPool"

#include <osgEarth/TileKey>

#include <osg/Geode>

#include <osg/Geometry>

#include <numeric>

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[ProxySurfaceNode] "

//..............................................................

ProxySurfaceNode::ProxySurfaceNode(const TileKey& tilekey, const MapInfo& mapInfo, unsigned tileSize, const osg::Texture* pTexture)
{
    _proxyGeom = new ProxyGeometry(tilekey, mapInfo, tileSize, pTexture);

    _proxySurfaceGeode = new osg::Geode();
    _proxySurfaceGeode->addDrawable( _proxyGeom );

    // Create the final node.
    addChild( _proxySurfaceGeode.get() );

    // Establish a local reference frame for the tile:
    osg::Vec3d centerWorld;
    GeoPoint centroid;
    tilekey.getExtent().getCentroid(centroid);
    centroid.toWorld(centerWorld);
    osg::Matrix local2world;
    centroid.createLocalToWorld( local2world );
    setMatrix( local2world );
}
