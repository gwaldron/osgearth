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
#include "SurfaceNode"
#include "GeometryPool"
#include "TileDrawable"
#include <osgEarth/TileKey>
#include <osg/Geode>

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[SurfaceNode] "

SurfaceNode::SurfaceNode(const TileKey& tilekey,
                         const MapInfo& mapinfo,
                         const RenderBindings& bindings,
                         GeometryPool*  pool)
{
    osg::Geode* geode = new osg::Geode();

    osg::BoundingBox bbox;

    if ( pool )
    {
        osg::ref_ptr<osg::Geometry> geom;
        pool->getPooledGeometry( tilekey, mapinfo, geom );

        TileDrawable* drawable = new TileDrawable(tilekey, bindings, geom.get());

        geode->addDrawable( drawable );
    
        //TODO: create a proper bounding box.
        //bbox = computeBoundingBox();
        //drawable->setInitialBound( bbox );
    }
    else
    {
        OE_WARN << LC << "INTERNAL: no geometry pool; result is an empty geode\n";
    }
    
    // Create the final node.
    addChild( geode );

    // TODO: reinstate this 
    //node->setBoundingBox( bbox );
    
    // Establish a local reference frame for the tile:
    osg::Vec3d centerWorld;
    GeoPoint centroid;
    tilekey.getExtent().getCentroid(centroid);
    centroid.toWorld(centerWorld);
    osg::Matrix local2world;
    centroid.createLocalToWorld( local2world );
    setMatrix( local2world );
}

#if 0
osg::BoundingBox
SurfaceNodeFactory::calculateBoundingBox(osg::Node* node) const
{
    osg::ComputeBoundsVisitor cbv;
    _geode->accept( cbv );

    if ( _model->elevationModel().valid() )
    {
        if ( _model->elevationModel()->getMaxHeight() > cbv.getBoundingBox().zMax() )
        {
            cbv.getBoundingBox().zMax() = _model->elevationModel()->getMaxHeight();
        }
        if ( _model->elevationModel()->getMinHeight() < cbv.getBoundingBox().zMin() )
        {
            cbv.getBoundingBox().zMin() = _model->elevationModel()->getMinHeight();
        }
    } 

    return cbv.getBoundingBox();
}
#endif