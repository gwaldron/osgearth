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
#include "SurfaceNodeFactory"
#include <osg/ComputeBoundsVisitor>

using namespace osgEarth::Drivers::RexTerrainEngine;

#define LC "[SurfaceNodeFactory] "

#if 0
SurfaceNodeFactory::SurfaceNodeFactory(const TerrainTileModel* model,
                                       const MapFrame&         frame,
                                       const RenderBindings&   bindings,
                                       GeometryPool*           geometryPool) :
_model       ( model ),
_frame       ( frame ),
_bindings    ( bindings ),
_geometryPool( geometryPool ),
_maskSet     ( model->getKey() )
{
    //nop
}

SurfaceNode*
SurfaceNodeFactory::createSurfaceNode()
{
    _geode = new osg::Geode();

    osg::BoundingBox bbox;

    if ( _geometryPool.valid() )
    {
        osg::ref_ptr<osg::Geometry> geom;
        _geometryPool->getPooledGeometry( _model->getKey(), _frame.getMapInfo(), geom );

        TileDrawable* drawable = new TileDrawable(
            _model->getKey(),
            _bindings,
            geom.get() );
        
        for(TerrainTileImageLayerModelVector::const_iterator i = _model->colorLayers().begin();
            i != _model->colorLayers().end();
            ++i)
        {
            if ( i->get()->getTexture() )
            {
                TileDrawable::Layer r;
                r._tex = i->get()->getTexture();
                r._texMatrix = i->get()->getMatrix();
                r._imageLayer = i->get()->getImageLayer();
                r._layerID = r._imageLayer ? r._imageLayer->getUID() : -1;
                
                // a shared layer needs access to a static uniform name.
                if ( r._imageLayer->isShared() )
                {
                    r._texMatrixUniformID = osg::Uniform::getNameID( r._imageLayer->shareTexMatUniformName().get() );
                }

                drawable->_layers.push_back(r);
            }
        }

        _geode->addDrawable( drawable );
    
        bbox = computeBoundingBox();
        drawable->setInitialBound( bbox );
    }
    else
    {
        OE_WARN << LC << "INTERNAL: no geometry pool; result is an empty geode\n";
    }
    
    // Create the final node.
    SurfaceNode* node = new SurfaceNode();
    node->addChild( _geode.get() );
    node->setBoundingBox( bbox );
    
    // Establish a local reference frame for the tile:
    osg::Vec3d centerWorld;
    GeoPoint centroid;
    _model->getKey().getExtent().getCentroid(centroid);
    centroid.toWorld(centerWorld);
    osg::Matrix local2world;
    centroid.createLocalToWorld( local2world );
    node->setMatrix( local2world );

    return node;
}

osg::BoundingBox
SurfaceNodeFactory::computeBoundingBox() const
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
