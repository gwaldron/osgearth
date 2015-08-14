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
#include "MPGeometry"
#include <osg/ComputeBoundsVisitor>

using namespace osgEarth::Drivers::MPTerrainEngine;

#define LC "[SurfaceNodeFactory] "


SurfaceNodeFactory::SurfaceNodeFactory(const TerrainTileModel*       model,
                                       const MapFrame&               frame,
                                       const RenderBindings&         bindings,
                                       GeometryPool*                 geometryPool,
                                       unsigned                      tileSize,
                                       const MPTerrainEngineOptions& options) :
_model       ( model ),
_frame       ( frame ),
_bindings    ( bindings ),
_geometryPool( geometryPool ),
_tileSize    ( tileSize ),
_options     ( options ),
_requireUInt ( false ),
_maskSet     ( model->getKey() )
{
    // Create a locator for transforming coordinates between unit [0..1]
    // and world coordinates:
    _locator = GeoLocator::createForKey(
        _model->getKey(),
        _frame.getMapInfo());
    
    // Establish a local reference frame for the tile:
    osg::Vec3d centerWorld;
    GeoPoint centroid;
    _model->getKey().getExtent().getCentroid(centroid);
    centroid.toWorld(centerWorld);
    osg::Matrix world2local;
    centroid.createWorldToLocal(_world2local);
    _local2world.invert(_world2local);

    // Tile size:
    _tileSize = 17;

    // Attempt to calculate the number of verts in the surface geometry.
    _createSkirt = _options.heightFieldSkirtRatio() > 0.0f;
    unsigned numVertsInSkirt = _createSkirt ? _tileSize*4u - 4u : 0;
    _numVerts = (_tileSize*_tileSize) + numVertsInSkirt;
    
    unsigned numIndiciesInSurface = (_tileSize-1) * (_tileSize-1) * 6;
    unsigned numIncidesInSkirt   = _createSkirt ? (_tileSize-1) * 4 * 6 : 0;
    
    // Pre-allocate enought space for all triangles.
    _surfacePrimSet = newDrawElements(GL_TRIANGLES);
    _surfacePrimSet->reserveElements(numIndiciesInSurface + numIncidesInSkirt);
}

SurfaceNode*
SurfaceNodeFactory::createSurfaceNode()
{
    _geode = new osg::Geode();

    if ( _geometryPool.valid() )
    {
        osg::ref_ptr<osg::Geometry> geom;
        _geometryPool->getPooledGeometry( _model->getKey(), _frame.getMapInfo(), geom );

        TileDrawable* drawable = new TileDrawable(
            _model->getKey(),
            _frame,
            _bindings,
            geom.get() );
        
        for(TerrainTileImageLayerModelVector::const_iterator i = _model->colorLayers().begin();
            i != _model->colorLayers().end();
            ++i)
        {
            TileDrawable::Layer r;
            r._tex = i->get()->getTexture();
            r._texMatrix = i->get()->getMatrix();
            r._imageLayer = i->get()->getImageLayer();
            r._layerID = r._imageLayer ? r._imageLayer->getUID() : -1;
            drawable->_layers.push_back(r);
        }

        _geode->addDrawable( drawable );
    }
    else
    {
        addSurfaceMesh();
        addSkirtMesh();
        installColorLayers();
        _geode->addDrawable( _surfaceGeom.get() );
    }
    
    _geode->getDrawable(0)->setInitialBound( computeBoundingBox() );

    SurfaceNode* node = new SurfaceNode();
    node->setMatrix( _local2world );
    node->addChild( _geode.get() );

    return node;
}

void
SurfaceNodeFactory::addSurfaceMesh()
{
    _tileBound.init();

    // the geometry:
    _surfaceGeom = new MPGeometry( _model->getKey(), _frame, 0 );
    _geode->addDrawable( _surfaceGeom );

    // the vertex locations:
    _verts = new osg::Vec3Array();
    _verts->reserve( _numVerts );
    _surfaceGeom->setVertexArray( _verts );

    // the surface normals (i.e. extrusion vectors)
    _normals = new osg::Vec3Array();
    _normals->reserve( _numVerts );
    _surfaceGeom->setNormalArray( _normals );

    // tex coord is [0..1] across the tile. The 3rd dimension tracks whether the
    // vert is masked: 0=yes, 1=no
    _texCoords = new osg::Vec3Array();
    _texCoords->reserve( _numVerts );
    _surfaceGeom->setTexCoordArray( 0, _texCoords );

    for(unsigned row=0; row<_tileSize; ++row)
    {
        float ny = (float)row/(float)(_tileSize-1);
        for(unsigned col=0; col<_tileSize; ++col)
        {
            float nx = (float)col/(float)(_tileSize-1);

            osg::Vec3d model;
            _locator->unitToModel(osg::Vec3d(nx, ny, 0.0f), model);
            osg::Vec3d modelLTP = model*_world2local;
            _verts->push_back( modelLTP );
            _tileBound.expandBy( _verts->back() );

            bool masked = _maskSet.contains(nx, ny);            
            _texCoords->push_back( osg::Vec3f(nx, ny, masked? 0.0f : 1.0f) );

            osg::Vec3d modelPlusOne;
            _locator->unitToModel(osg::Vec3d(nx, ny, 1.0f), modelPlusOne);
            osg::Vec3f normal = (modelPlusOne*_world2local)-modelLTP;
            normal.normalize();
            _normals->push_back( normal );
        }
    }

    // Now tessellate the surface.

     // Put it first so it can "early-z" the skirts.
    _surfaceGeom->insertPrimitiveSet(0, _surfacePrimSet);
    
    // TODO: do we really need this??
    bool swapOrientation = !_locator->orientationOpenGL();

    for(unsigned j=0; j<_tileSize-1; ++j)
    {
        for(unsigned i=0; i<_tileSize-1; ++i)
        {
            int i00;
            int i01;
            if (swapOrientation)
            {
                i01 = j*_tileSize + i;
                i00 = i01+_tileSize;
            }
            else
            {
                i00 = j*_tileSize + i;
                i01 = i00+_tileSize;
            }

            int i10 = i00+1;
            int i11 = i01+1;

            // If the quad does not intersect a mask, tessellate it; otherwise skip it
            // since the mask generator will take care of it.
            if (!vertIsMasked(i00) && !vertIsMasked(i01) &&
                !vertIsMasked(i10) && !vertIsMasked(i11) &&
                !_maskSet.containedByQuadAtColRow(i, j, _tileSize) )
            {
                _surfacePrimSet->addElement(i01);
                _surfacePrimSet->addElement(i00);
                _surfacePrimSet->addElement(i11);

                _surfacePrimSet->addElement(i00);
                _surfacePrimSet->addElement(i10);
                _surfacePrimSet->addElement(i11);
            }
        }
    }
}

void
SurfaceNodeFactory::addSkirtDataForIndex(unsigned index,
                                         float    skirtHeight)
{
    _verts->push_back( (*_verts)[index] );
    _normals->push_back( (*_normals)[index] );
    _texCoords->push_back( (*_texCoords)[index] );

    _verts->push_back( (*_verts)[index] - ((*_normals)[index])*skirtHeight );
    _normals->push_back( (*_normals)[index] );
    _texCoords->push_back( (*_texCoords)[index] );
}

void
SurfaceNodeFactory::addSkirtTriangles(unsigned           index0,
                                      unsigned           index1,
                                      osg::DrawElements* elements)
{
    if ( !vertIsMasked(index0) && !vertIsMasked(index1) )
    {
        elements->addElement(index0);
        elements->addElement(index0+1);
        elements->addElement(index1);

        elements->addElement(index1);
        elements->addElement(index0+1);
        elements->addElement(index1+1);
    }
}

void
SurfaceNodeFactory::addSkirtMesh()
{
    // calculate the skirt extrusion height
    double height = _tileBound.radius() * _options.heightFieldSkirtRatio().get();
        
    unsigned skirtIndex = _verts->size();

    // first, create all the skirt verts, normals, and texcoords.
    for(int c=0; c<(int)_tileSize-1; ++c)
        addSkirtDataForIndex( c, height ); //top

    for(int r=0; r<(int)_tileSize-1; ++r)
        addSkirtDataForIndex( r*_tileSize+(_tileSize-1), height ); //right
    
    for(int c=_tileSize-1; c>=0; --c)
        addSkirtDataForIndex( (_tileSize-1)*_tileSize+c, height ); //bottom

    for(int r=_tileSize-1; r>=0; --r)
        addSkirtDataForIndex( r*_tileSize, height ); //left
    
    // then create the elements indices:
    int i;
    for(i=skirtIndex; i<(int)_verts->size()-2; i+=2)
        addSkirtTriangles( i, i+2, _surfacePrimSet );

    addSkirtTriangles( i, skirtIndex, _surfacePrimSet );
}

void
SurfaceNodeFactory::installColorLayers()
{
    for(TerrainTileImageLayerModelVector::const_iterator i = _model->colorLayers().begin();
        i != _model->colorLayers().end();
        ++i)
    {
        MPGeometry::Layer r;
        r._tex = i->get()->getTexture();
        r._texMatrix = i->get()->getMatrix();
        r._imageLayer = i->get()->getImageLayer();
        r._layerID = r._imageLayer ? r._imageLayer->getUID() : -1;
        _surfaceGeom->_layers.push_back(r);
    }
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

osg::Vec3d 
SurfaceNodeFactory::unitToWorld(const osg::Vec3d& input) const
{
    osg::Vec3d model;
    _locator->unitToModel( input, model );
    return model;
}

osg::Vec3d 
SurfaceNodeFactory::unitToLocal(const osg::Vec3d& input) const
{
    osg::Vec3d model;
    _locator->unitToModel( input, model );
    return model * _world2local;
}

bool
SurfaceNodeFactory::vertIsMasked(unsigned index) const
{
    return
        _texCoords && 
        _texCoords->size() > index &&
        (*_texCoords)[index].z() == 0.0f;
}

osg::DrawElements*
SurfaceNodeFactory::newDrawElements(GLenum mode) const
{
    if ( _requireUInt )
        return new osg::DrawElementsUInt(mode);
    else
        return new osg::DrawElementsUShort(mode);
}
