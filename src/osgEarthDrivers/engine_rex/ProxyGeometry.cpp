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
#include "ProxyGeometry"
#include "ElevationTextureUtils"

#include <osgEarth/Locators>

using namespace osg;
using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define OSGEARTH_REX_PROXY_GEOMETRY_DEBUG 0

#define LC "[ProxyGeometry] "

void ProxyGeometry::constructXReferenceFrame()
{
    // Establish a local reference frame for the tile:
    osg::Vec3d centerWorld;
    GeoPoint centroid;
    _key.getExtent().getCentroid( centroid );
    centroid.toWorld( centerWorld );

    centroid.createWorldToLocal( _world2local );
    _local2world.invert( _world2local );
}

void ProxyGeometry::constructEmptyGeometry()
{
    unsigned numVerts   = getNumberOfVerts();    
    unsigned numIndices = getNumberOfIndices();

    // the geometry:
    osg::Geometry* geom = this;

    // Pre-allocate enough space for all triangles.
    osg::DrawElements* primSet = new osg::DrawElementsUShort(GL_TRIANGLES);
    primSet->reserveElements(numIndices);
    geom->addPrimitiveSet( primSet );

#if OSGEARTH_REX_PROXY_GEOMETRY_DEBUG
    geom->setUseVertexBufferObjects(true);
#endif
    geom->setUseDisplayList(false);

    // the vertex locations:
    osg::Vec3Array* verts = new osg::Vec3Array();
    verts->reserve( numVerts );
    geom->setVertexArray( verts );
}

ProxyGeometry::ProxyGeometry(const TileKey& key, const MapInfo& mapInfo, unsigned tileSize) : 
    _key(key), 
    _dirty(true),
    _tileSize(tileSize),
    _elevationTexture(0)
{
    _locator = GeoLocator::createForKey( _key, mapInfo );

    constructEmptyGeometry();
    constructXReferenceFrame();
    rebuild();
}

void ProxyGeometry::setElevationData(osg::Texture* elevationTexture, osg::Matrixf& scaleBiasMatrix)
{
    if (elevationTexture!=_elevationTexture)
    {
        _elevationTexture = elevationTexture;
        _scaleBiasMatrix  = scaleBiasMatrix;
        setDirty(true);
#if OSGEARTH_REX_PROXY_GEOMETRY_DEBUG
        rebuild();
#endif
    }
}

void ProxyGeometry::rebuild(void)
{
    //assert(isDirty()==true);

    clear();

    if (_elevationTexture)
    {
        makeVertices();
        tessellate();
        OE_DEBUG << LC << "Built proxy geometry: "<<_key.getLOD()<<std::endl;
    }
    else
    {
        OE_DEBUG << LC << "Error: Could not build proxy geometry: "<<_key.getLOD()<<std::endl;
    }

    setDirty(false);
    //assert(isDirty()==false);
}

void ProxyGeometry::rebuildIfNecessary() const
{
    if (isDirty())
    {
        ProxyGeometry* pNonConst = const_cast<ProxyGeometry*>(this);
        pNonConst->rebuild();
    }
}

void ProxyGeometry::accept(osg::PrimitiveFunctor& f) const 
{
    rebuildIfNecessary();
    osg::Geometry::accept(f);
}

void ProxyGeometry::accept(osg::PrimitiveIndexFunctor& f) const
{
    rebuildIfNecessary();
    osg::Geometry::accept(f);
}

void ProxyGeometry::clear(void)
{
    osg::Geometry* geom = this;

    osg::Vec3Array* verts   = static_cast<osg::Vec3Array*>(geom->getVertexArray());
    verts->clear();

    osg::DrawElementsUShort* primSet = static_cast<osg::DrawElementsUShort*>(geom->getPrimitiveSet(0));
    primSet->clear();
}

void ProxyGeometry::makeVertices()
{
    //assert(_elevationTexture && _elevationTexture->getImage(0));
    ElevationImageReader elevationImageReader(_elevationTexture->getImage(0), _scaleBiasMatrix);
    if (elevationImageReader.valid()==false)
    {
        OE_DEBUG << LC << "Error: Cannot use elevation texture. Window too small!!"<<std::endl;
        return;
    }

    //assert(elevationImageReader.startRow()<elevationImageReader.endRow());
    //assert(elevationImageReader.startCol()<elevationImageReader.endCol());
    
    osg::Vec3Array* verts   = static_cast<osg::Vec3Array*>(this->getVertexArray());

    for(unsigned row=0; row<_tileSize; ++row)
    {
        float ny = (float)row/(float)(_tileSize-1);
        for(unsigned col=0; col<_tileSize; ++col)
        {
            float nx = (float)col/(float)(_tileSize-1);

            osg::Vec3d model;
            _locator->unitToModel(osg::Vec3d(nx, ny, 0.0f), model);
            osg::Vec3d modelLTP = model*_world2local;

            osg::Vec3d modelPlusOne;
            _locator->unitToModel(osg::Vec3d(nx, ny, 1.0f), modelPlusOne);
            osg::Vec3d normal = (modelPlusOne*_world2local)-modelLTP;
            normal.normalize();

            modelLTP = modelLTP + normal*elevationImageReader.elevationN(nx,ny);
            verts->push_back( modelLTP);
        }
    }
    verts->dirty();
}

void ProxyGeometry::tessellate(void)
{
    bool swapOrientation = !_locator->orientationOpenGL();

    osg::DrawElements* primSet = static_cast<osg::DrawElements*>(this->getPrimitiveSet(0));

    int i00;
    int i01;
    for(unsigned j=0; j<_tileSize-1; ++j)
    {
        for(unsigned i=0; i<_tileSize-1; ++i)
        {
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

            primSet->addElement(i01);
            primSet->addElement(i00);
            primSet->addElement(i11);

            primSet->addElement(i00);
            primSet->addElement(i10);
            primSet->addElement(i11);
        }
    }
    primSet->dirty();
}

unsigned ProxyGeometry::getNumberOfVerts(void) const
{
    return (_tileSize*_tileSize);    
}
unsigned ProxyGeometry::getNumberOfIndices(void) const
{
    return (_tileSize-1) * (_tileSize-1) * 6;
}