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
#include <osg/KdTree>

using namespace osg;
using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define OSGEARTH_REX_PROXY_GEOMETRY_DEBUG 0

#define LC "[ProxyGeometry] "

void
ProxyGeometry::constructXReferenceFrame()
{
    // Establish a local reference frame for the tile:
    osg::Vec3d centerWorld;
    GeoPoint centroid;
    _key.getExtent().getCentroid( centroid );
    centroid.toWorld( centerWorld );

    centroid.createWorldToLocal( _world2local );
    _local2world.invert( _world2local );
}

void
ProxyGeometry::constructEmptyGeometry()
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

ProxyGeometry::ProxyGeometry(const TileKey& key, const MapInfo& mapInfo, unsigned tileSize, const osg::Texture* elevationTexture) : 
    _key(key), 
    _tileSize( 32 ), // temporary
    //_tileSize(elevationTexture->getImage(0)->s()),
    _elevationTexture(elevationTexture)
{
    _locator = GeoLocator::createForKey( _key, mapInfo );
}

const osg::Texture*
ProxyGeometry::getElevationTexture(void) const
{
    return _elevationTexture;
}

void
ProxyGeometry::build(void)
{
    constructEmptyGeometry();
    constructXReferenceFrame();
    makeVertices();
    tessellate();

    osg::KdTree* kd = new osg::KdTree();
    KdTree::BuildOptions kdoptions;
    kd->build(kdoptions, this);
    this->setShape(kd);

    OE_DEBUG << LC << "Built proxy geometry: "<<_key.str()<<std::endl;
}

void
ProxyGeometry::buildIfNecessary(void) const
{
    if (getVertexArray()==0)
    {
        ProxyGeometry* pNonConst = const_cast<ProxyGeometry*>(this);
        pNonConst->build();
    }   
}

void
ProxyGeometry::accept(osg::PrimitiveFunctor& f) const 
{
    buildIfNecessary();
    osg::Geometry::accept(f);
}

void
ProxyGeometry::accept(osg::PrimitiveIndexFunctor& f) const
{
    buildIfNecessary();
    osg::Geometry::accept(f);
}

void
ProxyGeometry::makeVertices()
{
    //assert(_elevationTexture && _elevationTexture->getImage(0));
    ElevationImageReader elevationImageReader(_elevationTexture->getImage(0));
    if (elevationImageReader.valid()==false)
    {
        OE_DEBUG << LC << "Error: Cannot use elevation texture. Window too small!!"<<std::endl;
        return;
    }

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

void
ProxyGeometry::tessellate(void)
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

unsigned
ProxyGeometry::getNumberOfVerts(void) const
{
    unsigned tileSize = _elevationTexture->getImage(0)->s();
    return (tileSize*tileSize);    
}

unsigned
ProxyGeometry::getNumberOfIndices(void) const
{
    unsigned tileSize = _elevationTexture->getImage(0)->s();
    return (tileSize-1) * (tileSize-1) * 6;
}