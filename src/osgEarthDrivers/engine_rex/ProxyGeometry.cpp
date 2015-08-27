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
#include <osgEarth/QuadTree>
#include <osg/KdTree>

using namespace osg;
using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define OSGEARTH_REX_PROXY_GEOMETRY_DEBUG 0

#define LC "[ProxyGeometry] "

IndexPool ProxyGeometry::_indexPool;
namespace 
{
    unsigned getNumberOfVerts(unsigned tileSize)
    {
        return (tileSize*tileSize);    
    }

    unsigned getNumberOfIndices(unsigned tileSize)
    {
        return (tileSize-1) * (tileSize-1) * 6;
    }

    unsigned getNumTriangles(unsigned tileSize)
    {
        return (tileSize-1)*(tileSize-1)*2;
    }

    void tessellate(osg::DrawElements* primSet, unsigned tileSize, bool swapOrientation)
    {
        int i00;
        int i01;
        for(unsigned j=0; j<tileSize-1; ++j)
        {
            for(unsigned i=0; i<tileSize-1; ++i)
            {
                if (swapOrientation)
                {
                    i01 = j*tileSize + i;
                    i00 = i01+tileSize;
                }
                else
                {
                    i00 = j*tileSize + i;
                    i01 = i00+tileSize;
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
}

DrawElements* IndexPool::getOrCreate(unsigned tileSize, bool swapOrientation)
{
    MapTileSizeToIndices::const_iterator it = _mapIndices.find(tileSize);
    if (it!=_mapIndices.end())
    {
        return it->second.get();
    }

    OE_INFO << LC <<" Constructing Indices for TileSize: "<<tileSize<<std::endl;

    // Pre-allocate enough space for all triangles.
    osg::DrawElements* primSet = new osg::DrawElementsUShort(GL_TRIANGLES);
    primSet->reserveElements(getNumberOfIndices(tileSize));
    _mapIndices.insert(std::make_pair(tileSize, primSet));
    tessellate(primSet, tileSize, swapOrientation);

    return primSet;
}

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
    // the geometry:
    osg::Geometry* geom = this;

    // Pre-allocate enough space for all triangles.
    osg::DrawElements* primSet = _indexPool.getOrCreate(_tileSize, !_locator->orientationOpenGL());
    geom->addPrimitiveSet(primSet);

#if OSGEARTH_REX_PROXY_GEOMETRY_DEBUG
    geom->setUseVertexBufferObjects(true);
#endif
    geom->setUseDisplayList(false);

    // the vertex locations:
    osg::Vec3Array* verts = new osg::Vec3Array();
    verts->reserve( getNumberOfVerts(_tileSize) );
    geom->setVertexArray( verts );
}

ProxyGeometry::ProxyGeometry(const TileKey& key, const MapInfo& mapInfo, unsigned tileSize, const osg::Texture* elevationTexture) : 
    _key(key), 
    //_tileSize( 32 ), // temporary
    _tileSize(elevationTexture->getImage(0)->s()),
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
    OE_START_TIMER(b1);
    makeVertices();
    double t1 = OE_STOP_TIMER(b1);

    QuadTree* quadTree = new QuadTree();
    QuadTree::BuildOptions options;
    options._maxNumLevels = 10;    // PPP: Auto compute an estimate?
    options._targetNumTrianglesPerLeaf = 16*16;    // PPP: Auto compute an estimate?
    options._numTriangles = getNumTriangles(_tileSize);
    quadTree->build(options, this);
    this->setShape(quadTree);


    OE_INFO << LC << "Built proxy: "<<_key.str()<<", t=" << t1*1000 << " ms\n";
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

            //modelLTP = modelLTP + normal*elevationImageReader.elevationN(nx,ny);
            verts->push_back( modelLTP);
        }
    }
    verts->dirty();
}
