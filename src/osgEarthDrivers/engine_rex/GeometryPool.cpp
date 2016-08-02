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
#include "GeometryPool"
#include <osgEarth/Locators>
#include <osg/Point>
#include <cstdlib> // for getenv

using namespace osgEarth;
using namespace osgEarth::Drivers::RexTerrainEngine;

#define LC "[GeometryPool] "

// TODO: experiment with sharing a single texture coordinate array 
//// across all shared geometries.
/// JB:  Disabled to fix issues with ATI.
//#define SHARE_TEX_COORDS 1


GeometryPool::GeometryPool(const RexTerrainEngineOptions& options) :
_options ( options ),
_enabled ( true ),
_debug   ( false )
{
    // sign up for the update traversal so we can prune unused pool objects.
    setNumChildrenRequiringUpdateTraversal(1u);

    _tileSize = _options.tileSize().get();

    // activate debugging mode
    if ( getenv("OSGEARTH_DEBUG_REX_GEOMETRY_POOL") != 0L )
    {
        _debug = true;
    }

    if ( ::getenv("OSGEARTH_REX_NO_POOL") )
    {
        _enabled = false;
        OE_INFO << LC << "Geometry pool disabled (environment)" << std::endl;
    }

    //if ( ::getenv( "OSGEARTH_MEMORY_PROFILE" ) )
    //{
    //    _enabled = false;
    //    OE_INFO << LC << "Geometry pool disabled (memory profile mode)" << std::endl;
    //}
}

void
GeometryPool::getPooledGeometry(const TileKey&               tileKey,
                                const MapInfo&               mapInfo,
                                osg::ref_ptr<osg::Geometry>& out,
                                MaskGenerator*               maskSet)
{
    // convert to a unique-geometry key:
    GeometryKey geomKey;
    createKeyForTileKey( tileKey, _tileSize, mapInfo, geomKey );

    if ( _enabled )
    {
        // Look it up in the pool:
        Threading::ScopedMutexLock exclusive( _geometryMapMutex );

        bool masking = maskSet && maskSet->hasMasks();

        GeometryMap::iterator i = _geometryMap.find( geomKey );
        if ( !masking && i != _geometryMap.end() )
        {
            // Found. return it.
            out = i->second.get();
        }
        else
        {
            // Not found. Create it.
            out = createGeometry( tileKey, mapInfo, maskSet );

            if (!masking)
                _geometryMap[ geomKey ] = out.get();

            if ( _debug )
            {
                OE_NOTICE << LC << "Geometry pool size = " << _geometryMap.size() << "\n";
            }
        }
    }

    else
    {
        out = createGeometry( tileKey, mapInfo, maskSet );
    }
}

void
GeometryPool::createKeyForTileKey(const TileKey&             tileKey,
                                  unsigned                   size,
                                  const MapInfo&             mapInfo,
                                  GeometryPool::GeometryKey& out) const
{
    out.lod  = tileKey.getLOD();
    out.yMin = mapInfo.isGeocentric()? tileKey.getExtent().yMin() : 0.0;
    out.size = size;
}

int
GeometryPool::getNumSkirtElements() const
{
    return _options.heightFieldSkirtRatio().get() > 0.0 ? (_tileSize-1) * 4 * 6 : 0;
}

namespace
{
    int getMorphNeighborIndexOffset(unsigned col, unsigned row, int rowSize)
    {
        if ( (col & 0x1)==1 && (row & 0x1)==1 ) return rowSize+2;
        if ( (row & 0x1)==1 )                   return rowSize+1;
        if ( (col & 0x1)==1 )                   return 2;
        return 1;            
    }
}

#define addSkirtDataForIndex(INDEX, HEIGHT) \
{ \
    verts->push_back( (*verts)[INDEX] ); \
    normals->push_back( (*normals)[INDEX] ); \
    texCoords->push_back( (*texCoords)[INDEX] ); \
    if ( neighbors ) neighbors->push_back( (*neighbors)[INDEX] ); \
    verts->push_back( (*verts)[INDEX] - ((*normals)[INDEX])*(HEIGHT) ); \
    normals->push_back( (*normals)[INDEX] ); \
    texCoords->push_back( (*texCoords)[INDEX] ); \
    if ( neighbors ) neighbors->push_back( (*neighbors)[INDEX] - ((*normals)[INDEX])*(HEIGHT) ); \
}

#define addSkirtTriangles(INDEX0, INDEX1) \
{ \
    if ( maskSet == 0L || (!maskSet->isMasked((*texCoords)[INDEX0]) && !maskSet->isMasked((*texCoords)[INDEX1])) ) \
    { \
        primSet->addElement((INDEX0));   \
        primSet->addElement((INDEX0)+1); \
        primSet->addElement((INDEX1));   \
        primSet->addElement((INDEX1));   \
        primSet->addElement((INDEX0)+1); \
        primSet->addElement((INDEX1)+1); \
    } \
}

osg::Geometry*
GeometryPool::createGeometry(const TileKey& tileKey,
                             const MapInfo& mapInfo,
                             MaskGenerator* maskSet) const
{    
    // Establish a local reference frame for the tile:
    osg::Vec3d centerWorld;
    GeoPoint centroid;
    tileKey.getExtent().getCentroid( centroid );
    centroid.toWorld( centerWorld );

    osg::Matrix world2local, local2world;
    centroid.createWorldToLocal( world2local );
    local2world.invert( world2local );

    // Attempt to calculate the number of verts in the surface geometry.
    bool createSkirt = _options.heightFieldSkirtRatio() > 0.0f;

    unsigned numVertsInSurface    = (_tileSize*_tileSize);
    unsigned numVertsInSkirt      = createSkirt ? _tileSize*4u - 4u : 0;
    unsigned numVerts             = numVertsInSurface + numVertsInSkirt;    
    unsigned numIndiciesInSurface = (_tileSize-1) * (_tileSize-1) * 6;
    unsigned numIncidesInSkirt    = getNumSkirtElements();
    
    GLenum mode = (_options.gpuTessellation() == true) ? GL_PATCHES : GL_TRIANGLES;

    // Pre-allocate enough space for all triangles.
    osg::DrawElements* primSet = new osg::DrawElementsUShort(mode);

    primSet->reserveElements(numIndiciesInSurface + numIncidesInSkirt);

    osg::BoundingSphere tileBound;

    // the geometry:
    osg::Geometry* geom = new osg::Geometry();
    geom->setUseVertexBufferObjects(true);
    geom->setUseDisplayList(false);

    geom->addPrimitiveSet( primSet );

    // the vertex locations:
    osg::Vec3Array* verts = new osg::Vec3Array();
    verts->reserve( numVerts );
    geom->setVertexArray( verts );

    // the surface normals (i.e. extrusion vectors)
    osg::Vec3Array* normals = new osg::Vec3Array();
    normals->reserve( numVerts );
    geom->setNormalArray( normals );
    geom->setNormalBinding( geom->BIND_PER_VERTEX );

    osg::Vec3Array* neighbors = 0L;
    if ( _options.morphTerrain() == true )
    {
        // neighbor positions (for morphing)
        neighbors = new osg::Vec3Array();
        neighbors->reserve( numVerts );
        geom->setTexCoordArray( 1, neighbors );
    }

    // tex coord is [0..1] across the tile. The 3rd dimension tracks whether the
    // vert is masked: 0=yes, 1=no
#ifdef SHARE_TEX_COORDS
    bool populateTexCoords = false;
    if ( !_sharedTexCoords.valid() )
    {
        _sharedTexCoords = new osg::Vec3Array();
        _sharedTexCoords->reserve( numVerts );
        populateTexCoords = true;
    }    
    osg::Vec3Array* texCoords = _sharedTexCoords.get();
#else
    bool populateTexCoords = true;
    osg::Vec3Array* texCoords = new osg::Vec3Array();
    texCoords->reserve( numVerts );
#endif

    geom->setTexCoordArray( 0, texCoords );
    
    float delta = 1.0/(_tileSize-1);
    osg::Vec3d tdelta(delta,0,0);
    tdelta.normalize();
    osg::Vec3d vZero(0,0,0);

    osg::ref_ptr<GeoLocator> locator = GeoLocator::createForKey( tileKey, mapInfo );

    for(unsigned row=0; row<_tileSize; ++row)
    {
        float ny = (float)row/(float)(_tileSize-1);
        for(unsigned col=0; col<_tileSize; ++col)
        {
            float nx = (float)col/(float)(_tileSize-1);

            osg::Vec3d model;
            locator->unitToModel(osg::Vec3d(nx, ny, 0.0f), model);
            osg::Vec3d modelLTP = model*world2local;
            verts->push_back( modelLTP );
            tileBound.expandBy( verts->back() );

            if ( populateTexCoords )
            {
                // if masked then set textCoord z-value to 0.0
                float marker = maskSet ? maskSet->getMarker(nx, ny) : MASK_MARKER_NORMAL;
                texCoords->push_back( osg::Vec3f(nx, ny, marker) );
            }

            osg::Vec3d modelPlusOne;
            locator->unitToModel(osg::Vec3d(nx, ny, 1.0f), modelPlusOne);
            osg::Vec3d normal = (modelPlusOne*world2local)-modelLTP;                
            normal.normalize();
            normals->push_back( normal );

            // neighbor:
            if ( neighbors )
            {
                osg::Vec3d modelNeighborLTP = (*verts)[verts->size() - getMorphNeighborIndexOffset(col, row, _tileSize)];
                neighbors->push_back(modelNeighborLTP);
            }
        }
    }

    // Now tessellate the surface.
    
    // TODO: do we really need this??
    bool swapOrientation = !locator->orientationOpenGL();

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

            // skip any triangles that have a discarded vertex:
            bool discard = maskSet && (
                maskSet->isMasked( (*texCoords)[i00] ) ||
                maskSet->isMasked( (*texCoords)[i11] )
            );

            if ( !discard )
            {
                discard = maskSet && maskSet->isMasked( (*texCoords)[i01] );
                if ( !discard )
                {
                    primSet->addElement(i01);
                    primSet->addElement(i00);
                    primSet->addElement(i11);
                }
            
                discard = maskSet && maskSet->isMasked( (*texCoords)[i10] );
                if ( !discard )
                {
                    primSet->addElement(i00);
                    primSet->addElement(i10);
                    primSet->addElement(i11);
                }
            }
        }
    }

    if ( createSkirt )
    {
        // SKIRTS:
        // calculate the skirt extrusion height
        double height = tileBound.radius() * _options.heightFieldSkirtRatio().get();
        
        unsigned skirtIndex = verts->size();

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
        for(i=skirtIndex; i<(int)verts->size()-2; i+=2)
            addSkirtTriangles( i, i+2 );

        addSkirtTriangles( i, skirtIndex );
    }

    // create mask geometry
    if (maskSet)
    {
        osg::ref_ptr<osg::DrawElementsUInt> maskPrim = maskSet->createMaskPrimitives(mapInfo, verts, texCoords, normals, neighbors);
        if (maskPrim)
            geom->addPrimitiveSet( maskPrim );
    }

    return geom;
}


void
GeometryPool::setReleaser(ResourceReleaser* releaser)
{
    if (_releaser.valid())
        ADJUST_UPDATE_TRAV_COUNT(this, -1);

    _releaser = releaser;

    if (_releaser.valid())
        ADJUST_UPDATE_TRAV_COUNT(this, +1);
}


void
GeometryPool::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.UPDATE_VISITOR && _releaser.valid() && _enabled)
    {
        // look for usused pool objects and push them to the resource releaser.
        ResourceReleaser::ObjectList objects;
        {
            Threading::ScopedMutexLock exclusive( _geometryMapMutex );

            std::vector<GeometryKey> keys;

            for (GeometryMap::iterator i = _geometryMap.begin(); i != _geometryMap.end(); ++i)
            {
                if (i->second.get()->referenceCount() == 1)
                {
                    keys.push_back(i->first);
                    objects.push_back(i->second.get());
                }
            }
            for (std::vector<GeometryKey>::iterator key = keys.begin(); key != keys.end(); ++key)
            {
                _geometryMap.erase(*key);
            }
        }

        if (!objects.empty())
        {
            _releaser->push(objects);
        }
    }

    osg::Group::traverse(nv);
}
