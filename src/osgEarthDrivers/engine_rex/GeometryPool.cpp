/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
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
#include <osgEarth/NodeUtils>
#include <osgEarth/TopologyGraph>
#include <osg/Point>
#include <cstdlib> // for getenv

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::REX;

#define LC "[GeometryPool] "

// TODO: experiment with sharing a single texture coordinate array 
//// across all shared geometries.
/// JB:  Disabled to fix issues with ATI.
//#define SHARE_TEX_COORDS 1

//struct DebugGeometry : public osg::Geometry {
//    void compileGLObjects(osg::RenderInfo& renderInfo) const {
//        OE_WARN << "Compiling GL Objects: " << this << std::endl;
//        osg::Geometry::compileGLObjects(renderInfo);
//    }
//    void releaseGLObjects(osg::State* state) const {
//        OE_WARN << "Releasing GL Objects: " << this << std::endl;
//        osg::Geometry::releaseGLObjects(state);
//    }
//};


GeometryPool::GeometryPool(const TerrainOptions& options) :
_options ( options ),
_enabled ( true ),
_debug   ( false )
{
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
GeometryPool::getPooledGeometry(const TileKey&                tileKey,
                                unsigned                      tileSize,
                                MaskGenerator*                maskSet,
                                osg::ref_ptr<SharedGeometry>& out)
{
    // convert to a unique-geometry key:
    GeometryKey geomKey;
    createKeyForTileKey( tileKey, tileSize, geomKey );

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
            out = createGeometry( tileKey, tileSize, maskSet );

            if (!masking && out.valid())
            {
                _geometryMap[ geomKey ] = out.get();
            }

            if ( _debug )
            {
                OE_NOTICE << LC << "Geometry pool size = " << _geometryMap.size() << "\n";
            }
        }
    }

    else
    {
        out = createGeometry( tileKey, tileSize, maskSet );
    }
}

void
GeometryPool::createKeyForTileKey(const TileKey&             tileKey,
                                  unsigned                   tileSize,
                                  GeometryPool::GeometryKey& out) const
{
    out.lod  = tileKey.getLOD();
    out.tileY = tileKey.getProfile()->getSRS()->isGeographic()? tileKey.getTileY() : 0;
    out.size = tileSize;
}

int
GeometryPool::getNumSkirtElements(unsigned tileSize) const
{
    return _options.heightFieldSkirtRatio().get() > 0.0 ? (tileSize-1) * 4 * 6 : 0;
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

    struct Sort_by_X {
        osg::Vec3Array& _verts;
        Sort_by_X(osg::Vec3Array* verts) : _verts(*verts) { }
        bool operator()(unsigned lhs, unsigned rhs) const {
            if (_verts[lhs].x() < _verts[rhs].x()) return true;
            if (_verts[lhs].x() > _verts[rhs].x()) return false;
            return _verts[lhs].y() < _verts[rhs].y();
        }
    };

    struct Sort_by_Y {
        osg::Vec3Array& _verts;
        Sort_by_Y(osg::Vec3Array* verts) : _verts(*verts) { }
        bool operator()(unsigned lhs, unsigned rhs) const {
            if (_verts[lhs].y() < _verts[rhs].y()) return true;
            if (_verts[lhs].y() > _verts[rhs].y()) return false;
            return _verts[lhs].x() < _verts[rhs].x();
        }
    };
}

#define addSkirtDataForIndex(INDEX, HEIGHT) \
{ \
    verts->push_back( (*verts)[INDEX] ); \
    normals->push_back( (*normals)[INDEX] ); \
    texCoords->push_back( (*texCoords)[INDEX] ); \
    texCoords->back().z() = (float)((int)texCoords->back().z() | VERTEX_MARKER_SKIRT); \
    if ( neighbors ) neighbors->push_back( (*neighbors)[INDEX] ); \
    if ( neighborNormals ) neighborNormals->push_back( (*neighborNormals)[INDEX] ); \
    verts->push_back( (*verts)[INDEX] - ((*normals)[INDEX])*(HEIGHT) ); \
    normals->push_back( (*normals)[INDEX] ); \
    texCoords->push_back( (*texCoords)[INDEX] ); \
    texCoords->back().z() = (float)((int)texCoords->back().z() | VERTEX_MARKER_SKIRT); \
    if ( neighbors ) neighbors->push_back( (*neighbors)[INDEX] - ((*normals)[INDEX])*(HEIGHT) ); \
    if ( neighborNormals ) neighborNormals->push_back( (*neighborNormals)[INDEX] ); \
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

#define addMaskSkirtTriangles(INDEX0, INDEX1) \
{ \
    primSet->addElement((INDEX0));   \
    primSet->addElement((INDEX0)+1); \
    primSet->addElement((INDEX1));   \
    primSet->addElement((INDEX1));   \
    primSet->addElement((INDEX0)+1); \
    primSet->addElement((INDEX1)+1); \
}

SharedGeometry*
GeometryPool::createGeometry(const TileKey& tileKey,
                             unsigned       tileSize,
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

    unsigned numVertsInSurface    = (tileSize*tileSize);
    unsigned numVertsInSkirt      = createSkirt ? tileSize*4u - 4u : 0;
    unsigned numVerts             = numVertsInSurface + numVertsInSkirt;    
    unsigned numIndiciesInSurface = (tileSize-1) * (tileSize-1) * 6;
    unsigned numIncidesInSkirt    = getNumSkirtElements(tileSize);
    
    // TODO: reconsider this ... 
    GLenum mode = (_options.gpuTessellation() == true) ? GL_PATCHES : GL_TRIANGLES;

    osg::BoundingSphere tileBound;

    // the geometry:
    osg::ref_ptr<SharedGeometry> geom = new SharedGeometry();
    geom->setUseVertexBufferObjects(true);
    //geom->setUseDisplayList(false);

    osg::ref_ptr<osg::VertexBufferObject> vbo = new osg::VertexBufferObject();

    // Pre-allocate enough space for all triangles.
    osg::DrawElements* primSet = new osg::DrawElementsUShort(mode);
    primSet->setElementBufferObject(new osg::ElementBufferObject());
    primSet->reserveElements(numIndiciesInSurface + numIncidesInSkirt);
    geom->setDrawElements(primSet);

    // the vertex locations:
    osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array();
    verts->setVertexBufferObject(vbo.get());
    verts->reserve( numVerts );
    verts->setBinding(verts->BIND_PER_VERTEX);
    geom->setVertexArray( verts.get() );

    // the surface normals (i.e. extrusion vectors)
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array();
    normals->setVertexBufferObject(vbo.get());
    normals->reserve( numVerts );
    normals->setBinding(normals->BIND_PER_VERTEX);
    geom->setNormalArray( normals.get() );
    
    osg::ref_ptr<osg::Vec3Array> neighbors = 0L;
    osg::ref_ptr<osg::Vec3Array> neighborNormals = 0L;
    if ( _options.morphTerrain() == true )
    {
        // neighbor positions (for morphing)
        neighbors = new osg::Vec3Array();
        neighbors->setBinding(neighbors->BIND_PER_VERTEX);
        neighbors->setVertexBufferObject(vbo.get());
        neighbors->reserve( numVerts );
        geom->setNeighborArray(neighbors.get());
        
        neighborNormals = new osg::Vec3Array();
        neighborNormals->setVertexBufferObject(vbo.get());
        neighborNormals->reserve( numVerts );
        neighborNormals->setBinding(neighborNormals->BIND_PER_VERTEX);
        geom->setNeighborNormalArray( neighborNormals.get() );
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
    osg::ref_ptr<osg::Vec3Array> texCoords = new osg::Vec3Array();
    texCoords->setBinding(texCoords->BIND_PER_VERTEX);
    texCoords->setVertexBufferObject(vbo.get());
    texCoords->reserve( numVerts );
#endif

    geom->setTexCoordArray(texCoords.get());
    
    GeoLocator locator(tileKey.getExtent());

    osg::Vec3d unit;
    osg::Vec3d model;
    osg::Vec3d modelLTP;
    osg::Vec3d modelPlusOne;
    osg::Vec3d normal;

    for(unsigned row=0; row<tileSize; ++row)
    {
        float ny = (float)row/(float)(tileSize-1);
        for(unsigned col=0; col<tileSize; ++col)
        {
            float nx = (float)col/(float)(tileSize-1);

            unit.set(nx, ny, 0.0f);
            locator.unitToWorld(unit, model);
            modelLTP = model*world2local;
            verts->push_back( modelLTP );

            tileBound.expandBy( verts->back() );

            if ( populateTexCoords )
            {
                // Use the Z coord as a type marker
                float marker = maskSet ? maskSet->getMarker(nx, ny) : VERTEX_MARKER_GRID;
                texCoords->push_back( osg::Vec3f(nx, ny, marker) );
            }

            unit.z() = 1.0f;
            locator.unitToWorld(unit, modelPlusOne);
            normal = (modelPlusOne*world2local)-modelLTP;                
            normal.normalize();
            normals->push_back(normal);

            // neighbor:
            if ( neighbors )
            {
                const osg::Vec3& modelNeighborLTP = (*verts)[verts->size() - getMorphNeighborIndexOffset(col, row, tileSize)];
                neighbors->push_back(modelNeighborLTP);
            }

            if ( neighborNormals )
            {
                const osg::Vec3& modelNeighborNormalLTP = (*normals)[normals->size() - getMorphNeighborIndexOffset(col, row, tileSize)];
                neighborNormals->push_back(modelNeighborNormalLTP);
            }
        }
    }

    // By default we tessellate the surface, but if there's a masking set
    // it might replace some or all of our surface geometry.
    bool tessellateSurface = true;
                    
    if (maskSet)
    {
        // The mask generator adds to the passed-in arrays as necessary,
        // and then returns a new primtive set containing all the new triangles.
        osg::ref_ptr<osg::DrawElementsUInt> maskElements;

        MaskGenerator::Result r = maskSet->createMaskPrimitives(
            verts.get(), texCoords.get(), normals.get(), neighbors.get(), neighborNormals.get(),
            maskElements);

        if (r == MaskGenerator::R_BOUNDARY_INTERSECTS_TILE && 
            maskElements.valid() && 
            maskElements->size() > 0)
        {
            // Share the same EBO as the surface geometry
            maskElements->setElementBufferObject(primSet->getElementBufferObject());
            geom->setMaskElements(maskElements.get());

            // Build a skirt for the mask geometry?
            if (createSkirt)
            {
                // calculate the skirt extrusion height
                double height = tileBound.radius() * _options.heightFieldSkirtRatio().get();

                // Construct a node+edge graph out of the masking geometry:
                osg::ref_ptr<TopologyGraph> graph = TopologyBuilder::create(verts.get(), maskElements.get(), tileKey.str());

                // Extract the boundaries (if the topology is discontinuous,
                // there will be more than one)
                for (unsigned i = 0; i<graph->getNumBoundaries(); ++i)
                {
                    TopologyGraph::IndexVector boundary;
                    graph->createBoundary(i, boundary);

                    if (boundary.size() >= 3)
                    {
                        unsigned skirtIndex = verts->size();

                        for (TopologyGraph::IndexVector::const_iterator i = boundary.begin(); i != boundary.end(); ++i)
                        {
                            addSkirtDataForIndex((*i)->index(), height);
                        }

                        // then create the elements:
                        int i;
                        for (i = skirtIndex; i < (int)verts->size() - 2; i += 2)
                            addSkirtTriangles(i, i + 2);

                        addSkirtTriangles(i, skirtIndex);
                    }
                }
            }
        }

        // If the boundary doesn't intersect the tile, draw the entire tile
        // as we normally would. Need to reset the masking marker.
        else if (r == MaskGenerator::R_BOUNDARY_DOES_NOT_INTERSECT_TILE)
        {
            maskSet = 0L;
            for (osg::Vec3Array::iterator i = texCoords->begin(); i != texCoords->end(); ++i)
                i->z() = VERTEX_MARKER_GRID;
        }

        // If the boundary contains the entire tile, draw nothing!
        else // if (r == MaskGenerator::R_BOUNDARY_CONTAINS_ENTIRE_TILE)
        {
            tessellateSurface = false;
        }
    }

    // Now tessellate the (unmasked) surface.
    
    if (tessellateSurface)
    {
        // TODO: do we really need this??
        bool swapOrientation = false; //!locator->orientationOpenGL();

        for(unsigned j=0; j<tileSize-1; ++j)
        {
            for(unsigned i=0; i<tileSize-1; ++i)
            {
                int i00;
                int i01;
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

        // Build skirts for the tile geometry
        if ( createSkirt )
        {
            // SKIRTS:
            // calculate the skirt extrusion height
            double height = tileBound.radius() * _options.heightFieldSkirtRatio().get();
        
            unsigned skirtIndex = verts->size();

            // first, create all the skirt verts, normals, and texcoords.
            for(int c=0; c<(int)tileSize-1; ++c)
                addSkirtDataForIndex( c, height ); //top

            for(int r=0; r<(int)tileSize-1; ++r)
                addSkirtDataForIndex( r*tileSize+(tileSize-1), height ); //right
    
            for(int c=tileSize-1; c>=0; --c)
                addSkirtDataForIndex( (tileSize-1)*tileSize+c, height ); //bottom

            for(int r=tileSize-1; r>=0; --r)
                addSkirtDataForIndex( r*tileSize, height ); //left
    
            // then create the elements indices:
            int i;
            for(i=skirtIndex; i<(int)verts->size()-2; i+=2)
                addSkirtTriangles( i, i+2 );

            addSkirtTriangles( i, skirtIndex );
        }
    }

    return geom.release();
}


void
GeometryPool::setReleaser(ResourceReleaser* releaser)
{
    if (_releaser.valid())
        ADJUST_EVENT_TRAV_COUNT(this, -1);

    _releaser = releaser;

    if (_releaser.valid())
        ADJUST_EVENT_TRAV_COUNT(this, +1);
}


void
GeometryPool::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.EVENT_VISITOR && _releaser.valid() && _enabled)
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
                    
                    OE_DEBUG << "Releasing: " << i->second.get() << std::endl;
                }
            }
            for (std::vector<GeometryKey>::iterator key = keys.begin(); key != keys.end(); ++key)
            {
                if (_geometryMap[*key]->referenceCount() != 2) // one for the map, and one for the local objects list
                    OE_WARN << LC << "Erasing key geom with refcount <> 2" << std::endl;

                _geometryMap.erase(*key);
            }

            //OE_INFO << "Released " << keys.size() << ", pool = " << _geometryMap.size() << std::endl;
        }

        if (!objects.empty())
        {
            _releaser->push(objects);
        }
    }

    osg::Group::traverse(nv);
}


void
GeometryPool::clear()
{
    if (!_releaser.valid() || !_enabled)
        return;

    ResourceReleaser::ObjectList objects;

    // collect all objects in a thread safe manner
    {
        Threading::ScopedMutexLock exclusive( _geometryMapMutex );

        for (GeometryMap::iterator i = _geometryMap.begin(); i != _geometryMap.end(); ++i)
        {
            //if (i->second.get()->referenceCount() == 1)
            {
                objects.push_back(i->second.get());
            }
        }

        _geometryMap.clear();

        if (!objects.empty())
        {
            OE_INFO << LC << "Cleared " << objects.size() << " objects from the geometry pool\n";
        }
    }

    // submit to the releaser.
    if (!objects.empty())
    {
        _releaser->push(objects);
    }
}

//.........................................................................
// Code mostly adapted from osgTerrain SharedGeometry.

SharedGeometry::SharedGeometry()
{
    setSupportsDisplayList(false);
    _supportsVertexBufferObjects = true;
    _ptype.resize(64u);
    _ptype.setAllElementsTo(GL_TRIANGLES);
}

SharedGeometry::SharedGeometry(const SharedGeometry& rhs,const osg::CopyOp& copyop):
    osg::Drawable(rhs, copyop),
    _vertexArray(rhs._vertexArray),
    _normalArray(rhs._normalArray),
    _texcoordArray(rhs._texcoordArray),
    _neighborArray(rhs._neighborArray),
    _neighborNormalArray(rhs._neighborNormalArray),
    _drawElements(rhs._drawElements),
    _maskElements(rhs._maskElements)
{
    //nop
}

SharedGeometry::~SharedGeometry()
{
    //nop
}

bool 
SharedGeometry::empty() const
{
    return
        (_drawElements.valid() == false || _drawElements->getNumIndices() == 0) &&
        (_maskElements.valid() == false || _maskElements->getNumIndices() == 0);
}

#ifdef SUPPORTS_VAO
#if OSG_MIN_VERSION_REQUIRED(3,5,9)
osg::VertexArrayState* SharedGeometry::createVertexArrayStateImplementation(osg::RenderInfo& renderInfo) const
#else
osg::VertexArrayState* SharedGeometry::createVertexArrayState(osg::RenderInfo& renderInfo) const
#endif
{
    osg::State& state = *renderInfo.getState();

    osg::VertexArrayState* vas = new osg::VertexArrayState(&state);

    if (_vertexArray.valid()) vas->assignVertexArrayDispatcher();
    if (_normalArray.valid()) vas->assignNormalArrayDispatcher();
    unsigned texUnits = 0;
    if (_neighborArray.valid())
    {
        texUnits = 3;
    }
    else if (_texcoordArray.valid())
    {
        texUnits = 1;
    }
    if (texUnits)
        vas->assignTexCoordArrayDispatcher(texUnits);

    if (state.useVertexArrayObject(_useVertexArrayObject))
    {
        vas->generateVertexArrayObject();
    }

    return vas;
}
#endif

void SharedGeometry::resizeGLObjectBuffers(unsigned int maxSize)
{
    osg::Drawable::resizeGLObjectBuffers(maxSize);

    if (_vertexArray.valid()) _vertexArray->resizeGLObjectBuffers(maxSize);
    if (_normalArray.valid()) _normalArray->resizeGLObjectBuffers(maxSize);
    if (_colorArray.valid()) _colorArray->resizeGLObjectBuffers(maxSize);
    if (_texcoordArray.valid()) _texcoordArray->resizeGLObjectBuffers(maxSize);
    if (_neighborArray.valid()) _neighborArray->resizeGLObjectBuffers(maxSize);
    if (_neighborNormalArray.valid()) _neighborNormalArray->resizeGLObjectBuffers(maxSize);
    if (_drawElements.valid()) _drawElements->resizeGLObjectBuffers(maxSize);
    if (_maskElements.valid()) _maskElements->resizeGLObjectBuffers(maxSize);

    //osg::BufferObject* vbo = _vertexArray->getVertexBufferObject();
    //if (vbo) vbo->resizeGLObjectBuffers(maxSize);

    //osg::BufferObject* ebo = _drawElements->getElementBufferObject();
    //if (ebo) ebo->resizeGLObjectBuffers(maxSize);
}

void SharedGeometry::releaseGLObjects(osg::State* state) const
{
    osg::Drawable::releaseGLObjects(state);

    if (_vertexArray.valid()) _vertexArray->releaseGLObjects(state);
    if (_normalArray.valid()) _normalArray->releaseGLObjects(state);
    if (_colorArray.valid()) _colorArray->releaseGLObjects(state);
    if (_texcoordArray.valid()) _texcoordArray->releaseGLObjects(state);
    if (_neighborArray.valid()) _neighborArray->releaseGLObjects(state);
    if (_neighborNormalArray.valid()) _neighborNormalArray->releaseGLObjects(state);
    if (_drawElements.valid()) _drawElements->releaseGLObjects(state);
    if (_maskElements.valid()) _maskElements->releaseGLObjects(state);

    //osg::BufferObject* vbo = _vertexArray->getVertexBufferObject();
    //if (vbo) vbo->releaseGLObjects(state);

    //osg::BufferObject* ebo = _drawElements->getElementBufferObject();
    //if (ebo) ebo->releaseGLObjects(state);
}

// called from DrawTileCommand
void SharedGeometry::drawImplementation(osg::RenderInfo& renderInfo) const
{
    osg::State& state = *renderInfo.getState();
    
#if OSG_VERSION_LESS_THAN(3,5,6)
    osg::ArrayDispatchers& dispatchers = state.getArrayDispatchers();
#else
    osg::AttributeDispatchers& dispatchers = state.getAttributeDispatchers();
#endif

    dispatchers.reset();
    dispatchers.setUseVertexAttribAlias(state.getUseVertexAttributeAliasing());
    dispatchers.activateNormalArray(_normalArray.get());

#ifdef SUPPORTS_VAO
    osg::VertexArrayState* vas = state.getCurrentVertexArrayState();
    
    if (!state.useVertexArrayObject(_useVertexArrayObject) || vas->getRequiresSetArrays())
    {
        vas->lazyDisablingOfVertexAttributes();

        // set up arrays
        if( _vertexArray.valid() )
            vas->setVertexArray(state, _vertexArray.get());

        if (_normalArray.valid() && _normalArray->getBinding()==osg::Array::BIND_PER_VERTEX)
            vas->setNormalArray(state, _normalArray.get());

        if (_colorArray.valid() && _colorArray->getBinding()==osg::Array::BIND_PER_VERTEX)
            vas->setColorArray(state, _colorArray.get());

        if (_texcoordArray.valid() && _texcoordArray->getBinding()==osg::Array::BIND_PER_VERTEX)
            vas->setTexCoordArray(state, 0, _texcoordArray.get());

        if (_neighborArray.valid() && _neighborArray->getBinding()==osg::Array::BIND_PER_VERTEX)
            vas->setTexCoordArray(state, 1, _neighborArray.get());

        if (_neighborNormalArray.valid() && _neighborNormalArray->getBinding()==osg::Array::BIND_PER_VERTEX)
            vas->setTexCoordArray(state, 2, _neighborNormalArray.get());

        vas->applyDisablingOfVertexAttributes(state);
    }
    else
#endif

    {
        state.lazyDisablingOfVertexAttributes();

        if( _vertexArray.valid() )
            state.setVertexPointer(_vertexArray.get());

        if (_normalArray.valid())
            state.setNormalPointer(_normalArray.get());

        if (_texcoordArray.valid())
            state.setTexCoordPointer(0, _texcoordArray.get());

        if (_neighborArray.valid())
            state.setTexCoordPointer(1, _neighborArray.get());

        if (_neighborNormalArray.valid())
            state.setTexCoordPointer(2, _neighborNormalArray.get());

        state.applyDisablingOfVertexAttributes();
    }


#ifdef SUPPORTS_VAO
    bool request_bind_unbind = !state.useVertexArrayObject(_useVertexArrayObject) || state.getCurrentVertexArrayState()->getRequiresSetArrays();
#else
    bool request_bind_unbind = true;
#endif

    GLenum primitiveType = _ptype[state.getContextID()];

    osg::GLBufferObject* ebo = _drawElements->getOrCreateGLBufferObject(state.getContextID());

    if (ebo)
    {
        /*if (request_bind_unbind)*/ state.bindElementBufferObject(ebo);

        if (_drawElements->getNumIndices() > 0u)
        {
            glDrawElements(primitiveType, _drawElements->getNumIndices(), _drawElements->getDataType(), (const GLvoid *)(ebo->getOffset(_drawElements->getBufferIndex())));
        }

        if (_maskElements.valid() && _maskElements->getNumIndices() > 0u)
        {
            glDrawElements(primitiveType, _maskElements->getNumIndices(), _maskElements->getDataType(), (const GLvoid *)(ebo->getOffset(_maskElements->getBufferIndex())));
        }

        /*if (request_bind_unbind)*/ state.unbindElementBufferObject();
    }
    else
    {
        if (_drawElements->getNumIndices() > 0u)
        {
            glDrawElements(primitiveType, _drawElements->getNumIndices(), _drawElements->getDataType(), _drawElements->getDataPointer());
        }

        if (_maskElements.valid() && _maskElements->getNumIndices() > 0u)
        {
            glDrawElements(primitiveType, _maskElements->getNumIndices(), _maskElements->getDataType(), _maskElements->getDataPointer());
        }
    }

    // unbind the VBO's if any are used.
    if (request_bind_unbind)
    {
        state.unbindVertexBufferObject();
    }
}

void SharedGeometry::accept(osg::Drawable::AttributeFunctor& af)
{
    osg::AttributeFunctorArrayVisitor afav(af);

    afav.applyArray(VERTICES,_vertexArray.get());
    afav.applyArray(NORMALS, _normalArray.get());
    afav.applyArray(TEXTURE_COORDS_0,_texcoordArray.get());
    afav.applyArray(TEXTURE_COORDS_1,_neighborArray.get());
    afav.applyArray(TEXTURE_COORDS_2,_neighborNormalArray.get());
}

void SharedGeometry::accept(osg::Drawable::ConstAttributeFunctor& af) const
{
    osg::ConstAttributeFunctorArrayVisitor afav(af);

    afav.applyArray(VERTICES,_vertexArray.get());
    afav.applyArray(NORMALS, _normalArray.get());
    afav.applyArray(TEXTURE_COORDS_0,_texcoordArray.get());
    afav.applyArray(TEXTURE_COORDS_1,_neighborArray.get());
    afav.applyArray(TEXTURE_COORDS_2,_neighborNormalArray.get());
}

void SharedGeometry::accept(osg::PrimitiveFunctor& pf) const
{
    pf.setVertexArray(_vertexArray->getNumElements(),static_cast<const osg::Vec3*>(_vertexArray->getDataPointer()));
    _drawElements->accept(pf);
}

void SharedGeometry::accept(osg::PrimitiveIndexFunctor& pif) const
{
    pif.setVertexArray(_vertexArray->getNumElements(),static_cast<const osg::Vec3*>(_vertexArray->getDataPointer()));
    _drawElements->accept(pif);
}

osg::Geometry*
SharedGeometry::makeOsgGeometry()
{
    osg::Geometry* geom = new osg::Geometry();
    geom->setUseVertexBufferObjects(true);
    geom->setUseDisplayList(false);

    geom->setVertexArray(getVertexArray());
    geom->setNormalArray(getNormalArray());
    geom->setTexCoordArray(0, getTexCoordArray());
    if (getDrawElements())
        geom->addPrimitiveSet(getDrawElements());
    if (getMaskElements())
        geom->addPrimitiveSet(getMaskElements());

    return geom;
}
