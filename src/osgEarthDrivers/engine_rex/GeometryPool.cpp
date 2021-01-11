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
#include "MeshEditor"
#include <osgEarth/Locators>
#include <osgEarth/NodeUtils>
#include <osgEarth/TopologyGraph>
#include <osgEarth/Metrics>
#include <osg/Point>
#include <osgUtil/MeshOptimizers>
#include <cstdlib> // for getenv

using namespace osgEarth;
using namespace osgEarth::REX;

#define LC "[GeometryPool] "


GeometryPool::GeometryPool(const TerrainOptions& options) :
_options ( options ),
_enabled ( true ),
_debug   ( false ),
_geometryMapMutex("GeometryPool(OE)")
{
    ADJUST_UPDATE_TRAV_COUNT(this, +1);

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
}

void
GeometryPool::getPooledGeometry(
    const TileKey& tileKey,
    unsigned tileSize,
    const Map* map,
    osg::ref_ptr<SharedGeometry>& out,
    Cancelable* progress)
{
    // convert to a unique-geometry key:
    GeometryKey geomKey;
    createKeyForTileKey( tileKey, tileSize, geomKey );

    // make our globally shared EBO if we need it
    {
        Threading::ScopedMutexLock lock(_geometryMapMutex);
        if (!_defaultPrimSet.valid())
        {
            _defaultPrimSet = createPrimitiveSet(tileSize);
        }
    }

    // TODO: progress callback
    MeshEditor meshEditor(tileKey, tileSize, map, nullptr);

    if ( _enabled )
    {
        // if this tile conatins mask/edit, it's a unique geometry - don't share it.
        if (!meshEditor.hasEdits())
        {
            Threading::ScopedMutexLock lock(_geometryMapMutex);
            GeometryMap::iterator i = _geometryMap.find(geomKey);
            if (i != _geometryMap.end())
            {
                // found it:
                out = i->second.get();
            }
        }

        if (!out.valid())
        {
            out = createGeometry(tileKey, tileSize, meshEditor, progress);

            if (!meshEditor.hasEdits() && out.valid())
            {
                Threading::ScopedMutexLock lock(_geometryMapMutex);
                _geometryMap[geomKey] = out.get();
            }
        }
    }

    else
    {
        out = createGeometry(tileKey, tileSize, meshEditor, progress);
    }
}

void
GeometryPool::createKeyForTileKey(const TileKey& tileKey,
                                  unsigned tileSize,
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
}

#define addSkirtDataForIndex(INDEX, HEIGHT) \
{ \
    verts->push_back( (*verts)[INDEX] ); \
    normals->push_back( (*normals)[INDEX] ); \
    texCoords->push_back( (*texCoords)[INDEX] ); \
    texCoords->back().z() = (float)((int)texCoords->back().z() | VERTEX_SKIRT); \
    if ( neighbors ) neighbors->push_back( (*neighbors)[INDEX] ); \
    if ( neighborNormals ) neighborNormals->push_back( (*neighborNormals)[INDEX] ); \
    verts->push_back( (*verts)[INDEX] - ((*normals)[INDEX])*(HEIGHT) ); \
    normals->push_back( (*normals)[INDEX] ); \
    texCoords->push_back( (*texCoords)[INDEX] ); \
    texCoords->back().z() = (float)((int)texCoords->back().z() | VERTEX_SKIRT); \
    if ( neighbors ) neighbors->push_back( (*neighbors)[INDEX] - ((*normals)[INDEX])*(HEIGHT) ); \
    if ( neighborNormals ) neighborNormals->push_back( (*neighborNormals)[INDEX] ); \
}

#define addSkirtTriangles(INDEX0, INDEX1) \
{ \
    primSet->addElement((INDEX0));   \
    primSet->addElement((INDEX0)+1); \
    primSet->addElement((INDEX1));   \
    primSet->addElement((INDEX1));   \
    primSet->addElement((INDEX0)+1); \
    primSet->addElement((INDEX1)+1); \
}

osg::DrawElements*
GeometryPool::createPrimitiveSet(
    unsigned tileSize) const
{
    // Attempt to calculate the number of verts in the surface geometry.
    bool needsSkirt = _options.heightFieldSkirtRatio() > 0.0f;

    unsigned numVertsInSurface    = (tileSize*tileSize);
    unsigned numVertsInSkirt      = needsSkirt ? (tileSize-1)*2u * 4u : 0;
    unsigned numVerts             = numVertsInSurface + numVertsInSkirt;
    unsigned numIndiciesInSurface = (tileSize-1) * (tileSize-1) * 6;
    unsigned numIncidesInSkirt    = getNumSkirtElements(tileSize);

    GLenum mode = (_options.gpuTessellation() == true) ? GL_PATCHES : GL_TRIANGLES;

    osg::ref_ptr<osg::DrawElements> primSet = new osg::DrawElementsUShort(mode);
    primSet->reserveElements(numIndiciesInSurface + numIncidesInSkirt);

    // add the elements for the surface:
    tessellateSurface(tileSize, primSet.get());

    if (needsSkirt)
    {
        // add the elements for the skirt:
        int skirtBegin = numVertsInSurface;
        int skirtEnd = skirtBegin + numVertsInSkirt;
        int i;
        for(i=skirtBegin; i<(int)skirtEnd-2; i+=2)
            addSkirtTriangles( i, i+2 );
        addSkirtTriangles( i, skirtBegin );
    }

    primSet->setElementBufferObject(new osg::ElementBufferObject());

    return primSet.release();
}



void
GeometryPool::tessellateSurface(unsigned tileSize, osg::DrawElements* primSet) const
{
    for (unsigned j = 0; j < tileSize - 1; ++j)
    {
        for (unsigned i = 0; i < tileSize - 1; ++i)
        {
            int i00 = j * tileSize + i;
            int i01 = i00 + tileSize;
            int i10 = i00 + 1;
            int i11 = i01 + 1;

            primSet->addElement(i01);
            primSet->addElement(i00);
            primSet->addElement(i11);

            primSet->addElement(i00);
            primSet->addElement(i10);
            primSet->addElement(i11);
        }
    }
}

SharedGeometry*
GeometryPool::createGeometry(const TileKey& tileKey,
                             unsigned tileSize,
                             MeshEditor& editor,
                             Cancelable* progress) const
{
    OE_PROFILING_ZONE;

    // Establish a local reference frame for the tile:
    osg::Vec3d centerWorld;
    GeoPoint centroid;
    tileKey.getExtent().getCentroid( centroid );
    centroid.toWorld( centerWorld );

    osg::Matrix world2local, local2world;
    centroid.createWorldToLocal( world2local );
    local2world.invert( world2local );

    // Attempt to calculate the number of verts in the surface geometry.
    bool needsSkirt = _options.heightFieldSkirtRatio() > 0.0f;

    unsigned numVertsInSurface    = (tileSize*tileSize);
    unsigned numVertsInSkirt      = needsSkirt ? (tileSize-1)*2u * 4u : 0;
    unsigned numVerts             = numVertsInSurface + numVertsInSkirt;
    unsigned numIndiciesInSurface = (tileSize-1) * (tileSize-1) * 6;
    unsigned numIncidesInSkirt    = getNumSkirtElements(tileSize);

    GLenum mode = (_options.gpuTessellation() == true) ? GL_PATCHES : GL_TRIANGLES;

    osg::BoundingSphere tileBound;

    // the geometry:
    osg::ref_ptr<SharedGeometry> geom = new SharedGeometry();

    osg::ref_ptr<osg::VertexBufferObject> vbo = new osg::VertexBufferObject();

    // Elements set ... later we'll decide whether to use the global one
    osg::DrawElements* primSet = NULL;

    // the initial vertex locations:
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
    if (_options.morphTerrain() == true)
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
    bool populateTexCoords = true;
    osg::ref_ptr<osg::Vec3Array> texCoords = new osg::Vec3Array();
    texCoords->setBinding(texCoords->BIND_PER_VERTEX);
    texCoords->setVertexBufferObject(vbo.get());
    texCoords->reserve( numVerts );
    geom->setTexCoordArray(texCoords.get());

    if (editor.hasEdits())
    {
        bool tileHasData = editor.createTileMesh(
            geom.get(),
            tileSize,
            _options.heightFieldSkirtRatio().get(),
            progress);

        if (tileHasData)
            geom->setHasConstraints(true);
        else
            return nullptr;
    }

    else // default mesh - no constraints
    {
        osg::Vec3d unit;
        osg::Vec3d model;
        osg::Vec3d modelLTP;
        osg::Vec3d modelPlusOne;
        osg::Vec3d normal;

        GeoLocator locator(tileKey.getExtent());

        for (unsigned row = 0; row < tileSize; ++row)
        {
            float ny = (float)row / (float)(tileSize - 1);
            for (unsigned col = 0; col < tileSize; ++col)
            {
                float nx = (float)col / (float)(tileSize - 1);

                unit.set(nx, ny, 0.0f);
                locator.unitToWorld(unit, model);
                modelLTP = model * world2local;
                verts->push_back(modelLTP);

                tileBound.expandBy(verts->back());

                if (populateTexCoords)
                {
                    // Use the Z coord as a type marker
                    float marker = VERTEX_VISIBLE;
                    texCoords->push_back(osg::Vec3f(nx, ny, marker));
                }

                unit.z() = 1.0f;
                locator.unitToWorld(unit, modelPlusOne);
                normal = (modelPlusOne*world2local) - modelLTP;
                normal.normalize();
                normals->push_back(normal);

                // neighbor:
                if (neighbors)
                {
                    const osg::Vec3& modelNeighborLTP = (*verts)[verts->size() - getMorphNeighborIndexOffset(col, row, tileSize)];
                    neighbors->push_back(modelNeighborLTP);
                }

                if (neighborNormals)
                {
                    const osg::Vec3& modelNeighborNormalLTP = (*normals)[normals->size() - getMorphNeighborIndexOffset(col, row, tileSize)];
                    neighborNormals->push_back(modelNeighborNormalLTP);
                }
            }
        }

        if (needsSkirt)
        {
            // calculate the skirt extrusion height
            double height = tileBound.radius() * _options.heightFieldSkirtRatio().get();

            // Normal tile skirt first:
            unsigned skirtIndex = verts->size();

            // first, create all the skirt verts, normals, and texcoords.
            for (int c = 0; c < (int)tileSize - 1; ++c)
                addSkirtDataForIndex(c, height); //top

            for (int r = 0; r < (int)tileSize - 1; ++r)
                addSkirtDataForIndex(r*tileSize + (tileSize - 1), height); //right

            for (int c = tileSize - 1; c >= 0; --c)
                addSkirtDataForIndex((tileSize - 1)*tileSize + c, height); //bottom

            for (int r = tileSize - 1; r >= 0; --r)
                addSkirtDataForIndex(r*tileSize, height); //left
        }

        // By default we tessellate the surface, but if there's a masking set
        // it might replace some or all of our surface geometry.
        bool tessellateSurface = true;

        if (tessellateSurface && primSet == nullptr)
        {
            primSet = _defaultPrimSet.get();
            //primSet = createPrimitiveSet(tileSize);
        }

        if (primSet)
        {
            geom->setDrawElements(primSet);
        }
    }

    return geom.release();
}

void
GeometryPool::setReleaser(ResourceReleaser* releaser)
{
    _releaser = releaser;
}

void
GeometryPool::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.UPDATE_VISITOR && _enabled)
    {
        Threading::ScopedMutexLock lock(_geometryMapMutex);

        std::vector<GeometryKey> keys;
        for (GeometryMap::iterator i = _geometryMap.begin(); i != _geometryMap.end(); ++i)
        {
            if (i->second.get()->referenceCount() == 1)
            {
                keys.push_back(i->first);
                i->second->releaseGLObjects(NULL);

                OE_DEBUG << "Releasing: " << i->second.get() << std::endl;
            }
        }
        for (std::vector<GeometryKey>::iterator key = keys.begin(); key != keys.end(); ++key)
        {
            _geometryMap.erase(*key);
        }
    }

    osg::Group::traverse(nv);
}

void
GeometryPool::clear()
{
    releaseGLObjects(NULL);
    Threading::ScopedMutexLock lock(_geometryMapMutex);
    _geometryMap.clear();
}

void
GeometryPool::resizeGLObjectBuffers(unsigned maxsize)
{
    if (!_enabled)
        return;

    // collect all objects in a thread safe manner
    Threading::ScopedMutexLock lock(_geometryMapMutex);
    {
        for (GeometryMap::const_iterator i = _geometryMap.begin(); i != _geometryMap.end(); ++i)
        {
            i->second->resizeGLObjectBuffers(maxsize);
        }
    }
}

void
GeometryPool::releaseGLObjects(osg::State* state) const
{
    if (!_enabled)
        return;

    ResourceReleaser::ObjectList objects;

    // collect all objects in a thread safe manner
    {
        Threading::ScopedMutexLock lock(_geometryMapMutex);
        {
            for (GeometryMap::const_iterator i = _geometryMap.begin(); i != _geometryMap.end(); ++i)
            {
                if (_releaser.valid())
                    objects.push_back(i->second.get());
                else
                    i->second->releaseGLObjects(state);
            }

            if (_releaser.valid() && !objects.empty())
            {
                OE_DEBUG << LC << "Released " << objects.size() << " objects in the geometry pool\n";
            }
        }
    }

    // submit to the releaser.
    if (_releaser.valid() && !objects.empty())
    {
        _releaser->push(objects);
    }
}

//.........................................................................
// Code mostly adapted from osgTerrain SharedGeometry.

SharedGeometry::SharedGeometry() :
    osg::Drawable()
{
    _supportsVertexBufferObjects = true;
    _ptype.resize(64u);
    _ptype.setAllElementsTo(GL_TRIANGLES);
    _hasConstraints = false;
    setSupportsDisplayList(false);
    setUseDisplayList(false);
    setUseVertexBufferObjects(true);
}

SharedGeometry::SharedGeometry(const SharedGeometry& rhs,const osg::CopyOp& copyop):
    osg::Drawable(rhs, copyop),
    _vertexArray(rhs._vertexArray),
    _normalArray(rhs._normalArray),
    _texcoordArray(rhs._texcoordArray),
    _neighborArray(rhs._neighborArray),
    _neighborNormalArray(rhs._neighborNormalArray),
    _drawElements(rhs._drawElements),
    _hasConstraints(rhs._hasConstraints)
{
    _ptype.resize(64u);
    _ptype.setAllElementsTo(GL_TRIANGLES);
}

SharedGeometry::~SharedGeometry()
{
    //nop
}

bool
SharedGeometry::empty() const
{
    return
        (_drawElements.valid() == false || _drawElements->getNumIndices() == 0); //&&
        //(_maskElements.valid() == false || _maskElements->getNumIndices() == 0);
}

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
}

void
SharedGeometry::compileGLObjects(osg::RenderInfo& renderInfo) const
{
    // Note: this method is not currently called. -gw
    osg::State& state = *renderInfo.getState();

    if (state.useVertexBufferObject(_supportsVertexBufferObjects && _useVertexBufferObjects))
    {
        // VBO:
        _vertexArray->getBufferObject()->getOrCreateGLBufferObject(renderInfo.getContextID())->compileBuffer();
        _drawElements->getBufferObject()->getOrCreateGLBufferObject(renderInfo.getContextID())->compileBuffer();

        // VAO:
        if (state.useVertexArrayObject(_useVertexArrayObject))
        {
            osg::VertexArrayState* vas = 0;

            _vertexArrayStateList[renderInfo.getContextID()] = vas = createVertexArrayState(renderInfo);

            osg::State::SetCurrentVertexArrayStateProxy setVASProxy(state, vas);

            state.bindVertexArrayObject(vas);

            drawVertexArraysImplementation(renderInfo);

            state.unbindVertexArrayObject();
        }

        // unbind the BufferObjects
        osg::GLExtensions* extensions = renderInfo.getState()->get<osg::GLExtensions>();
        extensions->glBindBuffer(GL_ARRAY_BUFFER_ARB, 0);
        extensions->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER_ARB, 0);
    }
    else
    {
        osg::Drawable::compileGLObjects(renderInfo);
    }
}

void
SharedGeometry::drawVertexArraysImplementation(osg::RenderInfo& renderInfo) const
{
    osg::State& state = *renderInfo.getState();
    osg::VertexArrayState* vas = state.getCurrentVertexArrayState();

    osg::AttributeDispatchers& attributeDispatchers = state.getAttributeDispatchers();

    attributeDispatchers.reset();
    attributeDispatchers.setUseVertexAttribAlias(state.getUseVertexAttributeAliasing());

    // activate or dispatch any attributes that are bound overall
    attributeDispatchers.activateNormalArray(_normalArray.get());
    attributeDispatchers.activateColorArray(_colorArray.get());

    if (state.useVertexArrayObject(_useVertexArrayObject))
    {
        if (!vas->getRequiresSetArrays()) return;
    }

    vas->lazyDisablingOfVertexAttributes();

    // set up arrays
    if (_vertexArray.valid())
        vas->setVertexArray(state, _vertexArray.get());

    if (_normalArray.valid() && _normalArray->getBinding() == osg::Array::BIND_PER_VERTEX)
        vas->setNormalArray(state, _normalArray.get());

    if (_colorArray.valid() && _colorArray->getBinding() == osg::Array::BIND_PER_VERTEX)
        vas->setColorArray(state, _colorArray.get());

    if (_texcoordArray.valid() && _texcoordArray->getBinding() == osg::Array::BIND_PER_VERTEX)
        vas->setTexCoordArray(state, 0, _texcoordArray.get());

    if (_neighborArray.valid() && _neighborArray->getBinding() == osg::Array::BIND_PER_VERTEX)
        vas->setTexCoordArray(state, 1, _neighborArray.get());

    if (_neighborNormalArray.valid() && _neighborNormalArray->getBinding() == osg::Array::BIND_PER_VERTEX)
        vas->setTexCoordArray(state, 2, _neighborNormalArray.get());

    vas->applyDisablingOfVertexAttributes(state);
}


void
SharedGeometry::drawPrimitivesImplementation(osg::RenderInfo& renderInfo) const
{
    if (_drawElements->getNumIndices() > 0u)
    {
        osg::State& state = *renderInfo.getState();
        osg::AttributeDispatchers& attributeDispatchers = state.getAttributeDispatchers();
        bool usingVertexBufferObjects = state.useVertexBufferObject(_supportsVertexBufferObjects && _useVertexBufferObjects);
        bool usingVertexArrayObjects = usingVertexBufferObjects && state.useVertexArrayObject(_useVertexArrayObject);

        GLenum primitiveType = _ptype[state.getContextID()];

        if (usingVertexArrayObjects || !usingVertexBufferObjects)
        {
            glDrawElements(
                primitiveType,
                _drawElements->getNumIndices(),
                _drawElements->getDataType(),
                _drawElements->getDataPointer());
        }
        else
        {
            osg::GLBufferObject* ebo = _drawElements->getOrCreateGLBufferObject(state.getContextID());

            if (ebo)
            {
                state.getCurrentVertexArrayState()->bindElementBufferObject(ebo);

                glDrawElements(
                    primitiveType,
                    _drawElements->getNumIndices(),
                    _drawElements->getDataType(),
                    (const GLvoid *)(ebo->getOffset(_drawElements->getBufferIndex())));
            }
            else
            {
                state.getCurrentVertexArrayState()->unbindElementBufferObject();

                glDrawElements(
                    primitiveType, 
                    _drawElements->getNumIndices(), 
                    _drawElements->getDataType(), 
                    _drawElements->getDataPointer());
            }
        }
    }
}

void SharedGeometry::drawImplementation(osg::RenderInfo& renderInfo) const
{
    osg::State& state = *renderInfo.getState();

    bool usingVertexBufferObjects = state.useVertexBufferObject(_supportsVertexBufferObjects && _useVertexBufferObjects);
    bool usingVertexArrayObjects = usingVertexBufferObjects && state.useVertexArrayObject(_useVertexArrayObject);

    osg::VertexArrayState* vas = state.getCurrentVertexArrayState();
    vas->setVertexBufferObjectSupported(usingVertexBufferObjects);

    bool checkForGLErrors = state.getCheckForGLErrors() == osg::State::ONCE_PER_ATTRIBUTE;
    if (checkForGLErrors) state.checkGLErrors("start of Geometry::drawImplementation()");

    drawVertexArraysImplementation(renderInfo);

    drawPrimitivesImplementation(renderInfo);

    if (usingVertexBufferObjects && !usingVertexArrayObjects)
    {
        // unbind the VBO's if any are used.
        vas->unbindVertexBufferObject();
        vas->unbindElementBufferObject();
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

    return geom;
}
