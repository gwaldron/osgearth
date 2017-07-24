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
#include <osgEarth/NodeUtils>
#include <osgEarthUtil/TopologyGraph>
#include <osg/Point>
#include <cstdlib> // for getenv

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Drivers::RexTerrainEngine;

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
                                osg::ref_ptr<SharedGeometry>& out,
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
    out.tileY = mapInfo.isGeocentric()? tileKey.getTileY() : 0; //.getExtent().yMin() : 0.0;
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
    
    // TODO: reconsider this ... 
    GLenum mode = (_options.gpuTessellation() == true) ? GL_PATCHES : GL_TRIANGLES;

    // Pre-allocate enough space for all triangles.
    osg::DrawElements* primSet = new osg::DrawElementsUShort(mode);
    primSet->setElementBufferObject(new osg::ElementBufferObject());

    primSet->reserveElements(numIndiciesInSurface + numIncidesInSkirt);

    osg::BoundingSphere tileBound;

    // the geometry:
    SharedGeometry* geom = new SharedGeometry();
    geom->setUseVertexBufferObjects(true);
    //geom->setUseDisplayList(false);

    osg::ref_ptr<osg::VertexBufferObject> vbo = new osg::VertexBufferObject();

    geom->setDrawElements(primSet);

    // the vertex locations:
    osg::Vec3Array* verts = new osg::Vec3Array();
    verts->setVertexBufferObject(vbo.get());
    verts->reserve( numVerts );
    verts->setBinding(verts->BIND_PER_VERTEX);
    geom->setVertexArray( verts );

    // the surface normals (i.e. extrusion vectors)
    osg::Vec3Array* normals = new osg::Vec3Array();
    normals->setVertexBufferObject(vbo.get());
    normals->reserve( numVerts );
    normals->setBinding(normals->BIND_PER_VERTEX);
    geom->setNormalArray( normals );
    
    osg::Vec3Array* neighbors = 0L;
    if ( _options.morphTerrain() == true )
    {
        // neighbor positions (for morphing)
        neighbors = new osg::Vec3Array();
        neighbors->setBinding(neighbors->BIND_PER_VERTEX);
        neighbors->setVertexBufferObject(vbo.get());
        neighbors->reserve( numVerts );
        geom->setNeighborArray(neighbors);
        //geom->setTexCoordArray( 1, neighbors );
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
    texCoords->setBinding(texCoords->BIND_PER_VERTEX);
    texCoords->setVertexBufferObject(vbo.get());
    texCoords->reserve( numVerts );
#endif

    geom->setTexCoordArray(texCoords);
    
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

    // create mask geometry
    bool skirtCreated = false;

    if (maskSet)
    {
        int s = verts->size();
        osg::ref_ptr<osg::DrawElementsUInt> maskPrim = maskSet->createMaskPrimitives(mapInfo, verts, texCoords, normals, neighbors);
        if (maskPrim)
        {
            maskPrim->setElementBufferObject(primSet->getElementBufferObject());
            geom->setMaskElements(maskPrim);

            if (createSkirt)
            {
                // Skirts for masking geometries are complicated. There are two parts.
                // The first part is the "perimeter" of the tile, i.e the outer edge of the 
                // tessellation. This code will detect that outer boundary and create skrits
                // for it.
                // The second part (NYI) detects the actual inner boundary ("patch geometry")
                // that patches the tile tessellation to the masking boundary. TDB.
                TopologyGraph topo;
                BuildTopologyVisitor visitor(topo);
                visitor.apply(geom, verts); 

                if (topo._verts.empty() == false)
                {
                    TopologyGraph::IndexVector boundary;
                    topo.createBoundary(boundary);
                
                    double height = tileBound.radius() * _options.heightFieldSkirtRatio().get();
                    unsigned skirtIndex = verts->size();

                    unsigned matches = 0;
                    for (TopologyGraph::IndexVector::const_iterator i = boundary.begin(); i != boundary.end(); ++i)
                    {
                        int k;
                        for (k = 0; k<skirtIndex; ++k)
                        {
                            if ((*verts)[k].x() == (*i)->x() && (*verts)[k].y() == (*i)->y())
                            {
                                addSkirtDataForIndex(k, height);
                                matches++;
                                break;
                            }
                        }
                    }

                    if (matches != boundary.size()) {
                        OE_WARN << LC << "matches != boundary size" << std::endl;
                    }

                    int n;
                    for (n = skirtIndex; n<(int)verts->size()-2; n+=2)
                        addMaskSkirtTriangles(n, n+2);

                    addMaskSkirtTriangles(n, skirtIndex);

                    skirtCreated = true;
                }
            }
        }
    }

    if ( createSkirt && !skirtCreated )
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
                    
                    //OE_INFO << "Releasing: " << i->second.get() << std::endl;
                }
            }
            for (std::vector<GeometryKey>::iterator key = keys.begin(); key != keys.end(); ++key)
            {
                if (_geometryMap[*key]->referenceCount() != 2) // one for the map, and one for the local objects list
                    OE_WARN << LC << "Erasing key geom with refcount <> 2" << std::endl;

                _geometryMap.erase(*key);
            }

            //OE_WARN << "Released " << keys.size() << ", pool = " << _geometryMap.size() << std::endl;
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
}

SharedGeometry::SharedGeometry(const SharedGeometry& rhs,const osg::CopyOp& copyop):
    osg::Drawable(rhs, copyop),
    _vertexArray(rhs._vertexArray),
    _normalArray(rhs._normalArray),
    _texcoordArray(rhs._texcoordArray),
    _neighborArray(rhs._neighborArray),
    _drawElements(rhs._drawElements),
    _maskElements(rhs._maskElements)
{
    //nop
}

SharedGeometry::~SharedGeometry()
{
    //nop
}

#ifdef SUPPORTS_VAO
osg::VertexArrayState* SharedGeometry::createVertexArrayState(osg::RenderInfo& renderInfo) const
{
    osg::State& state = *renderInfo.getState();

    osg::VertexArrayState* vas = new osg::VertexArrayState(&state);

    if (_vertexArray.valid()) vas->assignVertexArrayDispatcher();
    if (_normalArray.valid()) vas->assignNormalArrayDispatcher();
    unsigned texUnits = 0;
    if (_neighborArray.valid())
    {
        texUnits = 2;
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

void SharedGeometry::compileGLObjects(osg::RenderInfo& renderInfo) const
{
    if (!_vertexArray)
        return;

    if (_vertexArray->getVertexBufferObject())
    {
        osg::State& state = *renderInfo.getState();
        unsigned int contextID = state.getContextID();
        osg::GLExtensions* extensions = state.get<osg::GLExtensions>();
        if (!extensions) return;

        osg::BufferObject* vbo = _vertexArray->getVertexBufferObject();
        osg::GLBufferObject* vbo_glBufferObject = vbo->getOrCreateGLBufferObject(contextID);
        if (vbo_glBufferObject && vbo_glBufferObject->isDirty())
        {
            // OSG_NOTICE<<"Compile buffer "<<glBufferObject<<std::endl;
            vbo_glBufferObject->compileBuffer();
            extensions->glBindBuffer(GL_ARRAY_BUFFER_ARB,0);
        }

        osg::BufferObject* ebo = _drawElements->getElementBufferObject();
        osg::GLBufferObject* ebo_glBufferObject = ebo->getOrCreateGLBufferObject(contextID);
        if (ebo_glBufferObject && vbo_glBufferObject->isDirty())
        {
            // OSG_NOTICE<<"Compile buffer "<<glBufferObject<<std::endl;
            ebo_glBufferObject->compileBuffer();
            extensions->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER_ARB,0);
        }

#ifdef SUPPORTS_VAO
        if (state.useVertexArrayObject(_useVertexArrayObject))
        {
            osg::VertexArrayState* vas = 0;

            _vertexArrayStateList[contextID] = vas = createVertexArrayState(renderInfo);

            osg::State::SetCurrentVertexArrayStateProxy setVASProxy(state, vas);

#if OSG_MIN_VERSION_REQUIRED(3,5,6)
            state.bindVertexArrayObject(vas);
#else
            vas->bindVertexArrayObject();
#endif

            if (vbo_glBufferObject) vas->bindVertexBufferObject(vbo_glBufferObject);
            if (ebo_glBufferObject) vas->bindElementBufferObject(ebo_glBufferObject);
        }
#endif
    }
    else
    {
        Drawable::compileGLObjects(renderInfo);
    }
}

void SharedGeometry::resizeGLObjectBuffers(unsigned int maxSize)
{
    Drawable::resizeGLObjectBuffers(maxSize);

    osg::BufferObject* vbo = _vertexArray->getVertexBufferObject();
    if (vbo) vbo->resizeGLObjectBuffers(maxSize);

    osg::BufferObject* ebo = _drawElements->getElementBufferObject();
    if (ebo) ebo->resizeGLObjectBuffers(maxSize);
}

void SharedGeometry::releaseGLObjects(osg::State* state) const
{
    Drawable::releaseGLObjects(state);

    osg::BufferObject* vbo = _vertexArray->getVertexBufferObject();
    if (vbo) vbo->releaseGLObjects(state);

    osg::BufferObject* ebo = _drawElements->getElementBufferObject();
    if (ebo) ebo->releaseGLObjects(state);
}

// called from DrawTileCommand
void SharedGeometry::render(GLenum primitiveType, osg::RenderInfo& renderInfo) const
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
        // OSG_NOTICE<<"   sending vertex arrays vas->getRequiresSetArrays()="<<vas->getRequiresSetArrays()<<std::endl;

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

        state.applyDisablingOfVertexAttributes();
    }


#ifdef SUPPORTS_VAO
    bool request_bind_unbind = !state.useVertexArrayObject(_useVertexArrayObject) || state.getCurrentVertexArrayState()->getRequiresSetArrays();
#else
    bool request_bind_unbind = true;
#endif

    osg::GLBufferObject* ebo = _drawElements->getOrCreateGLBufferObject(state.getContextID());

    if (ebo)
    {
        /*if (request_bind_unbind)*/ state.bindElementBufferObject(ebo);

        glDrawElements(primitiveType, _drawElements->getNumIndices(), _drawElements->getDataType(), (const GLvoid *)(ebo->getOffset(_drawElements->getBufferIndex())));

        if (_maskElements.valid())
        {
            glDrawElements(primitiveType, _maskElements->getNumIndices(), _maskElements->getDataType(), (const GLvoid *)(ebo->getOffset(_maskElements->getBufferIndex())));
        }

        /*if (request_bind_unbind)*/ state.unbindElementBufferObject();
    }
    else
    {
        glDrawElements(primitiveType, _drawElements->getNumIndices(), _drawElements->getDataType(), _drawElements->getDataPointer());

        if (_maskElements.valid())
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
}

void SharedGeometry::accept(osg::Drawable::ConstAttributeFunctor& af) const
{
    osg::ConstAttributeFunctorArrayVisitor afav(af);

    afav.applyArray(VERTICES,_vertexArray.get());
    afav.applyArray(NORMALS, _normalArray.get());
    afav.applyArray(TEXTURE_COORDS_0,_texcoordArray.get());
    afav.applyArray(TEXTURE_COORDS_1,_neighborArray.get());
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

