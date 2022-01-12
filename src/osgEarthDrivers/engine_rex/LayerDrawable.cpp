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
#include "LayerDrawable"
#include "TerrainRenderData"
#include "SelectionInfo"
#include "EngineContext"
#include <osgEarth/Metrics>
#include <sstream>

using namespace osgEarth::REX;

#undef  LC
#define LC "[LayerDrawable] "

#define COPY_MAT4F(FROM,TO) ::memcpy((TO), (FROM).ptr(), 16*sizeof(float))


LayerDrawable::LayerDrawable() :
_renderType(Layer::RENDERTYPE_TERRAIN_SURFACE),
_drawOrder(0),
_layer(0L),
_visibleLayer(0L),
_imageLayer(0L),
_patchLayer(0L),
_clearOsgState(false),
_draw(true),
_useIndirectRendering(false)
{
    // Since we refresh the render state in the CULL traversal, we must
    // set the variance to dynamic to prevent overlap with DRAW
    setDataVariance(DYNAMIC);

    setUseDisplayList(false);
    setUseVertexBufferObjects(true);
    _tiles.reserve(128);

    // set up an arena with "auto release" which means th textures
    // will automatically get released when all references drop.
    // TODO: move this to the engine level and share??
    _textures = new TextureArena();
    _textures->setBindingPoint(29);
    _textures->setAutoRelease(true);
}

LayerDrawable::~LayerDrawable()
{
    // Drawable's DTOR will release GL objects on any attached stateset;
    // we don't want that because our Layer stateset is shared and re-usable.
    // So detach it before OSG has a chance to do so.
    setStateSet(nullptr);
}

void
LayerDrawable::accept(osg::PrimitiveFunctor& functor) const
{
    for (auto& tile : _tiles)
        tile.accept(functor);
}

void
LayerDrawable::accept(osg::PrimitiveIndexFunctor& functor) const
{
    for (auto& tile : _tiles)
        tile.accept(functor);
}

namespace
{
    // Hack State so we can dirty the texture attrs without dirtying the other 
    // attributes (as dirtyAllAttributes() would do).
    struct StateEx : public osg::State
    {
        void dirtyAllTextureAttributes()
        {
            for (auto& attr_map : _textureAttributeMapList)
            {
                for (auto& attr_entry : attr_map)
                {
                    attr_entry.second.last_applied_attribute = 0;
                    attr_entry.second.changed = true;
                }
            }
        }
    };
}

void
LayerDrawable::drawImplementation(osg::RenderInfo& ri) const
{
    const char* zone = _layer ? _layer->getName().c_str() : className();

    OE_PROFILING_ZONE;
    OE_PROFILING_ZONE_TEXT(zone);
    OE_GL_ZONE_NAMED(zone);

    if (_useIndirectRendering)
    {
        drawImplementationIndirect(ri);
    }
    else
    {
        drawImplementationDirect(ri);
    }

    // If set, dirty all OSG state to prevent any leakage - this is sometimes
    // necessary when doing custom OpenGL within a Drawable.
    if (_clearOsgState)
    {
        // Dirty the texture attributes so OSG can properly reset them
        // NOTE: cannot call state.dirtyAllAttributes, because that would invalidate
        // positional state like light sources!
        reinterpret_cast<StateEx*>(ri.getState())->dirtyAllTextureAttributes();
        
        // make sure any VAO is unbound before unbinind the VBO/EBOs,
        // as failing to do so will remove the VBO/EBO from the VAO
        if (ri.getState()->useVertexArrayObject(_useVertexArrayObject))
        {
            ri.getState()->unbindVertexArrayObject();
        }

        // unbind local buffers when finished.
        // Not necessary if using VAOs?
        osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();
        ext->glBindBuffer(GL_ARRAY_BUFFER_ARB,0);
        ext->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER_ARB,0);

        // gw: no need to do this, in fact it will cause positional attributes
        // (light clip planes and lights) to immediately be reapplied under the
        // current MVM, which will by definition be wrong!)
        //ri.getState()->apply();
    }
}

void
LayerDrawable::drawImplementationDirect(osg::RenderInfo& ri) const
{
    if (_patchLayer && _patchLayer->getRenderer())
    {
        TileBatch batch(_drawState.get());
        batch._tiles.reserve(_tiles.size());
        for (auto& tile : _tiles)
            batch._tiles.push_back(&tile);

        _patchLayer->getRenderer()->draw(ri, batch);
    }
    else
    {
        ProgramState& pps = _drawState->getProgramState(ri);

        if (pps._layerUidUL >= 0)
        {
            osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();
            GLint uid = _layer ? (GLint)_layer->getUID() : (GLint)-1;
            ext->glUniform1i(pps._layerUidUL, uid);
        }

        for (auto& tile : _tiles)
        {
            tile.apply(ri, _drawState.get());
            tile.draw(ri);
        }
    }
}

namespace
{
    // template to append one array onto another
    template<typename T> inline void append(T* dest, const T* src)
    {
        const T* src_typed = static_cast<const T*>(src);
        for (unsigned i = 0; i < src->getNumElements(); ++i)
            dest->push_back((*src)[i]);
    }

    template<typename T> inline void insert(T* dest, int index, const T* src)
    {
        const T* src_typed = static_cast<const T*>(src);
        for (unsigned i = 0; i < src->getNumElements(); ++i)
        {
            (*dest)[index + i] = (*src)[i];
        }            
    }

    template<typename T> inline void copy(T* dest, const osg::Array* src, unsigned offset)
    {
        const T* src_typed = static_cast<const T*>(src);
        std::copy(src_typed->begin(), src_typed->end(), dest->begin() + offset);
    }
}

void
LayerDrawable::accept(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.CULL_VISITOR &&
        _useIndirectRendering)
    {
        refreshRenderState();
    }

    osg::Drawable::accept(nv);
}

void LayerDrawable::RenderState::grow(unsigned int size)
{
    verts->resize(verts->size() + size);
    verts->dirty();
    uvs->resize(uvs->size() + size);
    uvs->dirty();
    upvectors->resize(upvectors->size() + size);
    upvectors->dirty();
    neighbors->resize(neighbors->size() + size);
    neighbors->dirty();
    neighborupvectors->resize(neighborupvectors->size() + size);
    neighborupvectors->dirty();
}

const LayerDrawable::Block& LayerDrawable::RenderState::requestBlock(unsigned int size)
{

#if 0
    // Always dirty
    verts->dirty();
    uvs->dirty();
    upvectors->dirty();
    neighbors->dirty();
    neighborupvectors->dirty();
#endif

    for (unsigned int i = 0; i < blocks.size(); ++i)
    {
        if (blocks[i].isFree && blocks[i].count >= size)
        {
            blocks[i].isFree = false;
            dirtyRanges.emplace_back(blocks[i].index, size);
            return blocks[i];
        }
    }

    // We couldn't find a block so we need to resize all of the arrays and append a block at the end.
    blocks.emplace_back(verts->size(), size);
    grow(size);

    _dirtyVBO = true;

    return blocks.back();
}

void
LayerDrawable::refreshRenderState()
{
    OE_PROFILING_ZONE;

    // This is THREAD SAFE because the LayerDrawable object itself
    // exists on a per-camera basis. Therefore we do not need to
    // worry about statefulness

    // Do we need DEI commands? Not for patch layers.
    bool refreshGeometry = (_patchLayer == nullptr);

    if (_tiles != _rs._tiles)
    {
        if (refreshGeometry)
        {
            _rs._dirtyVBO = false;

            unsigned int numExpired = false;

            // Find all the tiles that were in the previous renderstate but not in this one
            for (auto& prevTile : _rs._tiles)
            {
                auto id = prevTile._geom->uid();

                bool found = false;
                for (auto& t : _tiles)
                {
                    if (t._geom->uid() == id)
                    {
                        found = true;
                        break;
                    }
                }
                if (!found)
                {                    
                    auto itr = _rs.baseVertexLUT.find(id);
                    if (itr != _rs.baseVertexLUT.end())
                    {
                        //std::cout << "Expired tile " << id << " base vertex " << itr->second << " size=" << prevTile._geom.get()->getVertexArray()->size() << std::endl;
                        for (auto& b: _rs.blocks)
                        {
                            if (b.index == itr->second)
                            {
                                b.isFree = true;
                                _rs.baseVertexLUT.erase(itr);
                                break;
                            }
                        }
                    }                    

                    ++numExpired;
                }
            }
            //std::cout << "Old tiles " << _rs._tiles.size() << " New tiles " << _tiles.size() << " Expired " << numExpired << std::endl;

            unsigned int numCopied = 0;

            osg::DefaultIndirectCommandDrawElements& commands =
                static_cast<osg::DefaultIndirectCommandDrawElements&>(*_rs.elements->getIndirectCommandArray());

            commands.resizeElements(_tiles.size());
            commands.dirty();

            _rs.elements->setNumCommandsToDraw(_tiles.size());

            unsigned cmd = 0;
            bool vboDirty = false;
            bool eboDirty = false;

            /*
            const bool reset = false;            
            if (reset)
            {
                _rs.verts->clear();
                _rs.uvs->clear();
                _rs.upvectors->clear();
                _rs.neighbors->clear();
                _rs.neighborupvectors->clear();
                _rs.elements->clear();
                _rs.baseVertexLUT.clear();
                _rs.firstIndexLUT.clear();
            }
            */

            for (auto& tile : _tiles)
            {
                SharedGeometry* geom = tile._geom.get();

                // Try to find the existing array
                auto baseVertexItr = _rs.baseVertexLUT.find(geom->uid());
                if (baseVertexItr == _rs.baseVertexLUT.end())
                {
                    auto block = _rs.requestBlock(geom->getVertexArray()->size());
                    _rs.baseVertexLUT[geom->uid()] = block.index;
                    copy(_rs.verts, geom->getVertexArray(), block.index);                    
                    copy(_rs.uvs, geom->getTexCoordArray(), block.index );
                    copy(_rs.upvectors, geom->getNormalArray(), block.index );
                    copy(_rs.neighbors, geom->getNeighborArray(), block.index );
                    copy(_rs.neighborupvectors, geom->getNeighborNormalArray(), block.index );

                    //insert(_rs.verts, block.index, geom->getVertexArray());
                    //insert(_rs.uvs, block.index, geom->getTexCoordArray());
                    //insert(_rs.upvectors, block.index, geom->getNormalArray());
                    //insert(_rs.neighbors, block.index, geom->getNeighborArray());
                    //insert(_rs.neighborupvectors, block.index, geom->getNeighborNormalArray());
                    commands[cmd].baseVertex = block.index;
                }
                else
                {
                    commands[cmd].baseVertex = baseVertexItr->second;
                }

                

                /*
                // TODO:  Figure out the right block to insert it and not at the end
                auto r0 = _rs.baseVertexLUT.emplace(geom->uid(), _rs.verts->size());
                commands[cmd].baseVertex = r0.first->second;
                if (r0.second) // new array
                {
                    vboDirty = true;
                    numCopied += geom->getVertexArray()->size();
                    append(_rs.verts, geom->getVertexArray());
                    append(_rs.uvs, geom->getTexCoordArray());
                    append(_rs.upvectors, geom->getNormalArray());
                    append(_rs.neighbors, geom->getNeighborArray());
                    append(_rs.neighborupvectors, geom->getNeighborNormalArray());
                }
                */

                // Does the draw elements need a memory manager too?
                SharedDrawElements* de = geom->getDrawElements();
                auto r1 = _rs.firstIndexLUT.emplace(de->uid(), _rs.elements->size());

                commands[cmd].firstIndex = r1.first->second;
                unsigned numIndices = de->getNumIndices();
                if (r1.second) // new elements
                {
                    eboDirty = true;
                    for (unsigned i = 0; i < numIndices; ++i)
                        _rs.elements->push_back(de->getElement(i));
                }
                commands[cmd].count = numIndices;
                commands[cmd].baseInstance = 0;
                commands[cmd].instanceCount = 1;

                ++cmd;
            }

            /*
            if (vboDirty)
            {
                _rs.verts->dirty();
                _rs.uvs->dirty();
                _rs.upvectors->dirty();
                _rs.neighbors->dirty();
                _rs.neighborupvectors->dirty();
            }
            */

            //std::cout << "Copied " << numCopied << " elements" << std::endl;

            if (eboDirty)
            {
                _rs.elements->dirty();
            }

            OE_PROFILING_PLOT("Terrain VBO size", float(_rs.verts->size() * sizeof(osg::Vec3f) * 5));
        }


        // Next assemble the TileBuffer structures
        if (_rs.tilebuf.size() < _tiles.size())
        {
            _rs.tilebuf.resize(_tiles.size());
        }

        unsigned tile_num = 0;
        for (auto& tile : _tiles)
        {
            GL4Tile& buf = _rs.tilebuf[tile_num++];

            // main MVM (double to float is OK)
            for (int i = 0; i < 16; ++i)
                buf.modelViewMatrix[i] = tile._modelViewMatrix->ptr()[i];

            // Tile key encoding
            for (int i = 0; i < 4; ++i)
                buf.tileKey[i] = tile._keyValue[i];

            // Color sampler and matrix:
            buf.colorIndex = -1;
            buf.parentIndex = -1;
            if (tile._colorSamplers != nullptr)
            {
                const Sampler& color = (*tile._colorSamplers)[SamplerBinding::COLOR];
                if (color._arena_texture != nullptr)
                {
                    buf.colorIndex = _textures->add(color._arena_texture);
                    COPY_MAT4F(color._matrix, buf.colorMat);
                }

                const Sampler& parent = (*tile._colorSamplers)[SamplerBinding::COLOR_PARENT];
                if (parent._arena_texture != nullptr)
                {
                    buf.parentIndex = _textures->add(parent._arena_texture);
                    COPY_MAT4F(parent._matrix, buf.parentMat);
                }
            }

            // Elevation sampler:
            buf.elevIndex = -1;
            if (tile._sharedSamplers != nullptr /* && is elevation active */)
            {
                const Sampler& s = (*tile._sharedSamplers)[SamplerBinding::ELEVATION];
                if (s._arena_texture)
                {
                    s._arena_texture->_compress = false;
                    s._arena_texture->_mipmap = false;
                    s._arena_texture->_internalFormat = GL_R32F;
                    s._arena_texture->_maxAnisotropy = 1.0f;
                    buf.elevIndex = _textures->add(s._arena_texture);
                    COPY_MAT4F(s._matrix, buf.elevMat);
                }
            }

            // Normal sampler:
            buf.normalIndex = -1;
            if (tile._sharedSamplers != nullptr /* && is normalmapping active */)
            {
                const Sampler& s = (*tile._sharedSamplers)[SamplerBinding::NORMAL];
                if (s._arena_texture)
                {
                    s._arena_texture->_compress = false;
                    s._arena_texture->_mipmap = true;
                    s._arena_texture->_maxAnisotropy = 1.0f;
                    buf.normalIndex = _textures->add(s._arena_texture);
                    COPY_MAT4F(s._matrix, buf.normalMat);
                }
            }

            // Other shared samplers.
            if (tile._sharedSamplers != nullptr)
            {
                for (unsigned i = SamplerBinding::SHARED; i < tile._sharedSamplers->size(); ++i)
                {
                    const Sampler& s = (*tile._sharedSamplers)[i];
                    if (s._arena_texture)
                    {
                        int k = i - SamplerBinding::SHARED;
                        if (k < MAX_NUM_SHARED_SAMPLERS)
                        {
                            s._arena_texture->_compress = false;
                            s._arena_texture->_mipmap = true;
                            //s._arena_texture->_maxAnisotropy = 4.0f;
                            buf.sharedIndex[k] = _textures->add(s._arena_texture);
                            COPY_MAT4F(s._matrix, buf.sharedMat[k]);
                        }
                        else
                        {
                            OE_WARN << LC << "Exceeded number of shared samplers" << std::endl;
                        }
                    }
                }
            }

            // stuck the layer order here (for now...later, hide it elsewhere)
            buf.drawOrder = _drawOrder;
        }

        _rs._tiles.swap(_tiles);

        // This will trigger a GPU upload on the next draw
        _rs._dirty = true;
    }
}

LayerDrawable::RenderState::RenderState()
{
    gcState.resize(64);

    geom = new osg::Geometry();
    geom->setUseVertexBufferObjects(true);
    geom->setUseDisplayList(false);

    // configure the GL4 geometry
    verts = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    verts->setBufferObject(new osg::VertexBufferObject());

    uvs = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    uvs->setBufferObject(new osg::VertexBufferObject());

    upvectors = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    upvectors->setBufferObject(new osg::VertexBufferObject());

    neighbors = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    neighbors->setBufferObject(new osg::VertexBufferObject());

    neighborupvectors = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    neighborupvectors->setBufferObject(new osg::VertexBufferObject());

    elements = new osg::MultiDrawElementsIndirectUShort(GL_TRIANGLES);
    
    geom->setVertexArray(verts);
    geom->setNormalArray(upvectors);
    geom->setTexCoordArray(0, uvs);
    geom->setTexCoordArray(1, neighbors);
    geom->setTexCoordArray(2, neighborupvectors);
    // todo: neighbors and neighbor normals
    geom->addPrimitiveSet(elements);

    // TODO: This will force the VAO to update every time..do we want that?
    geom->setDataVariance(osg::Object::DYNAMIC);
}

void
LayerDrawable::drawImplementationIndirect(osg::RenderInfo& ri) const
{
    GCState& gs = _rs.gcState[ri.getContextID()];
    osg::State& state = *ri.getState();
    osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();

    if (_rs._tiles.empty())
        return;

    if (gs.tiles == nullptr || !gs.tiles->valid())
    {
        gs.tiles = GLBuffer::create(
            GL_SHADER_STORAGE_BUFFER,
            state,
            "LayerDrawable Tiles");
    }

    // First time through any layer? Make the shared buffer
    // TODO: this might eventually go to the terrain level.
    if (gs.shared == nullptr || !gs.shared->valid())
    {
        GL4GlobalData buf;

        // Encode morphing constants, one per LOD
        const SelectionInfo& info = _context->getSelectionInfo();
        for (unsigned lod = 0; lod < 19; ++lod)
        {
            float end = info.getLOD(lod)._morphEnd;
            float start = info.getLOD(lod)._morphStart;
            float one_over_end_minus_start = 1.0f / (end - start);
            buf.morphConstants[(2 * lod) + 0] = end * one_over_end_minus_start;
            buf.morphConstants[(2 * lod) + 1] = one_over_end_minus_start;
        }

        gs.shared = GLBuffer::create(
            GL_SHADER_STORAGE_BUFFER,
            state,
            "LayerDrawable Global");

        gs.shared->bind();
        gs.shared->bufferStorage((GLsizei)sizeof(GL4GlobalData), &buf, 0); // permanent
    }

    if (_rs._dirty)
    {
        _rs._dirty = false;

        // Update the tile render buffer:
        gs.tiles->bind();
        gs.tiles->uploadData(sizeof(GL4Tile) * _rs._tiles.size(), _rs.tilebuf.data());
    }

    // Bind the layout indices so we can access our tile data from the shader:
    gs.shared->bindBufferBase(30);
    gs.tiles->bindBufferBase(31);

    // Apply the the texture arena:
    if (state.getLastAppliedAttribute(OE_TEXTURE_ARENA_SA_TYPE_ID) != _textures)
    {
        _textures->apply(state);
        state.haveAppliedAttribute(_textures.get());
    }

    if (_patchLayer)
    {
        if (_patchLayer->getRenderer())
        {
            // If it's a patch layer, the layer does its own rendering
            // TODO: pass along the fact that we're using GL4 so that
            // the patch layer doesn't actually APPLY each DrawTileCommand!
            TileBatch batch(_drawState.get());
            batch._tiles.reserve(_tiles.size());
            for (auto& tile : _tiles)
                batch._tiles.push_back(&tile);

            _patchLayer->getRenderer()->draw(ri, batch);
        }
    }
    else
    {

        if (!_rs._dirtyVBO)
        {
#if 1
            // Needs to be single threaded without a mutex....
            for (auto& dirtyRange : _rs.dirtyRanges)
            {
                
                {
                    auto vbo = _rs.verts->getVertexBufferObject();
                    auto bufferObject = vbo->getGLBufferObject(ri.getContextID());
                    ext->glBindBuffer(GL_ARRAY_BUFFER, bufferObject->getGLObjectID());                    
                    ext->glBufferSubData(vbo->getProfile()._target, dirtyRange.index * _rs.verts->getElementSize(), _rs.verts->getElementSize() * dirtyRange.count, _rs.verts->getDataPointer(dirtyRange.index));
                }

                {
                    auto vbo = _rs.uvs->getVertexBufferObject();
                    auto bufferObject = vbo->getGLBufferObject(ri.getContextID());
                    ext->glBindBuffer(GL_ARRAY_BUFFER, bufferObject->getGLObjectID());
                    ext->glBufferSubData(vbo->getProfile()._target, dirtyRange.index * _rs.uvs->getElementSize(), _rs.uvs->getElementSize() * dirtyRange.count, _rs.uvs->getDataPointer(dirtyRange.index));
                }

                {
                    auto vbo = _rs.upvectors->getVertexBufferObject();
                    auto bufferObject = vbo->getGLBufferObject(ri.getContextID());
                    ext->glBindBuffer(GL_ARRAY_BUFFER, bufferObject->getGLObjectID());
                    ext->glBufferSubData(vbo->getProfile()._target, dirtyRange.index * _rs.upvectors->getElementSize(), _rs.upvectors->getElementSize() * dirtyRange.count, _rs.upvectors->getDataPointer(dirtyRange.index));
                }

                {
                    auto vbo = _rs.neighbors->getVertexBufferObject();
                    auto bufferObject = vbo->getGLBufferObject(ri.getContextID());
                    ext->glBindBuffer(GL_ARRAY_BUFFER, bufferObject->getGLObjectID());
                    ext->glBufferSubData(vbo->getProfile()._target, dirtyRange.index * _rs.neighbors->getElementSize(), _rs.neighbors->getElementSize() * dirtyRange.count, _rs.neighbors->getDataPointer(dirtyRange.index));
                }

                {
                    auto vbo = _rs.neighborupvectors->getVertexBufferObject();
                    auto bufferObject = vbo->getGLBufferObject(ri.getContextID());
                    ext->glBindBuffer(GL_ARRAY_BUFFER, bufferObject->getGLObjectID());
                    ext->glBufferSubData(vbo->getProfile()._target, dirtyRange.index * _rs.neighborupvectors->getElementSize(), _rs.neighborupvectors->getElementSize() * dirtyRange.count, _rs.neighborupvectors->getDataPointer(dirtyRange.index));
                }
            }
            _rs.dirtyRanges.clear();
#endif
        }



        // we don't care about the MVM but we do need to projection matrix:
        if (ri.getState()->getUseModelViewAndProjectionUniforms())
            ri.getState()->applyModelViewAndProjectionUniformsIfRequired();

        _rs.geom->draw(ri);
    }
}

void
LayerDrawable::releaseGLObjects(osg::State* state) const
{
    RenderState& cs = _rs;
    if (state)
    {
        GCState& gs = cs.gcState[state->getContextID()];
        gs.shared = nullptr;
        gs.tiles = nullptr;
        if (cs.geom.valid())
            cs.geom->releaseGLObjects(state);
    }
    else
    {
        cs.gcState.setAllElementsTo(GCState());
        if (cs.geom.valid())
            cs.geom->releaseGLObjects(nullptr);
    }

    if (_textures.valid())
        _textures->releaseGLObjects(state);

    osg::Drawable::releaseGLObjects(state);
}

void
LayerDrawable::resizeGLObjectBuffers(unsigned size)
{
    if (_rs.gcState.size() < size)
        _rs.gcState.resize(size);

    if (_rs.geom.valid())
        _rs.geom->resizeGLObjectBuffers(size);

    if (_textures.valid())
        _textures->resizeGLObjectBuffers(size);

    osg::Drawable::resizeGLObjectBuffers(size);
}
