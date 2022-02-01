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
#include <cstddef>

using namespace osgEarth::REX;

#undef  LC
#define LC "[LayerDrawable] "

#define COPY_MAT4F(FROM,TO) ::memcpy((TO), (FROM).ptr(), 16*sizeof(float))

#ifndef GL_VERTEX_ATTRIB_ARRAY_UNIFIED_NV
#define GL_VERTEX_ATTRIB_ARRAY_UNIFIED_NV 0x8F1E
#define GL_ELEMENT_ARRAY_UNIFIED_NV 0x8F1F
#endif

typedef void (APIENTRYP PFNGLENABLECLIENTSTATEPROC) (GLenum cap);


LayerDrawable::LayerDrawable() :
_renderType(Layer::RENDERTYPE_TERRAIN_SURFACE),
_drawOrder(0),
_surfaceDrawOrder(0),
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
    //TODO: Check this.
    setDataVariance(DYNAMIC);

    setUseDisplayList(false);
    setUseVertexBufferObjects(true);
    _tiles.reserve(128);

    // set up an arena with "auto release" which means th textures
    // will automatically get released when all references drop.
    // TODO: move this to the engine level and share??
    //_textures = new TextureArena();
    //_textures->setBindingPoint(29);
    //_textures->setAutoRelease(true);
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

void
LayerDrawable::refreshRenderState()
{
    OE_PROFILING_ZONE;

    // NOTE: LayerDrawable exists on a per-camera basis
    // ...BUT in a multithreading mode, we will probably need
    //    to double-buffer some things. TODO.

    if (_tiles != _rs.tiles)
    {
        // Next assemble the TileBuffer structures
        if (_rs.tilebuf.size() < _tiles.size())
        {
            _rs.tilebuf.resize(_tiles.size());
        }

        TextureArena* textures = _context->textures();

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
                    buf.colorIndex = textures->add(color._arena_texture);
                    COPY_MAT4F(color._matrix, buf.colorMat);
                }

                const Sampler& parent = (*tile._colorSamplers)[SamplerBinding::COLOR_PARENT];
                if (parent._arena_texture != nullptr)
                {
                    buf.parentIndex = textures->add(parent._arena_texture);
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
                    buf.elevIndex = textures->add(s._arena_texture);
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
                    buf.normalIndex = textures->add(s._arena_texture);
                    COPY_MAT4F(s._matrix, buf.normalMat);
                }
            }

            // LandCover sampler:
            if (_context->options().useLandCover() == true)
            {
                buf.landcoverIndex = -1;
                if (tile._sharedSamplers != nullptr /* && is normalmapping active */)
                {
                    const Sampler& s = (*tile._sharedSamplers)[SamplerBinding::LANDCOVER];
                    if (s._arena_texture)
                    {
                        s._arena_texture->_compress = false;
                        s._arena_texture->_mipmap = false;
                        s._arena_texture->_maxAnisotropy = 1.0f;
                        buf.landcoverIndex = textures->add(s._arena_texture);
                        COPY_MAT4F(s._matrix, buf.landcoverMat);
                    }
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
                            buf.sharedIndex[k] = textures->add(s._arena_texture);
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
            buf.drawOrder = _surfaceDrawOrder;
        }

        _rs.tiles = std::move(_tiles);

        // This will trigger a GPU upload on the next draw
        _rs.dirty = true;
    }
}

LayerDrawable::GL4RenderState::GL4RenderState()
{
    gcState.resize(64);
}

void
LayerDrawable::drawImplementationIndirect(osg::RenderInfo& ri) const
{
    // Research on glMultiDrawElementsIndirectBindlessNV:
    // https://github.com/ychding11/HelloWorld/wiki/Modern-GPU-Driven-Rendering--%28How-to-draw-fast%29
    // https://on-demand.gputechconf.com/siggraph/2014/presentation/SG4117-OpenGL-Scene-Rendering-Techniques.pdf
    // https://developer.download.nvidia.com/opengl/tutorials/bindless_graphics.pdf

    GCState& gs = _rs.gcState[ri.getContextID()];
    osg::State& state = *ri.getState();

    if (_rs.tiles.empty())
        return;

    bool renderTerrainSurface = (_patchLayer == nullptr);


    if (gs.tiles == nullptr || !gs.tiles->valid())
    {
        gs.ext = osg::GLExtensions::Get(state.getContextID(), true);

        gs.tiles = GLBuffer::create(
            GL_SHADER_STORAGE_BUFFER,
            state,
            "REX Renderer");

        // preallocate space for a bunch of tiles (just for fun)
        gs.tiles->bufferData(
            512 * sizeof(GL4Tile),
            nullptr,
            GL_DYNAMIC_DRAW);
    }

    if (renderTerrainSurface)
    {
        if (gs.commands == nullptr || !gs.commands->valid())
        {
            gs.commands = GLBuffer::create(
                GL_DRAW_INDIRECT_BUFFER,
                state,
                "REX Renderer");

            // preallocate space for a bunch of draw commands (just for fun)
            gs.commands->bufferData(
                512 * sizeof(DrawElementsIndirectBindlessCommandNV),
                nullptr,
                GL_DYNAMIC_DRAW);

            osg::setGLExtensionFuncPtr(
                gs.glMultiDrawElementsIndirectBindlessNV,
                "glMultiDrawElementsIndirectBindlessNV");
            OE_HARD_ASSERT(gs.glMultiDrawElementsIndirectBindlessNV != nullptr);

            // OSG bug: glVertexAttribFormat is mapped to the wrong function :( so
            // we have to look it up fresh.
            osg::setGLExtensionFuncPtr(
                gs.glVertexAttribFormat,
                "glVertexAttribFormat");
            OE_HARD_ASSERT(gs.glVertexAttribFormat != nullptr);

            // Core profile doesn't support glEnableClientState(), but if GL_ELEMENT_ARRAY_UNIFIED_NV
            // is supported then glEnableClientState() will exist.
            osg::setGLExtensionFuncPtr(
                gs.glEnableClientState,
                "glEnableClientState");

            // Set up a VAO that we'll use to render with bindless NV.
            gs.vao = GLVAO::create(state, "REX Renderer");

            // Start recording
            gs.vao->bind();

            // set up the VAO for NVIDIA bindless buffers
            if (gs.glEnableClientState)
            {
                gs.glEnableClientState(GL_VERTEX_ATTRIB_ARRAY_UNIFIED_NV);
                gs.glEnableClientState(GL_ELEMENT_ARRAY_UNIFIED_NV);
            }

            // Record the format for each of the attributes in GL4Vertex
            const GLuint offsets[5] = {
                offsetof(GL4Vertex, position),
                offsetof(GL4Vertex, normal),
                offsetof(GL4Vertex, uv),
                offsetof(GL4Vertex, neighborPosition),
                offsetof(GL4Vertex, neighborNormal)
            };
            for (unsigned location = 0; location < 5; ++location)
            {
                gs.glVertexAttribFormat(location, 3, GL_FLOAT, GL_FALSE, offsets[location]);
                gs.ext->glVertexAttribBinding(location, 0);
                gs.ext->glEnableVertexAttribArray(location);
            }

            // bind a "dummy buffer" that will record the stride, which is
            // just the size of our vertex structure.
            gs.ext->glBindVertexBuffer(0, 0, 0, sizeof(GL4Vertex));

            // Finish recording
            gs.vao->unbind();
        }

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
                "REX Renderer");

            gs.shared->bind();
            gs.shared->bufferStorage(sizeof(GL4GlobalData), &buf, 0); // permanent
        }
    }

    if (_rs.dirty)
    {
        // The CULL traversal determined that the tile set changed,
        // so we need to re-upload the tile buffer and we need to
        // rebuild the command list.

        _rs.dirty = false;

        // TODO: implement double/triple buffering so OSG multi-threading modes
        // will not overlap and corrupt the buffers

        // Update the tile data buffer:
        gs.tiles->uploadData(
            _rs.tiles.size() * sizeof(GL4Tile),
            _rs.tilebuf.data());

        if (renderTerrainSurface)
        {
            // Reconstruct and upload the command list:
            _rs.commands.clear();
            for (auto& tile : _rs.tiles)
            {
                SharedGeometry* geom = tile._geom.get();
                _rs.commands.push_back(geom->getOrCreateCommand(state));
            }

            gs.commands->uploadData(_rs.commands);
        }
    }

    // Apply the the texture arena:
    if (state.getLastAppliedAttribute(OE_TEXTURE_ARENA_SA_TYPE_ID) != _context->textures())
    {
        _context->textures()->apply(state);
        state.haveAppliedAttribute(_context->textures());
    }

    // Bind the tiles data to its layout(binding=X) in the shader.
    gs.tiles->bindBufferBase(31);


    if (renderTerrainSurface)
    {
        // Bind the command buffer for rendering.
        gs.commands->bind();

        // Bind the shared data to its layout(binding=X) in the shader.
        // For shared data we only need to do this once per pass
        if (_surfaceDrawOrder == 0)
        {
            gs.shared->bindBufferBase(30);
        }

        gs.vao->bind();

        gs.glMultiDrawElementsIndirectBindlessNV(
            GL_TRIANGLES,
            GL_UNSIGNED_SHORT,
            nullptr,
            _rs.commands.size(),
            sizeof(DrawElementsIndirectBindlessCommandNV),
            1);

        gs.vao->unbind();
    }

    else if (_patchLayer && _patchLayer->getRenderer())
    {
        // If it's a patch layer, the layer does its own rendering
        // TODO: pass along the fact that we're using GL4 so that
        // the patch layer doesn't actually APPLY each DrawTileCommand!
        TileBatch batch(_drawState.get());
        batch._tiles.reserve(_rs.tiles.size());
        for (auto& tile : _rs.tiles)
            batch._tiles.push_back(&tile);

        _patchLayer->getRenderer()->draw(ri, batch);
    }
}

void
LayerDrawable::releaseGLObjects(osg::State* state) const
{
    GL4RenderState& cs = _rs;
    if (state)
    {
        GCState& gs = cs.gcState[state->getContextID()];
        gs.shared = nullptr;
        gs.tiles = nullptr;
        gs.commands = nullptr;
        gs.vao = nullptr;
    }
    else
    {
        cs.gcState.setAllElementsTo(GCState());
    }

    osg::Drawable::releaseGLObjects(state);
}

void
LayerDrawable::resizeGLObjectBuffers(unsigned size)
{
    if (_rs.gcState.size() < size)
        _rs.gcState.resize(size);

    osg::Drawable::resizeGLObjectBuffers(size);
}
