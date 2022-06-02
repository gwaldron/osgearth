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



LayerDrawable::LayerDrawable() :
_renderType(Layer::RENDERTYPE_TERRAIN_SURFACE),
_drawOrder(0),
_surfaceDrawOrder(0),
_layer(0L),
_visibleLayer(0L),
_imageLayer(0L),
_patchLayer(0L),
_clearOsgState(false),
_draw(true)
{
    // Since we refresh the render state in the CULL traversal, we must
    // set the variance to dynamic to prevent overlap with DRAW
    //TODO: Check this.
    setDataVariance(DYNAMIC);
    setUseDisplayList(false);
    setUseVertexBufferObjects(true);
    _tiles.reserve(128);
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

//.........................................................

LayerDrawableGL3::LayerDrawableGL3() :
    LayerDrawable()
{
    setName("LayerDrawableGL3");
}

LayerDrawableGL3::~LayerDrawableGL3()
{
    //nop
}

void
LayerDrawableGL3::drawImplementation(osg::RenderInfo& ri) const
{
    const char* zone = _layer ? _layer->getName().c_str() : className();
    OE_GL_ZONE_NAMED(zone);

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
            if (tile.apply(ri, _drawState.get()))
                tile.draw(ri);
        }
    }

    LayerDrawable::drawImplementation(ri);
}

//.........................................................

LayerDrawableNVGL::LayerDrawableNVGL() :
    LayerDrawable()
{
    setName("LayerDrawableNVGL");
}
LayerDrawableNVGL::~LayerDrawableNVGL()
{
    //nop
}

void
LayerDrawableNVGL::accept(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        refreshRenderState();
    }

    osg::Drawable::accept(nv);
}

void
LayerDrawableNVGL::refreshRenderState()
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
                if (color._texture != nullptr)
                {
                    buf.colorIndex = textures->add(color._texture);
                    COPY_MAT4F(color._matrix, buf.colorMat);
                }

                const Sampler& parent = (*tile._colorSamplers)[SamplerBinding::COLOR_PARENT];
                if (parent._texture != nullptr)
                {
                    buf.parentIndex = textures->add(parent._texture);
                    COPY_MAT4F(parent._matrix, buf.parentMat);
                }
            }

            // Elevation sampler:
            buf.elevIndex = -1;
            if (tile._sharedSamplers != nullptr /* && is elevation active */)
            {
                const Sampler& s = (*tile._sharedSamplers)[SamplerBinding::ELEVATION];
                if (s._texture)
                {
                    s._texture->compress() = false;
                    s._texture->mipmap() = false;
                    s._texture->keepImage() = true; // never discard.. we use it elsewhere
                    buf.elevIndex = textures->add(s._texture);
                    COPY_MAT4F(s._matrix, buf.elevMat);
                }
            }

            // Normal sampler:
            buf.normalIndex = -1;
            if (tile._sharedSamplers != nullptr /* && is normalmapping active */)
            {
                const Sampler& s = (*tile._sharedSamplers)[SamplerBinding::NORMAL];
                if (s._texture)
                {
                    s._texture->compress() = false;
                    s._texture->mipmap() = true;
                    s._texture->maxAnisotropy() = 1.0f;
                    buf.normalIndex = textures->add(s._texture);
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
                    if (s._texture)
                    {
                        s._texture->compress() = false;
                        s._texture->mipmap() = false;
                        s._texture->maxAnisotropy() = 1.0f;
                        buf.landcoverIndex = textures->add(s._texture);
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
                    if (s._texture)
                    {
                        int k = i - SamplerBinding::SHARED;
                        if (k < MAX_NUM_SHARED_SAMPLERS)
                        {
                            s._texture->compress() = false;
                            s._texture->mipmap() = true;
                            //s._arena_texture->_maxAnisotropy = 4.0f;
                            buf.sharedIndex[k] = textures->add(s._texture);
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

        _rs.tiles.swap(_tiles);
        //_rs.tiles = _tiles;
        _tiles.clear(); // std::move(_tiles);

        // This will trigger a GPU upload on the next draw
        _rs.dirty = true;
    }
}

LayerDrawableNVGL::RenderState::RenderState()
{
    globjects.resize(64);
}

void
LayerDrawableNVGL::drawImplementation(osg::RenderInfo& ri) const
{
    // work around the GDP makOsg hack (ViewerBase.cpp:894)
    //ri.getState()->getGraphicsContext()->makeCurrent();

    // Research on glMultiDrawElementsIndirectBindlessNV:
    // https://github.com/ychding11/HelloWorld/wiki/Modern-GPU-Driven-Rendering--%28How-to-draw-fast%29
    // https://on-demand.gputechconf.com/siggraph/2014/presentation/SG4117-OpenGL-Scene-Rendering-Techniques.pdf
    // https://developer.download.nvidia.com/opengl/tutorials/bindless_graphics.pdf

    osg::State& state = *ri.getState();
    
    GLObjects& gl = GLObjects::get(_rs.globjects, state);

    if (_rs.tiles.empty())
        return;

    bool renderTerrainSurface = (_patchLayer == nullptr);

    if (gl.tiles == nullptr || !gl.tiles->valid())
    {
        gl.ext = osg::GLExtensions::Get(state.getContextID(), true);

        gl.tiles = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state);
        gl.tiles->bind();
        gl.tiles->debugLabel("REX geometry");

        // preallocate space for a bunch of tiles (just for fun)
        gl.tiles->bufferData(
            512 * sizeof(GL4Tile),
            nullptr,
            GL_DYNAMIC_DRAW);
    }

    if (renderTerrainSurface)
    {
        if (gl.commands == nullptr || !gl.commands->valid())
        {
            gl.commands = GLBuffer::create(GL_DRAW_INDIRECT_BUFFER, state);
            gl.commands->bind();
            gl.commands->debugLabel("REX geometry");
            // preallocate space for a bunch of draw commands (just for fun)
            gl.commands->bufferData(
                512 * sizeof(DrawElementsIndirectBindlessCommandNV),
                nullptr,
                GL_DYNAMIC_DRAW);
            gl.commands->unbind();

            osg::setGLExtensionFuncPtr(
                gl.glMultiDrawElementsIndirectBindlessNV,
                "glMultiDrawElementsIndirectBindlessNV");
            OE_HARD_ASSERT(gl.glMultiDrawElementsIndirectBindlessNV != nullptr);

            // OSG bug: glVertexAttribFormat is mapped to the wrong function :( so
            // we have to look it up fresh.
            osg::setGLExtensionFuncPtr(
                gl.glVertexAttribFormat,
                "glVertexAttribFormat");
            OE_HARD_ASSERT(gl.glVertexAttribFormat != nullptr);

            // Needed for core profile
            void(GL_APIENTRY * glEnableClientState_)(GLenum);
            osg::setGLExtensionFuncPtr(glEnableClientState_, "glEnableClientState");
            OE_HARD_ASSERT(glEnableClientState_ != nullptr);


            // Set up a VAO that we'll use to render with bindless NV.
            gl.vao = GLVAO::create(state);

            // Start recording
            gl.vao->bind();

            // after the bind, please
            gl.vao->debugLabel("REX geometry");

            // set up the VAO for NVIDIA bindless buffers
            glEnableClientState_(GL_VERTEX_ATTRIB_ARRAY_UNIFIED_NV);
            glEnableClientState_(GL_ELEMENT_ARRAY_UNIFIED_NV);

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
                gl.glVertexAttribFormat(location, 3, GL_FLOAT, GL_FALSE, offsets[location]);
                gl.ext->glVertexAttribBinding(location, 0);
                gl.ext->glEnableVertexAttribArray(location);
            }

            // bind a "dummy buffer" that will record the stride, which is
            // just the size of our vertex structure.
            gl.ext->glBindVertexBuffer(0, 0, 0, sizeof(GL4Vertex));

            // Finish recording
            gl.vao->unbind();
        }

        if (gl.shared == nullptr || !gl.shared->valid())
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

            gl.shared = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state);

            gl.shared->bind();
            gl.shared->debugLabel("REX geometry");
            gl.shared->bufferStorage(sizeof(GL4GlobalData), &buf, 0); // permanent
            gl.shared->unbind();
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
        gl.tiles->uploadData(
            _rs.tiles.size() * sizeof(GL4Tile),
            _rs.tilebuf.data());

        if (renderTerrainSurface)
        {
            // Reconstruct and upload the command list:
            _rs.commands.clear();
            for (auto& tile : _rs.tiles)
            {
                SharedGeometry* geom = tile._geom.get();
                _rs.commands.push_back(geom->getOrCreateNVGLCommand(state));
            }

            gl.commands->uploadData(_rs.commands);
        }
    }

    // Apply the the texture arena:
    //if (state.getLastAppliedAttribute(OE_TEXTURE_ARENA_SA_TYPE_ID) != _context->textures())
    {
        _context->textures()->apply(state);
        state.haveAppliedAttribute(_context->textures());
    }

    // Bind the tiles data to its layout(binding=X) in the shader.
    gl.tiles->bindBufferBase(31);


    if (renderTerrainSurface)
    {
        // Bind the command buffer for rendering.
        gl.commands->bind();

        // Bind the shared data to its layout(binding=X) in the shader.
        // For shared data we only need to do this once per pass
        if (_surfaceDrawOrder == 0)
        {
            gl.shared->bindBufferBase(30);
        }

        gl.vao->bind();

        GLenum primitive_type =
            _context->options().gpuTessellation() == true ?
            GL_PATCHES : GL_TRIANGLES;

        GLenum element_type =
            sizeof(DrawElementsBase::value_type) == sizeof(short) ?
            GL_UNSIGNED_SHORT : GL_UNSIGNED_INT;
        
        gl.glMultiDrawElementsIndirectBindlessNV(
            primitive_type,
            element_type,
            nullptr,
            _rs.commands.size(),
            sizeof(DrawElementsIndirectBindlessCommandNV),
            1);

        gl.vao->unbind();

        if (_clearOsgState)
        {
            gl.commands->unbind();
        }
    }

    else if (_patchLayer && _patchLayer->getRenderer())
    {
        // If it's a patch layer, the layer does its own rendering
        // TODO: pass along the fact that we're using GL4 so that
        // the patch layer doesn't actually APPLY each DrawTileCommand!
        TileBatch batch(_drawState.get());
        batch._tiles.reserve(_rs.tiles.size());
        for (auto& tile : _rs.tiles)
        {
            batch._tiles.push_back(&tile);
        }

        _patchLayer->getRenderer()->draw(ri, batch);
    }

    LayerDrawable::drawImplementation(ri);
}

void
LayerDrawableNVGL::releaseGLObjects(osg::State* state) const
{
    if (state)
    {
        GLObjects& gl = GLObjects::get(_rs.globjects, *state);
        gl.shared = nullptr;
        gl.tiles = nullptr;
        gl.commands = nullptr;
        gl.vao = nullptr;
    }
    else
    {
        _rs.globjects.setAllElementsTo(GLObjects());
    }

    _rs.dirty = true;

    LayerDrawable::releaseGLObjects(state);
}

void
LayerDrawableNVGL::resizeGLObjectBuffers(unsigned size)
{
    LayerDrawable::resizeGLObjectBuffers(size);
}
