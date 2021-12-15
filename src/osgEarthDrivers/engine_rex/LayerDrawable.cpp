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
#include <osgEarth/Metrics>
#include <sstream>

using namespace osgEarth::REX;

#undef  LC
#define LC "[LayerDrawable] "


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
    setDataVariance(DYNAMIC);
    setUseDisplayList(false);
    setUseVertexBufferObjects(true);
    _tiles.reserve(128);

    // set up an arena with "auto release" which means th textures
    // will automatically get released when all references drop.
    _textures = new TextureArena();
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
    OE_PROFILING_ZONE;
    if (_layer)
    {
        OE_PROFILING_ZONE_TEXT(_layer->getName().c_str());
    }

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
        if (_useIndirectRendering)
            drawImplementationIndirect(ri);
        else
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
        
        // unbind local buffers when finished.
        osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();

        // make sure any VAO is unbound before unbinind the VBO/EBOs,
        // as failing to do so will remove the VBO/EBO from the VAO
        if (ri.getState()->useVertexArrayObject(_useVertexArrayObject))
        {
            ri.getState()->unbindVertexArrayObject();
        }

        // Not necessary if using VAOs?
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


void
LayerDrawable::drawImplementationIndirect(osg::RenderInfo& ri) const
{
    RenderState& cs = _rs;
    GCState& gs = cs.gcState[ri.getContextID()];
    osg::State& state = *ri.getState();

    if (gs.tiles == nullptr)
    {
        gs.tiles = GLBuffer::create(
            GL_SHADER_STORAGE_BUFFER,
            state,
            "LayerDrawable Tiles");
    }

    if (gs.commands == nullptr)
    {
        gs.commands = GLBuffer::create(
            GL_DRAW_INDIRECT_BUFFER,
            state,
            "LayerDrawable DEI Commands");
    }

    if (_tiles != cs._previous_tiles)
    {
        int num_indices = 0;
        {
            OE_PROFILING_ZONE_NAMED("DEI Collect");

            if (cs.tilebuf.size() < _tiles.size())
                cs.tilebuf.resize(_tiles.size());

            if (cs.commands.size() == 0)
            {
                // Construct the draw command:
                DrawElementsIndirectCommand cmd;
                cmd.count = num_indices; // element count
                cmd.instanceCount = 1; // will fill this in later! // one instance per tile (look up using gl_InstanceID)
                cmd.firstIndex = 0; // offset into element array - zero since all tiles share the same element set
                cmd.baseVertex = 0; // unused - we aren't using a VBO
                cmd.baseInstance = 0; // no instancing
                cs.commands.emplace_back(std::move(cmd));
            }

            // copy all the vertex data into a single giant buffer so we can
            // update it to th GPU in one call. 
            // TODO: Benchmark to see whether it is faster to only update 
            // individual tiles that change BUT to use multiple uploads to do so.

            int tile_num = 0;

            for (auto& tile : _tiles)
            {
                TileBuffer& buf = cs.tilebuf[tile_num];

                //TODO: pre-build the tile buffer in each TileNode so all we have to do it copy it.

                osg::Vec3Array* verts = static_cast<osg::Vec3Array*>(tile._geom->getVertexArray());
                osg::Vec3Array* upvectors = static_cast<osg::Vec3Array*>(tile._geom->getNormalArray());
                osg::Vec3Array* uvs = static_cast<osg::Vec3Array*>(tile._geom->getTexCoordArray());

                for (int i = 0; i < verts->size(); ++i)
                {
                    const osg::Vec3f& v = (*verts)[i];
                    buf.verts[4 * i + 0] = v.x();
                    buf.verts[4 * i + 1] = v.y();
                    buf.verts[4 * i + 2] = v.z();
                    buf.verts[4 * i + 3] = (*uvs)[i].z(); // vertex flags marker

                    const osg::Vec3f& up = (*upvectors)[i];
                    buf.upvectors[4 * i + 0] = up.x();
                    buf.upvectors[4 * i + 1] = up.y();
                    buf.upvectors[4 * i + 2] = up.z();
                    buf.upvectors[4 * i + 3] = 0.0f;
                }

                // main MVM (double to float is OK)
                for (int i = 0; i < 16; ++i)
                    buf.modelViewMatrix[i] = tile._modelViewMatrix->ptr()[i];

                // Tile key encoding
                buf.tileKey[0] = tile._keyValue[0];
                buf.tileKey[1] = tile._keyValue[1];
                buf.tileKey[2] = tile._keyValue[2];
                buf.tileKey[3] = tile._keyValue[3];

                // Color sampler and matrix:
                buf.colorIndex = -1;
                buf.parentIndex = -1;
                if (tile._colorSamplers != nullptr)
                {
                    const Sampler& color = (*tile._colorSamplers)[SamplerBinding::COLOR];
                    if (color._arena_texture != nullptr)
                    {
                        buf.colorIndex = _textures->add(color._arena_texture);
                        for (int i = 0; i < 16; ++i) buf.colorMat[i] = color._matrix.ptr()[i];
                    }

                    const Sampler& parent = (*tile._colorSamplers)[SamplerBinding::COLOR_PARENT];
                    if (parent._arena_texture != nullptr)
                    {
                        buf.parentIndex = _textures->add(parent._arena_texture);
                        for (int i = 0; i < 16; ++i) buf.parentMat[i] = parent._matrix.ptr()[i];
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
                        for (int i = 0; i < 16; ++i) buf.elevMat[i] = s._matrix.ptr()[i];
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
                        for (int i = 0; i < 16; ++i) buf.normalMat[i] = s._matrix.ptr()[i];
                    }
                }

                // Morphing constants. Should probably put these in the global structure.


                //TODO: other samplers and shared samplers...
                buf.sharedIndex = -1;
                if (tile._sharedSamplers != nullptr)
                {
                    for (unsigned i = SamplerBinding::SHARED; i < tile._sharedSamplers->size(); ++i)
                    {
                        const Sampler& s = (*tile._sharedSamplers)[i];
                        if (s._arena_texture)
                        {
                            s._arena_texture->_compress = false;
                            s._arena_texture->_mipmap = true;
                            //s._arena_texture->_maxAnisotropy = 4.0f;
                            buf.sharedIndex = _textures->add(s._arena_texture);
                            for (int i = 0; i < 16; ++i) buf.sharedMat[i] = s._matrix.ptr()[i];
                        }

                        //TODO:
                        break;
                    }
                }


                //cs.tilebuf.emplace_back(std::move(buf));

                // First time through any layer? Make the global buffer
                // TODO: this will eventually go to the terrain level.
                if (gs.global == nullptr)
                {
                    GlobalBuffer buf;
                    for (unsigned i = 0; i < uvs->size(); ++i)
                    {
                        const osg::Vec3f& uv = (*uvs)[i];
                        buf.uvs[2 * i + 0] = uv.x();
                        buf.uvs[2 * i + 1] = uv.y();
                    }

                    gs.global = GLBuffer::create(
                        GL_SHADER_STORAGE_BUFFER,
                        state,
                        "LayerDrawable Global");

                    gs.global->bind();

                    gs.global->bufferStorage(
                        align((GLsizei)sizeof(GlobalBuffer), GLUtils::getSSBOAlignment(state)),
                        &buf,
                        0); // permanent
                }

                // First time through? Create the various shared buffer objects
                // we will need.
                if (gs.ebo == nullptr)
                {
                    // A shared EBO used by all tiles
                    osg::DrawElementsUShort* de = dynamic_cast<osg::DrawElementsUShort*>(
                        tile._geom->getDrawElements());

                    gs.ebo = GLBuffer::create(
                        GL_ELEMENT_ARRAY_BUFFER_ARB,
                        *ri.getState(),
                        "LayerDrawable EBO");

                    gs.ebo->bind();

                    gs.ebo->bufferStorage(
                        sizeof(unsigned short) * de->getNumIndices(),
                        const_cast<GLvoid*>(de->getDataPointer()),
                        0); // 0 = permanent storage. Will never be updated

                    // Make a fake VBO. It's fake because we never actually access it,
                    // but some drivers get upset if it's not there :)
                    gs.vbo = GLBuffer::create(
                        GL_ARRAY_BUFFER_ARB,
                        state,
                        "LayerDrawable VBO");

                    gs.vbo->bind();

                    gs.vbo->bufferStorage(
                        sizeof(float) * 4 * verts->size(),
                        nullptr,
                        0);

                    // More fakeness. A VAO we also don't use, but it required for GL CORE :)
                    gs.vbo->ext()->glGenVertexArrays(1, &gs.vao);
                    gs.vbo->ext()->glBindVertexArray(gs.vao);
                    gs.vbo->bind();
                    gs.vbo->ext()->glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
                    gs.vbo->ext()->glBindVertexArray(0);
                }

                // stuck the layer order here (for now...later, hide it elsewhere)
                buf.drawOrder = _drawOrder;

                // advance the tile counter
                ++tile_num;

                if (num_indices == 0)
                {
                    num_indices = tile._geom->getDrawElements()->getNumIndices();
                }
            }
        }

        // Construct the draw command:
        DrawElementsIndirectCommand& cmd = cs.commands.back();
        cmd.instanceCount = _tiles.size();
        cmd.count = num_indices;

        // Update the tile render buffer:
        GLsizei tileBufSize = sizeof(TileBuffer) * _tiles.size();

        if (tileBufSize > gs.tiles->size())
        {
            OE_PROFILING_ZONE_NAMED("Tiles bufferData");

            //OE_INFO << LC << "Reallocating tiles buffer = " << cs._tilebuf.size() << "(" << tileBufSize << " bytes)" << std::endl;
            gs.tiles->bind();

            // re-allocate the storage if necessary (check size)
            gs.tiles->bufferData(
                tileBufSize,
                cs.tilebuf.data(),
                GL_DYNAMIC_DRAW);
        }

        else
        {
            OE_PROFILING_ZONE_NAMED("Tiles bufferSubData");

            gs.tiles->bind();

            // copy the latest tile data to the GPU
            // TODO: only copy if it changed, or only copy the parts that changed
            gs.tiles->bufferSubData(
                0,
                tileBufSize,
                cs.tilebuf.data());
        }

        // Bind the indirect command buffer.
        gs.commands->bind();

        // Update the command buffer if necessary.
        GLsizei cmdBufSize = sizeof(DrawElementsIndirectCommand) * cs.commands.size();
        if (cmdBufSize > gs.commands->size())
        {
            OE_PROFILING_ZONE_NAMED("Cmd bufferData");

            // re-allocate the storage if necessary (check size)
            gs.commands->bufferData(
                cmdBufSize,
                cs.commands.data(),
                GL_DYNAMIC_DRAW);
        }
        else
        {
            OE_PROFILING_ZONE_NAMED("Cmd bufferSubData");
            gs.commands->bufferSubData(
                0,
                cmdBufSize,
                cs.commands.data());
        }

        cs._previous_tiles = _tiles;
    }
    else
    {
        // Bind the indirect command buffer.
        gs.commands->bind();
    }

    {
        OE_PROFILING_ZONE_NAMED("Draw");

        // we don't care about the MVM but we do need to projection matrix:
        if (ri.getState()->getUseModelViewAndProjectionUniforms())
            ri.getState()->applyModelViewAndProjectionUniformsIfRequired();

        // a VAO is required for GL CORE even though we don't really use it:
        gs.vbo->ext()->glBindVertexArray(gs.vao);

        // bind the shared element buffer object:
        gs.ebo->bind();

        // Bind the layout indices so we can access our tile data from the shader:
        gs.tiles->bindBufferBase(0);
        gs.global->bindBufferBase(1);

        // Apply the the texture arena:
        _textures->apply(state);

        GLFunctions::get(*ri.getState()).glDrawElementsIndirect(
            GL_TRIANGLES,
            GL_UNSIGNED_SHORT,
            nullptr);            // nullptr means data is bound at GL_DRAW_INDIRECT_BUFFER
    }
}
