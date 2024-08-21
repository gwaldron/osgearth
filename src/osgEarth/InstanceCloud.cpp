/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include "InstanceCloud"
#include "GeometryCloud"
#include "Metrics"
#include "Utils"

#ifndef GL_DYNAMIC_STORAGE_BIT
#define GL_DYNAMIC_STORAGE_BIT 0x0100
#endif

using namespace osgEarth;

#undef LC
#define LC "[InstanceCloud] "

#define PASS_GENERATE (int)0
#define PASS_MERGE    (int)1
#define PASS_CULL     (int)2
#define PASS_SORT     (int)3

//...................................................................

// pre OSG-3.6 support
#ifndef GL_DRAW_INDIRECT_BUFFER
#define GL_DRAW_INDIRECT_BUFFER 0x8F3F
#endif

void
InstanceCloud::CommandBuffer::allocate(
    GeometryCloud* geom,
    osg::State& state)
{
    if (_buf == nullptr)
    {
        _buf = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state);
        _buf->bind();
        _buf->debugLabel("InstanceCloud");
        _buf->unbind();
    }

    _geom = geom;

    if (_geom && _geom->getNumDrawCommands() > _backing.size())
    {
        _backing.resize(_geom->getNumDrawCommands());
    }
}

void
InstanceCloud::CommandBuffer::reset(osg::State& state)
{
    //OE_PROFILING_GPU_ZONE("CommandBuffer::reset");

    OE_HARD_ASSERT(_geom != nullptr);

    if (_geom->getNumDrawCommands() > 0)
    {
        OE_HARD_ASSERT(_backing.size() == _geom->getNumDrawCommands());

        // copy comands into our backing store
        for (unsigned i = 0; i < _geom->getNumDrawCommands(); ++i)
        {
            _geom->getDrawCommand(i, _backing[i]);
        }

        _buf->uploadData(
            sizeof(DrawElementsIndirectCommand)*_backing.size(),
            _backing.data());
    }
}

void
InstanceCloud::TileBuffer::allocate(
    unsigned numTiles,
    osg::State& state)
{
    if (_buf == nullptr)
    {
        _buf = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state);
        _buf->bind();
        _buf->debugLabel("InstanceCloud");
        _buf->unbind();
    }

    if (numTiles > 0)
    {
        _backing.resize(numTiles);
    }
}

void
InstanceCloud::TileBuffer::update() const
{
    //OE_PROFILING_GPU_ZONE("TileBuffer::update");
    if (_backing.size() > 0)
    {
        _buf->uploadData(sizeof(Data) * _backing.size(), _backing.data());
    }
}

void
InstanceCloud::CullBuffer::allocate(
    unsigned numInstances,
    osg::State& state)
{
    if (_buf == nullptr)
    {
        _buf = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state);
        _buf->bind();
        _buf->debugLabel("InstanceCloud");
        _buf->unbind();
    }

    // allocate enough space for the data header and all instance refs
    GLsizei size = sizeof(Data) + (numInstances * sizeof(GLuint));
    if (size > _buf->size())
    {
        _buf->uploadData(size, nullptr);
    }
}

void
InstanceCloud::CullBuffer::clear()
{
    //OE_PROFILING_GPU_ZONE("CullBuffer::clear");
    OE_SOFT_ASSERT_AND_RETURN(_buf != nullptr, void());

    // Zero out the workgroup/instance count.
    _backing.di.num_groups_x = 0u;
    _backing.di.num_groups_y = 1u;
    _backing.di.num_groups_z = 1u;
    _backing._padding[0] = 0;

    _buf->bind();
    _buf->bufferSubData(0, sizeof(Data), &_backing);
}

void
InstanceCloud::InstanceBuffer::allocate(
    unsigned numTiles, 
    unsigned numInstancesPerTile,
    osg::State& state)
{
    if (_buf == nullptr)
    {
        _buf = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state);
        _buf->bind();
        _buf->debugLabel("InstanceCloud");
        _buf->unbind();
    }

    GLsizei size = sizeof(InstanceData) * numTiles * numInstancesPerTile;
    if (size > _buf->size())
    {
        _buf->uploadData(size, nullptr);
    }
}

void
InstanceCloud::RenderBuffer::allocate(
    unsigned numInstances,
    osg::State& state)
{
    if (_buf == nullptr)
    {
        _buf = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state);
        _buf->bind();
        _buf->debugLabel("InstanceCloud");
        _buf->unbind();
    }

    GLsizei size = numInstances * (sizeof(GLuint)*2); // sizeof RenderLeaf
    if (size > _buf->size())
    {
        _buf->uploadData(size, nullptr);
    }
}

InstanceCloud::InstancingData::InstancingData() :
    _numTilesAllocated(0u),
    _numX(0u),
    _numY(0u)
{
    //nop
}

InstanceCloud::InstancingData::~InstancingData()
{
    //nop
}

bool
InstanceCloud::InstancingData::allocateGLObjects(osg::State& state, unsigned numTiles)
{
    OE_PROFILING_ZONE;

    _numTilesActive = numTiles;

    if (numTiles > _numTilesAllocated)
    {
        numTiles = align(numTiles, 8u);

        GLuint numInstances = numTiles * _numX * _numY;

        _commandBuffer.allocate(_geom.get(), state);
        _instanceBuffer.allocate(numTiles, _numX*_numY, state);
        _cullBuffer.allocate(numInstances, state);
        _tileBuffer.allocate(numTiles, state);
        _renderBuffer.allocate(numInstances, state);

        _numTilesAllocated = numTiles;

        return true;
    }

    return false;
}

void
InstanceCloud::InstancingData::releaseGLObjects(osg::State* state) const
{
    OE_PROFILING_ZONE;
    _commandBuffer.release();
    _instanceBuffer.release();
    _tileBuffer.release();
    _cullBuffer.release();
    _renderBuffer.release();
    _numTilesAllocated = 0;
}

InstanceCloud::InstanceCloud() :
    glDispatchComputeIndirect(nullptr)
{
    //nop
}

void
InstanceCloud::releaseGLObjects(osg::State* state) const
{
    _data.releaseGLObjects(state);
}

void
InstanceCloud::setGeometryCloud(GeometryCloud* geom)
{
    if (_data._geom.get() != geom)
    {
        GeometryValidator validator;
        geom->getGeometry()->accept(validator);

        _data._geom = geom;
        releaseGLObjects(nullptr);
    }
}

GeometryCloud*
InstanceCloud::getGeometryCloud() const 
{
    return _data._geom.get();
}

void
InstanceCloud::setNumInstancesPerTile(unsigned x, unsigned y)
{
    // Enforce an even number of instances. For whatever reason that I cannot
    // figure out today, an odd number causes the shader to freak out.
    _data._numX = (x & 0x01) ? x+1 : x;
    _data._numY = (y & 0x01) ? y+1 : y;
}

unsigned
InstanceCloud::getNumInstancesPerTile() const
{
    return _data._numX * _data._numY;
}

bool
InstanceCloud::allocateGLObjects(osg::RenderInfo& ri, unsigned numTiles)
{
    return _data.allocateGLObjects(*ri.getState(), numTiles);
}

void
InstanceCloud::bind(osg::RenderInfo& ri)
{
    // These map the the "layout binding" directives in the shader
    _data._commandBuffer._buf->bindBufferBase(0u);
    _data._instanceBuffer._buf->bindBufferBase(1u);
    _data._cullBuffer._buf->bindBufferBase(2u);
    _data._tileBuffer._buf->bindBufferBase(3u);
    _data._renderBuffer._buf->bindBufferBase(4u);
}

void
InstanceCloud::setHighestTileSlot(unsigned slot)
{
    _data._highestTileSlotInUse = slot;
}

void
InstanceCloud::setMatrix(unsigned tileNum, const osg::Matrix& modelView)
{
    // double to float before doing a memcpy!
    TileBuffer::Data& tile = _data._tileBuffer._backing[tileNum];

    osg::Matrixf modelViewF = modelView;
    ::memcpy(
        tile._modelViewMatrix,
        modelViewF.ptr(),
        sizeof(TileBuffer::Data::_modelViewMatrix));
}

void
InstanceCloud::setTileActive(unsigned tileNum, bool value)
{
    //OE_SOFT_ASSERT(tileNum < _data._numTilesAllocated);

    if (tileNum < _data._numTilesAllocated)
    {
        TileBuffer::Data& data = _data._tileBuffer._backing[tileNum];
        data._inUse = value ? 1 : 0;
    }
}

void
InstanceCloud::generate_begin(osg::RenderInfo& ri)
{
    osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();

    PCPData* pcp_data = _data.getPCPData(ri);
    if (!pcp_data) return;

    ext->glUniform1i(pcp_data->_passUL, PASS_GENERATE);

    // Reset the instance buffer so we can generate all new data
    _data._cullBuffer.clear();
}

void
InstanceCloud::generate_tile(osg::RenderInfo& ri)
{
    osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();
    ext->glDispatchCompute(_data._numX, _data._numY, 1);
    //OE_INFO << LC << "glDispatchCompute(" << _data._numX << ", " << _data._numY << ", 1)" << std::endl;
}

void
InstanceCloud::generate_end(osg::RenderInfo& ri)
{
    osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();

    // run the merge pass to consolidate gen buffers into instance buffer
    PCPData* pcp_data = _data.getPCPData(ri);
    if (!pcp_data) return;

    ext->glUniform1i(pcp_data->_passUL, (int)PASS_MERGE);

    int numPossibleInstances = (_data._highestTileSlotInUse+1) * _data._numX * _data._numY;

    // To update the "in use" flag for each tile before the merge:
    _data._tileBuffer.update();
    
    // prepare to read from SSBO:
    ext->glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    ext->glDispatchCompute(numPossibleInstances, 1, 1);
}

void
InstanceCloud::cull(osg::RenderInfo& ri)
{
    OE_PROFILING_ZONE;

    //OE_SOFT_ASSERT_AND_RETURN(_data._commandBuffer.valid(), void());
    OE_SOFT_ASSERT_AND_RETURN(_data._cullBuffer._buf != nullptr, void());
    OE_SOFT_ASSERT_AND_RETURN(_data._geom != nullptr, void());

    osg::State& state = *ri.getState();
    osg::GLExtensions* ext = state.get<osg::GLExtensions>();

    if (glDispatchComputeIndirect == nullptr)
        osg::setGLExtensionFuncPtr(glDispatchComputeIndirect, "glDispatchComputeIndirect", "glDispatchComputeIndirectARB");

    // reset the command buffer in prep for culling and sorting
    _data._commandBuffer.reset(state);

    // update with the newest tile matrices and data
    _data._tileBuffer.update();

    // Bind the indirect dispatch parameters to the header
    // of our cull buffer, which contains the X=instance count and YZ=1.
    _data._cullBuffer._buf->bind(GL_DISPATCH_INDIRECT_BUFFER);

    // need the projection matrix for frustum culling.
    if (state.getUseModelViewAndProjectionUniforms())
        state.applyModelViewAndProjectionUniformsIfRequired();

    PCPData* pcp_data = _data.getPCPData(ri);
    if (!pcp_data) return;

    // first pass: cull
    {
        //OE_PROFILING_GPU_ZONE("IC:Cull");
        ext->glUniform1i(pcp_data->_passUL, (int)PASS_CULL);
        ext->glUniform1i(pcp_data->_numCommandsUL, (int)_data._geom->getNumDrawCommands());
        ext->glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT); // prep to read from SSBO
        glDispatchComputeIndirect(0);
    }

    // second pass: sort
    {
        //OE_PROFILING_GPU_ZONE("IC:Sort");

        ext->glUniform1i(pcp_data->_passUL, (int)PASS_SORT);
        ext->glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT); // prep to read from SSBO
        glDispatchComputeIndirect(0);
    }
}

void
InstanceCloud::unbind(osg::RenderInfo& ri)
{
    // be a good citizen
    //osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();
    //ext->glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);
}
void
InstanceCloud::draw(osg::RenderInfo& ri)
{
    //OE_PROFILING_GPU_ZONE("IC:Draw");
    OE_SOFT_ASSERT_AND_RETURN(_data._commandBuffer._buf != nullptr, void());
    OE_SOFT_ASSERT_AND_RETURN(_data._geom != nullptr, void());

    osg::GLExtensions* ext = _data._commandBuffer._buf->ext();

    // GL_DRAW_INDIRECT_BUFFER buffing binding is NOT part of VAO state (per spec)
    // so we have to bind it here.
    _data._commandBuffer._buf->bind(GL_DRAW_INDIRECT_BUFFER);

    // sync to read from SSBO and CMD
    ext->glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_COMMAND_BARRIER_BIT);

    // draw the geom
    _data._geom->draw(ri);

    // unbind it after draw is complete.
    ext->glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);
}
