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
    GLsizei alignment,
    osg::State& state)
{
    unsigned numCommands = geom->getNumDrawCommands();

    _requiredSize = numCommands * sizeof(DrawElementsIndirectCommand);

    if (_requiredSize > _allocatedSize || _buffer == nullptr)
    {
        //OE_PROFILING_GPU_ZONE("CommandBuffer::allocate");

        _geom = geom;

        release();
        if (_buf) delete [] _buf;

        _buf = new DrawElementsIndirectCommand[numCommands];
        _allocatedSize = align(_requiredSize, alignment);

        _buffer = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state, "OE IC DrawCommands");

        _buffer->bind();
        _buffer->allocateStorage(_allocatedSize, nullptr, GL_DYNAMIC_STORAGE_BIT);
    }
}

void
InstanceCloud::CommandBuffer::reset()
{
    //OE_PROFILING_GPU_ZONE("CommandBuffer::reset");

    // Initialize and blit to GPU
    if (_requiredSize > 0)
    {
        for (unsigned i = 0; i < _geom->getNumDrawCommands(); ++i)
        {
            _geom->getDrawCommand(i, _buf[i]);
        }

        _buffer->bind();
        _buffer->subData(0, _requiredSize, _buf);
    }
}

void
InstanceCloud::TileBuffer::allocate(unsigned numTiles, GLsizei alignment, osg::State& state)
{
    _requiredSize = numTiles * sizeof(Data);
    if (_requiredSize > _allocatedSize || _buffer == nullptr)
    {
        //OE_PROFILING_GPU_ZONE("TileBuffer::allocate");
        release();
        if (_buf) delete [] _buf;

        _buf = new Data[numTiles];
        _allocatedSize = align(_requiredSize, alignment);

        osg::GLExtensions* ext = state.get<osg::GLExtensions>();

        _buffer = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state, "OE IC TileBuffer");
        
        _buffer->bind();
        _buffer->allocateStorage(_allocatedSize, nullptr, GL_DYNAMIC_STORAGE_BIT);
    }
}

void
InstanceCloud::TileBuffer::update() const
{
    //OE_PROFILING_GPU_ZONE("TileBuffer::update");
    if (_requiredSize > 0)
    {
        _buffer->bind();
        _buffer->subData(0, _requiredSize, _buf);
    }
}

void
InstanceCloud::CullBuffer::allocate(unsigned numInstances, GLsizei alignment, osg::State& state)
{
    _requiredSize = sizeof(Data) + (numInstances * sizeof(GLuint));

    if (_requiredSize > _allocatedSize || _buffer == nullptr)
    {
        //OE_PROFILING_GPU_ZONE("InstanceBuffer::allocate");

        release();
        if (_buf) delete[] _buf;

        _buf = new Data();
        _allocatedSize = align(_requiredSize, alignment);

        _buffer = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state, "OE IC CullBuffer");

        _buffer->bind();
        _buffer->allocateStorage(_allocatedSize, nullptr, GL_DYNAMIC_STORAGE_BIT);
    }
}

void
InstanceCloud::CullBuffer::clear()
{
    //OE_PROFILING_GPU_ZONE("CullBuffer::clear");
    OE_SOFT_ASSERT_AND_RETURN(valid(), void());

    // Zero out the workgroup/instance count.
    _buf->di.num_groups_x = 0u;
    _buf->di.num_groups_y = 1u;
    _buf->di.num_groups_z = 1u;
    _buf->_padding[0] = 0;

    _buffer->bind();
    _buffer->subData(0, sizeof(Data), _buf);
}

void
InstanceCloud::InstanceBuffer::allocate(unsigned numTiles, unsigned numInstancesPerTile, GLsizei alignment, osg::State& state)
{
    unsigned numBytesPerTile = numInstancesPerTile * sizeof(InstanceData);
    _requiredSize = numTiles * numBytesPerTile;
    if (_requiredSize > _allocatedSize || _buffer == nullptr)
    {
        //OE_PROFILING_GPU_ZONE("GenBuffer::allocate");

        release();
        if (_buf) delete[] _buf;

        _numInstancesPerTile = numInstancesPerTile;
        _allocatedSize = align(_requiredSize, alignment);

        _buffer = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state, "OE IC GenBuffer");

        _buffer->bind();
        _buffer->allocateStorage(_allocatedSize, nullptr, 0);
    }
}

void
InstanceCloud::RenderBuffer::allocate(unsigned numInstances, GLsizei alignment, osg::State& state)
{
    _requiredSize = numInstances * (sizeof(GLuint)*2); // sizeof RenderLeaf
    if (_requiredSize > _allocatedSize || _buffer == nullptr)
    {
        //OE_PROFILING_GPU_ZONE("RenderBuffer::allocate");

        release();

        _allocatedSize = align(_requiredSize, alignment);

        _buffer = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state, "OE IC RenderBuffer");

        _buffer->bind();
        _buffer->allocateStorage(_allocatedSize, nullptr, 0);
    }
}

InstanceCloud::InstancingData::InstancingData() :
    _numTilesAllocated(0u),
    _alignment((GLsizei)-1),
    _numX(0u),
    _numY(0u)
{
    // Layout bindings. Make sure these match the layout specs in the shaders.
    _commandBuffer._bindingIndex = 0;
    _instanceBuffer._bindingIndex = 1;
    _cullBuffer._bindingIndex = 2;
    _tileBuffer._bindingIndex = 3;
    _renderBuffer._bindingIndex = 4;
}

InstanceCloud::InstancingData::~InstancingData()
{
    //nop
}

bool
InstanceCloud::InstancingData::allocateGLObjects(osg::State& state, unsigned numTiles)
{
    OE_PROFILING_ZONE;

    if (_alignment < 0)
    {
        glGetIntegerv(GL_SHADER_STORAGE_BUFFER_OFFSET_ALIGNMENT, &_alignment);
        OE_DEBUG << LC << "SSBO Alignment = " << _alignment << std::endl;
    }

    _numTilesActive = numTiles;

    if (numTiles > _numTilesAllocated)
    {
        numTiles = align(numTiles, 8u);

        GLuint numInstances = numTiles * _numX * _numY;

        _commandBuffer.allocate(_geom.get(), _alignment, state);
        _instanceBuffer.allocate(numTiles, _numX*_numY, _alignment, state);
        _cullBuffer.allocate(numInstances, _alignment, state);
        _tileBuffer.allocate(numTiles, _alignment, state);
        _renderBuffer.allocate(numInstances, _alignment, state);

        _numTilesAllocated = numTiles;
        OE_DEBUG << "Re-alloc: tiles = " << _numTilesAllocated << std::endl;

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

InstanceCloud::InstanceCloud()
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
InstanceCloud::bind()
{
    _data._commandBuffer.bindLayout();
    _data._instanceBuffer.bindLayout();
    _data._tileBuffer.bindLayout();
    _data._cullBuffer.bindLayout();
    _data._renderBuffer.bindLayout();
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
    osg::Matrixf modelViewF = modelView;
    ::memcpy(
        _data._tileBuffer._buf[tileNum]._modelViewMatrix,
        modelViewF.ptr(),
        sizeof(TileBuffer::Data::_modelViewMatrix));

    // derive normal matrix (transposed inverse of MVM 3x3)
    // NOTE: ignore this for now, and just derive it from
    // the mat3(mvm) in the shader. This works as long as the
    // MVM is not anisotropic (scale factors are different on
    // different axes) which is (?) always the case in OE
#if 0
    osg::Matrix mv(modelView);
    mv.setTrans(0.0, 0.0, 0.0);
    osg::Matrix matrix;
    matrix.invert(mv);
    osg::Matrixf normalMatrix(
        matrix(0,0), matrix(1,0), matrix(2,0), matrix(3,0),
        matrix(0,1), matrix(1,1), matrix(2,1), matrix(3,1),
        matrix(0,2), matrix(1,2), matrix(2,2), matrix(3,2),
        matrix(0,3), matrix(1,3), matrix(2,3), matrix(3,3));
    ::memcpy(
        _data._tileBuffer._buf[tileNum]._normalMatrix,
        normalMatrix.ptr(),
        sizeof(TileBuffer::Data::_normalMatrix));
#endif
}

void
InstanceCloud::setTileActive(unsigned tileNum, bool value)
{
    //OE_SOFT_ASSERT(tileNum < _data._numTilesAllocated);

    if (tileNum < _data._numTilesAllocated)
        _data._tileBuffer._buf[tileNum]._inUse = value ? 1 : 0;
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

    OE_SOFT_ASSERT_AND_RETURN(_data._commandBuffer.valid(), void());
    OE_SOFT_ASSERT_AND_RETURN(_data._cullBuffer.valid(), void());
    OE_SOFT_ASSERT_AND_RETURN(_data._geom != nullptr, void());

    osg::State& state = *ri.getState();
    osg::GLExtensions* ext = state.get<osg::GLExtensions>();

    // reset the command buffer in prep for culling and sorting
    _data._commandBuffer.reset();

    // update with the newest tile matrices and data
    _data._tileBuffer.update();

    // Bind the indirect dispatch parameters to the header
    // of our cull buffer, which contains the X=instance count and YZ=1.
    _data._cullBuffer._buffer->bind(GL_DISPATCH_INDIRECT_BUFFER);

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

        GLFunctions::get(state).
            glDispatchComputeIndirect(0);
    }

    // second pass: sort
    {
        //OE_PROFILING_GPU_ZONE("IC:Sort");

        ext->glUniform1i(pcp_data->_passUL, (int)PASS_SORT);
        ext->glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT); // prep to read from SSBO

        GLFunctions::get(state).
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
    OE_SOFT_ASSERT_AND_RETURN(_data._commandBuffer.valid(), void());
    OE_SOFT_ASSERT_AND_RETURN(_data._geom != nullptr, void());

    // GL_DRAW_INDIRECT_BUFFER buffing binding is NOT part of VAO state (per spec)
    // so we have to bind it here.
    _data._commandBuffer._buffer->bind(GL_DRAW_INDIRECT_BUFFER);

    // sync to read from SSBO and CMD
    osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();
    ext->glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_COMMAND_BARRIER_BIT);

    // draw the geom
    _data._geom->draw(ri);

    // unbind it after draw is complete.
    ext->glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);
}
