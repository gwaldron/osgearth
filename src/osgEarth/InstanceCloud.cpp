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
#include <osgEarth/InstanceCloud>
#include <osgEarth/ShaderLoader>
#include <osgEarth/Utils>
#include <osgEarth/ImageUtils>
#include <osgEarth/Math>
#include <osgEarth/GLUtils>
#include <osgEarth/Metrics>
#include <osg/Program>
#include <osg/GLExtensions>
#include <osgUtil/Optimizer>
#include <osgUtil/MeshOptimizers>
#include <iterator>

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

    if (_requiredSize > _allocatedSize)
    {
        OE_PROFILING_GPU_ZONE("CommandBuffer::allocate");

        _geom = geom;

        release();
        if (_buf) delete [] _buf;

        _buf = new DrawElementsIndirectCommand[numCommands];
        _allocatedSize = align(_requiredSize, alignment);

        _buffer = new GLBuffer(GL_SHADER_STORAGE_BUFFER, state, "OE IC DrawCommands");

        _buffer->bind();

        GLFunctions::get(state).
            glBufferStorage(GL_SHADER_STORAGE_BUFFER, _allocatedSize, NULL, GL_DYNAMIC_STORAGE_BIT);
    }
}

void
InstanceCloud::CommandBuffer::reset()
{
    OE_PROFILING_GPU_ZONE("CommandBuffer::reset");

    // Initialize and blit to GPU
    for(unsigned i=0; i<_geom->getNumDrawCommands(); ++i)
    {
        _geom->getDrawCommand(i, _buf[i]);
    }

    _buffer->bind();
    _buffer->ext()->glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, _requiredSize, _buf);
}

void
InstanceCloud::TileBuffer::allocate(unsigned numTiles, GLsizei alignment, osg::State& state)
{
    if (numTilesAllocated < numTiles || commands == nullptr)
    {
        OE_PROFILING_GPU_ZONE("TileBuffer::allocate");
        release();
        if (_buf) delete [] _buf;

        _buf = new Data[numTiles];
        _allocatedSize = align(_requiredSize, alignment);

        osg::GLExtensions* ext = state.get<osg::GLExtensions>();

        _buffer = new GLBuffer(GL_SHADER_STORAGE_BUFFER, state, "OE IC TileBuffer");
        
        _buffer->bind();

        GLFunctions::get(state).
            glBufferStorage(GL_SHADER_STORAGE_BUFFER, _allocatedSize, NULL, GL_DYNAMIC_STORAGE_BIT);  
    }
}

void
InstanceCloud::TileBuffer::update() const
{
    OE_PROFILING_GPU_ZONE("TileBuffer::update");
    _buffer->bind();
    _buffer->ext()->glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, _requiredSize, _buf);
}

void
InstanceCloud::CullBuffer::allocate(unsigned numInstances, GLsizei alignment, osg::State& state)
{
    _requiredSize = 
        sizeof(DispatchIndirectCommand) +
        sizeof(GLuint) +
        (numInstances * sizeof(GLuint));
        //(numInstances * sizeof(InstanceData));

    if (_requiredSize > _allocatedSize)
    {
        OE_PROFILING_GPU_ZONE("InstanceBuffer::allocate");

        release();
        if (_buf) delete[] _buf;

        _buf = new DispatchIndirectCommand;
        _allocatedSize = align(_requiredSize, alignment);

        _buffer = new GLBuffer(GL_SHADER_STORAGE_BUFFER, state, "OE IC InstBuffer");

        _buffer->bind();

        GLFunctions::get(state).
            glBufferStorage(GL_SHADER_STORAGE_BUFFER, _allocatedSize, NULL, GL_DYNAMIC_STORAGE_BIT);
    }
}

void
InstanceCloud::CullBuffer::clear()
{
    OE_PROFILING_GPU_ZONE("CullBuffer::clear");

    // Zero out the workgroup/instance count.
    _buf->num_groups_x = 0u;
    _buf->num_groups_y = 1u;
    _buf->num_groups_z = 1u;

    _buffer->bind();
    _buffer->ext()->glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(DispatchIndirectCommand), _buf);
}

void
InstanceCloud::InstanceBuffer::allocate(unsigned numTiles, unsigned numInstancesPerTile, GLsizei alignment, osg::State& state)
{
    unsigned numBytesPerTile = numInstancesPerTile * sizeof(InstanceData);
    _requiredSize = numTiles * numBytesPerTile;
    if (_requiredSize > _allocatedSize)
    {
        OE_PROFILING_GPU_ZONE("GenBuffer::allocate");

        release();
        if (_buf) delete[] _buf;

        _numInstancesPerTile = numInstancesPerTile;
        _allocatedSize = align(_requiredSize, alignment);

        _buffer = new GLBuffer(GL_SHADER_STORAGE_BUFFER, state, "OE IC GenBuffer");

        _buffer->bind();

        GLFunctions::get(state).
            glBufferStorage(GL_SHADER_STORAGE_BUFFER, _allocatedSize, NULL, 0);
    }
}

void
InstanceCloud::RenderBuffer::allocate(unsigned numInstances, GLsizei alignment, osg::State& state)
{
    _requiredSize = numInstances*sizeof(GLuint);
    if (_requiredSize > _allocatedSize)
    {
        OE_PROFILING_GPU_ZONE("RenderBuffer::allocate");

        release();

        _allocatedSize = align(_requiredSize, alignment);

        _buffer = new GLBuffer(GL_SHADER_STORAGE_BUFFER, state, "OE IC RenderBuffer");

        _buffer->bind();

        GLFunctions::get(state).
            glBufferStorage(GL_SHADER_STORAGE_BUFFER, _allocatedSize, NULL, 0);
    }
}

InstanceCloud::InstancingData::InstancingData() :
    _numTilesAllocated(0u),
    _alignment((GLsizei)-1),
    _passUL((GLint)-1),
    _numCommandsUL((GLint)-1),
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
        OE_DEBUG << "SSBO Alignment = " << _alignment << std::endl;
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
}

InstanceCloud::InstanceCloud()
{
    //nop
}

void
InstanceCloud::releaseGLObjects(osg::State* state) const
{
    _data.releaseGLObjects(state);
    _geom->releaseGLObjects(state);
}

void
InstanceCloud::setGeometryCloud(GeometryCloud* geom)
{
    GeometryValidator validator;
    geom->getGeometry()->accept(validator);

    _data._geom = geom;
    if (geom)
    {
        Installer installer(geom);
        geom->getGeometry()->accept(installer);
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
InstanceCloud::newFrame()
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
    if (tileNum < _data._numTilesAllocated)
        _data._tileBuffer._buf[tileNum]._inUse = value ? 1 : 0;
}

void
InstanceCloud::generate_begin(osg::RenderInfo& ri)
{
    osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();

    // find the "pass" uniform first time in
    if (_data._passUL < 1)
    {
        const osg::Program::PerContextProgram* pcp = ri.getState()->getLastAppliedProgramObject();
        if (!pcp) return;

        _data._passUL = pcp->getUniformLocation(osg::Uniform::getNameID("oe_pass"));
        _data._numCommandsUL = pcp->getUniformLocation(osg::Uniform::getNameID("oe_gc_numCommands"));
    }

    ext->glUniform1i(_data._passUL, PASS_GENERATE);

    // Reset the instance buffer so we can generate all new data
    _data._cullBuffer.clear();
}

void
InstanceCloud::generate_tile(osg::RenderInfo& ri)
{
    osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();
    ext->glDispatchCompute(_data._numX, _data._numY, 1);
}

void
InstanceCloud::generate_end(osg::RenderInfo& ri)
{
    osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();

    // run the merge pass to consolidate gen buffers into instance buffer.
    // TODO: optimization. During generate, store the count per tile into an
    // intermediate location (array?) and count the total gens into a DI buffer. Use
    // that to dispatch indirect.
    ext->glUniform1i(_data._passUL, (int)PASS_MERGE);

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

    osg::State& state = *ri.getState();
    osg::GLExtensions* ext = state.get<osg::GLExtensions>();

    // reset the command buffer in prep for culling and sorting
    _data._commandBuffer.reset();

    // update with the newest tile matrices and data
    _data._tileBuffer.update();

    // need the projection matrix for frustum culling.
    if (state.getUseModelViewAndProjectionUniforms())
        state.applyModelViewAndProjectionUniformsIfRequired();

    // Bind the indirect dispatch parameters to the header
    // of our instance buffer, which contains the X=instance count and YZ=1.
    _data._cullBuffer._buffer->bind(GL_DISPATCH_INDIRECT_BUFFER);

    // first pass: cull
    {
        OE_PROFILING_GPU_ZONE("IC:Cull");

        ext->glUniform1i(_data._passUL, (int)PASS_CULL);
        ext->glUniform1i(_data._numCommandsUL, (int)_data._geom->getNumDrawCommands());
        ext->glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT); // prep to read from SSBO

        GLFunctions::get(state).
            glDispatchComputeIndirect(0);
    }

    // second pass: sort
    {
        OE_PROFILING_GPU_ZONE("IC:Sort");

        ext->glUniform1i(_data._passUL, (int)PASS_SORT);
        ext->glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT); // prep to read from SSBO

        GLFunctions::get(state).
            glDispatchComputeIndirect(0);
    }
}

void
InstanceCloud::endFrame(osg::RenderInfo& ri)
{
    // be a good citizen
    osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();
    ext->glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);
}
void
InstanceCloud::draw(osg::RenderInfo& ri)
{
    OE_PROFILING_GPU_ZONE("IC:Draw");

    // GL_DRAW_INDIRECT_BUFFER buffing binding is NOT part of VAO state (per spec)
    // so we have to bind it here.
    _data._commandBuffer._buffer->bind(GL_DRAW_INDIRECT_BUFFER);

    // sync to read from SSBO and CMD
    osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();
    ext->glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_COMMAND_BARRIER_BIT);

    // draw the geom
    _data._geom->draw(ri);
}


InstanceCloud::Renderer::Renderer(GeometryCloud* geom) :
    _geom(geom)
{
    //nop
}

void
InstanceCloud::Renderer::drawImplementation(osg::RenderInfo& ri, const osg::Drawable* drawable) const
{
    OE_PROFILING_ZONE_NAMED("drawImplementation");

    osg::State& state = *ri.getState();

    osg::GLExtensions* ext = state.get<osg::GLExtensions>();

    const osg::Geometry* geom = static_cast<const osg::Geometry*>(drawable);

    // prepare the VAO, etc.
    osg::VertexArrayState* vas = state.getCurrentVertexArrayState();
    vas->setVertexBufferObjectSupported(true);
    geom->drawVertexArraysImplementation(ri);

    // bind the combined EBO to the VAO
    if (vas->getRequiresSetArrays())
    {
        OE_PROFILING_ZONE_NAMED("Bind EBO");
        osg::GLBufferObject* ebo = geom->getPrimitiveSet(0)->getOrCreateGLBufferObject(state.getContextID());
        state.bindElementBufferObject(ebo);
    }

    // engage

    {
    OE_PROFILING_ZONE_NAMED("MEDI");
    GLFunctions::get(state).
        glMultiDrawElementsIndirect(
            GL_TRIANGLES,
            GL_UNSIGNED_SHORT,
            NULL,                         // in GL_DRAW_INDIRECT_BUFFER on GPU
            _geom->getNumDrawCommands(),  // number of commands to execute
            0);                           // stride=0, commands are tightly packed
    }
}


InstanceCloud::Installer::Installer(GeometryCloud* geom) :
    osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
{
    _callback = new Renderer(geom);
}

void 
InstanceCloud::Installer::apply(osg::Drawable& drawable)
{
    osg::Geometry* geom = drawable.asGeometry();
    if (geom)
    {
        geom->setDrawCallback(_callback.get());
        geom->setCullingActive(false);
    }
}

//...................................................................

GeometryCloud::GeometryCloud() :
    osg::NodeVisitor()
{
    setTraversalMode(TRAVERSE_ALL_CHILDREN);
    setNodeMaskOverride(~0);

    _geom = new osg::Geometry();
    _geom->setUseVertexBufferObjects(true);
    _geom->setUseDisplayList(false);

    _verts = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    _geom->setVertexArray(_verts);

    _colors = new osg::Vec4Array(osg::Array::BIND_PER_VERTEX);
    _geom->setColorArray(_colors);

    _normals = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    _geom->setNormalArray(_normals);

    _texcoords = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    _geom->setTexCoordArray(7, _texcoords);

    // N.B. UShort is sufficient b/c we are using the DrawElementsIndirect.baseVertex
    // technique to offset element indicies and thereby support a VBO with >65535 verts.
    _primset = new osg::DrawElementsUShort(GL_TRIANGLES);
    _geom->addPrimitiveSet(_primset);

    _geom->getOrCreateStateSet()->setMode(GL_BLEND, 1);
}

Texture*
GeometryCloud::add(osg::Node* node, unsigned alignment)
{
    // convert all primitive sets to GL_TRIANGLES
    osgUtil::Optimizer o;
    o.optimize(node, o.INDEX_MESH);

    // Reorder indices for optimal cache usage.
    // DO NOT do this if alignment is set; using alignment
    // implies the verts are in a specific order for a good reason.
    if (alignment == 0u)
    {
        osgUtil::VertexCacheVisitor vcv;
        node->accept(vcv);
        vcv.optimizeVertices();
    }

    // pad the arrays to the alignment:
    if (alignment > 0u)
    {
        unsigned padding = align((unsigned)_verts->size(), alignment) - _verts->size();
        OE_DEBUG << "vert size = " << _verts->size() << " so we gonna pad it with " << padding << " bytes" << std::endl;
        for(unsigned i=0; i<padding; ++i)
        {   
            _verts->push_back(osg::Vec3());
            _colors->push_back(osg::Vec4(1,0,0,1));
            _normals->push_back(osg::Vec3(0,0,1));
            _texcoords->push_back(osg::Vec3(0,0,0));
        }
    }

    // track the starting vertex of this model:
    _vertexOffsets.push_back(_verts->size());

    // track the starting element index of this model:
    _elementOffsets.push_back(_primset->getNumIndices());

    // traverse the model, consolidating arrays and rewriting texture indices
    // and counting elements indices
    _numElements = 0u;
    _texLayer = 0;
    _texturesToAdd.clear();
    node->accept(*this);
    _elementCounts.push_back(_numElements);

    // finally, consolidate all discovered textures into a single "mini-atlas"
    // and return it.
    if (_texturesToAdd.empty())
        return NULL; // nothing.

    // find the largest image and make that the atlas size.
    GLenum pixelFormat = GL_RGBA;
    GLenum dataType = GL_UNSIGNED_BYTE;
    GLenum internalFormat = GL_RGBA8;

    int width=0, height=0;
    for(auto i : _texturesToAdd)
    {
        osg::Image* src = i->getImage(0);
        width = osg::maximum(src->s(), width);
        height = osg::maximum(src->t(), height);

        //// upgrade pixelformat/datatype as we go:
        //if (pixelFormat == GL_RED && src->getPixelFormat() != GL_RED)
        //    pixelFormat = src->getPixelFormat();
        //if (pixelFormat == GL_RGB && src->getPixelFormat() == GL_RGBA)
        //    pixelFormat = GL_RGBA;
        //if (dataType == GL_FLOAT && src->getDataType() != GL_FLOAT)
        //    dataType = src->getDataType();
    }

    //GLenum internalFormat =
    //    pixelFormat == GL_RED ? GL_R16F :
    //    pixelFormat == GL_RG ? GL_RG8 :
    //    pixelFormat == GL_RGB ? GL_RGB8 :
    //    pixelFormat == GL_RGBA ? GL_RGBA8 :
    //    GL_RGBA8;

    // round to a power of 2 so we can properly mipmap/compress the textures
    width = nextPowerOf2(width);
    height = nextPowerOf2(height);

    Texture* tex = new Texture();

    tex->_image = new osg::Image();
    tex->_image->allocateImage(width, height, _texturesToAdd.size(), pixelFormat, dataType);
    tex->_image->setInternalTextureFormat(internalFormat);

    ImageUtils::PixelWriter write(tex->_image.get());
    osg::Vec4 value;

    for(auto& i : _texturesToAdd)
    {
        int layer = _textureLayers[i.get()];

        ImageUtils::PixelReader read(i->getImage(0));
        
        ImageUtils::ImageIterator iter(write);

        iter.forEachPixel([&]() {
            read(value, iter.u(), iter.v());
            write(value, iter.s(), iter.t(), layer);
        });
    }

    return tex;
}

namespace
{
    template<typename T> void append(T* dest, const osg::Array* src, unsigned numVerts)
    {
        if (src == NULL)
        {
            dest->reserveArray(dest->size() + numVerts);
            for(unsigned i=0; i<numVerts; ++i)
                dest->push_back(typename T :: ElementDataType ());
        }
        else if (src->getBinding() == osg::Array::BIND_PER_VERTEX)
        {
            dest->reserveArray(dest->size() + numVerts); //src->getNumElements());
            const T* src_typed = static_cast<const T*>(src);
            std::copy(src_typed->begin(), src_typed->end(), std::back_inserter(*dest));

            // pad out to match
            if (src->getNumElements() < numVerts)
                for(unsigned i=0; i<numVerts-src->getNumElements(); ++i)
                    dest->push_back(typename T :: ElementDataType ());

        }
        else if (src->getBinding() == osg::Array::BIND_OVERALL)
        {
            dest->reserveArray(dest->size() + numVerts);
            const T* src_typed = static_cast<const T*>(src);
            for(unsigned i=0; i<numVerts; ++i)
                dest->push_back((*src_typed)[0]);
        }
    }
}

bool
GeometryCloud::pushStateSet(osg::Node& node)
{
    osg::StateSet* stateset = node.getStateSet();
    if (stateset)
    {
        osg::Texture* tex = dynamic_cast<osg::Texture*>(
            stateset->getTextureAttribute(0, osg::StateAttribute::TEXTURE));

        if (tex)
        {
            _textureStack.push_back(tex);
            
            AtlasIndexLUT::iterator i = _textureLayers.find(tex);
            if (i == _textureLayers.end())
            {
                _texturesToAdd.push_back(tex);
                _textureLayers[tex] = _texLayer++;

            }

            return true;
        }
    }
    return false;
}

void
GeometryCloud::popStateSet()
{
    _textureStack.pop_back();
}

void
GeometryCloud::apply(osg::Node& node)
{
    bool pushed = pushStateSet(node);
    traverse(node);
    if (pushed) popStateSet();
}

void
GeometryCloud::apply(osg::Transform& node)
{
    osg::Matrix m = _matrixStack.empty() ? osg::Matrix() : _matrixStack.back();
    node.computeLocalToWorldMatrix(m, this);
    _matrixStack.push_back(m);
    traverse(node);
    _matrixStack.pop_back();
}

void
GeometryCloud::apply(osg::Geometry& node)
{
    bool pushed = pushStateSet(node);

    // offset of verts within this model:
    unsigned globalOffset = _vertexOffsets.empty() ? 0 : _vertexOffsets.back();

    // offset of verts local to this geometry:
    unsigned localOffset = _verts->size() - globalOffset;

    unsigned size = node.getVertexArray()->getNumElements();

    if (!_matrixStack.empty())
    {
        osg::Vec3Array* nodeVerts = dynamic_cast<osg::Vec3Array*>(node.getVertexArray());
        if (nodeVerts)
        {
            for(unsigned i=0; i<nodeVerts->size(); ++i)
            {
                (*nodeVerts)[i] = (*nodeVerts)[i] * _matrixStack.back();
            }
        }
    }
    //TODO: transform normals too if necessary

    append(_verts, node.getVertexArray(), size);
    append(_normals, node.getNormalArray(), size);
    append(_colors, node.getColorArray(), size);

    unsigned numTexCoords = 0;
    osg::Vec2Array* texcoords = dynamic_cast<osg::Vec2Array*>(node.getTexCoordArray(0));
    if (texcoords)
    {
        numTexCoords = texcoords->size();

        // find the current texture in the atlas
        int layer = -1;
        if (!_textureStack.empty())
        {
            AtlasIndexLUT::iterator i = _textureLayers.find(_textureStack.back());
            if (i != _textureLayers.end())
            {
                layer = i->second;
            }
        }

        _texcoords->reserve(_texcoords->size() + texcoords->size());
        for(int i=0; i<texcoords->size(); ++i)
        {
            _texcoords->push_back(osg::Vec3(
                (*texcoords)[i].x(),
                (*texcoords)[i].y(),
                layer));
        }
    }

    // pad out
    if (numTexCoords < size)
    {
        for(unsigned i=0; i < size - numTexCoords; ++i)
        {
            _texcoords->push_back(osg::Vec3(0,0,-2));
        }
    }

    for(unsigned i=0; i < node.getNumPrimitiveSets(); ++i)
    {
        osg::DrawElements* de = dynamic_cast<osg::DrawElements*>(node.getPrimitiveSet(i));
        if (de)
        {
            for(unsigned k=0; k<de->getNumIndices(); ++k)
            {
                int index = de->getElement(k);
                // by using a "model-local" offset here, we can use UShort even
                // if our vertex array size exceeds 65535 by storing the 
                // baseVertex in our DrawElements structure
                _primset->addElement(localOffset + index);
                ++_numElements;
            }
        }
    }

    if (pushed) popStateSet();
}

void
GeometryCloud::draw(osg::RenderInfo& ri)
{
    _geom->draw(ri);
}

bool
GeometryCloud::getDrawCommand(unsigned i, DrawElementsIndirectCommand& cmd) const
{
    if (i <= getNumDrawCommands())
    {
        cmd.count = _elementCounts[i];       // how many indices comprise this draw command
        cmd.instanceCount = 0;               // will be assigned by the Cull/Sort shader
        cmd.firstIndex = _elementOffsets[i]; // not used by us...or is it
        cmd.baseVertex = _vertexOffsets[i];  // offset to add to element indices (nice, lets us use USHORT)
        cmd.baseInstance = 0;                // will be assigned by the Cull/Sort shader
        return true;
    }
    return false;
}
