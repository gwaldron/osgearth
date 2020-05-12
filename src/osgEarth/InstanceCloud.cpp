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
#include <osgEarth/Math>
#include <osgEarth/Registry>
#include <osgEarth/Utils>
#include <osgEarth/ImageUtils>
#include <osgEarth/Math>
#include <osgEarth/GLUtils>
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

//...................................................................

// pre OSG-3.6 support
#ifndef GL_DRAW_INDIRECT_BUFFER
#define GL_DRAW_INDIRECT_BUFFER 0x8F3F
#endif

SSBO::SSBO() :
    _contextID(0),
    _handle(0),
    _allocatedSize(0),
    _bindingIndex(0)
{
    // polyfill for pre-OSG 3.6 support
    osg::setGLExtensionFuncPtr(_glBufferStorage, "glBufferStorage", "glBufferStorageARB");
}

void
SSBO::release() const
{
    if (_handle != 0u)
    {
        GLUtils::deleteGLBuffer(_contextID, _handle);
        //ext->glDeleteBuffers(1, &_handle);
    }

    _handle = 0u;
    _allocatedSize = 0u;
}

void
SSBO::bind(osg::GLExtensions* ext) const
{
    ext->glBindBuffer(GL_SHADER_STORAGE_BUFFER, _handle);
}

void
SSBO::bindLayout(osg::GLExtensions* ext) const
{
    ext->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, _bindingIndex, _handle);
}

void
InstanceCloud::CommandBuffer::allocate(
    GeometryCloud* geom,
    GLsizei alignment,
    osg::GLExtensions* ext)
{
    unsigned numCommands = geom->getNumDrawCommands();

    _requiredSize = numCommands * sizeof(DrawElementsIndirectCommand);

    if (_requiredSize > _allocatedSize)
    {
        _geom = geom;

        release();
        if (_buf) delete [] _buf;

        _buf = new DrawElementsIndirectCommand[numCommands];
        _allocatedSize = align(_requiredSize, alignment);

        ext->glGenBuffers(1, &_handle);
        ext->glBindBuffer(GL_SHADER_STORAGE_BUFFER, _handle);
        _glBufferStorage(GL_SHADER_STORAGE_BUFFER, _allocatedSize, NULL, GL_DYNAMIC_STORAGE_BIT);
        ext->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, _bindingIndex, _handle);
    }
}

void
InstanceCloud::CommandBuffer::reset(osg::GLExtensions* ext)
{
    // Initialize and blit to GPU
    for(unsigned i=0; i<_geom->getNumDrawCommands(); ++i)
    {
        _geom->getDrawCommand(i, _buf[i]);
    }

    ext->glBindBuffer(GL_SHADER_STORAGE_BUFFER, _handle);
    ext->glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, _requiredSize, _buf);
}

void
InstanceCloud::TileBuffer::allocate(unsigned numTiles, GLsizei alignment, osg::GLExtensions* ext)
{
    _requiredSize = numTiles * sizeof(Data);
    if (_requiredSize > _allocatedSize)
    {
        OE_INFO << "Allocating " << numTiles << " tiles" << std::endl;
        release();
        if (_buf) delete [] _buf;

        _buf = new Data[numTiles];
        _allocatedSize = align(_requiredSize, alignment);

        ext->glGenBuffers(1, &_handle);
        ext->glBindBuffer(GL_SHADER_STORAGE_BUFFER, _handle);
        _glBufferStorage(GL_SHADER_STORAGE_BUFFER, _allocatedSize, NULL, GL_DYNAMIC_STORAGE_BIT);  
        ext->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, _bindingIndex, _handle);
    }
}

void
InstanceCloud::TileBuffer::update(osg::GLExtensions* ext) const
{
    ext->glBindBuffer(GL_SHADER_STORAGE_BUFFER, _handle);
    ext->glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, _requiredSize, _buf);
}

void
InstanceCloud::InstanceBuffer::allocate(unsigned numInstances, GLsizei alignment, osg::GLExtensions* ext)
{
    _requiredSize = sizeof(HeaderData) + numInstances*sizeof(InstanceData);
    if (_requiredSize > _allocatedSize)
    {
        release();
        if (_buf) delete[] _buf;

        _buf = new HeaderData;
        _allocatedSize = align(_requiredSize, alignment);

        ext->glGenBuffers(1, &_handle);
        ext->glBindBuffer(GL_SHADER_STORAGE_BUFFER, _handle);
        _glBufferStorage(GL_SHADER_STORAGE_BUFFER, _allocatedSize, NULL, GL_DYNAMIC_STORAGE_BIT | GL_MAP_READ_BIT);  
        ext->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, _bindingIndex, _handle);
    }
    clear(ext);
}

void
InstanceCloud::InstanceBuffer::clear(osg::GLExtensions* ext)
{
    // Zero out the instance count.
    _buf[0]._count = 0u;

    // placeholders
    _buf[0]._padding[0] = 1;
    _buf[0]._padding[1] = 2;
    _buf[0]._padding[2] = 3;

    ext->glBindBuffer(GL_SHADER_STORAGE_BUFFER, _handle);
    ext->glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(unsigned), _buf);
}

void
InstanceCloud::InstanceBuffer::readHeader(osg::GLExtensions* ext)
{
    // Zero out the instance count.
    _buf[0]._count = 0u;

    // placeholders
    _buf[0]._padding[0] = 1;
    _buf[0]._padding[1] = 2;
    _buf[0]._padding[2] = 3;

    ext->glBindBuffer(GL_SHADER_STORAGE_BUFFER, _handle);
    ext->glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(unsigned), _buf);
}


InstanceCloud::InstancingData::InstancingData() :
    _numTilesAllocated(0u),
    _alignment((GLsizei)-1),
    _passUL((GLint)-1),
    _numX(0u),
    _numY(0u)
{
    // Layout bindings. Make sure these match the layout specs in the shaders.
    _commandBuffer._bindingIndex = 0;
    _instanceBuffer._bindingIndex = 1;
    _tileBuffer._bindingIndex = 2;
    _renderListBuffer._bindingIndex = 4;
}

InstanceCloud::InstancingData::~InstancingData()
{
    //nop
}

void
InstanceCloud::InstancingData::allocateGLObjects(osg::State* state, unsigned numTiles)
{
    osg::GLExtensions* ext = state->get<osg::GLExtensions>();

    if (_alignment < 0)
    {
        glGetIntegerv(GL_SHADER_STORAGE_BUFFER_OFFSET_ALIGNMENT, &_alignment);
        OE_DEBUG << "SSBO Alignment = " << _alignment << std::endl;
    }

    GLuint numInstances = numTiles * _numX * _numY;

    _commandBuffer.allocate(_geom.get(), _alignment, ext);
    _instanceBuffer.allocate(numInstances, _alignment, ext);
    _tileBuffer.allocate(numTiles, _alignment, ext);
    _renderListBuffer.allocate(numInstances, _alignment, ext);

    _numTilesAllocated = numTiles;
}

void
InstanceCloud::InstancingData::releaseGLObjects(osg::State* state) const
{
    _commandBuffer.release();
    _tileBuffer.release();
    _instanceBuffer.release();
    _renderListBuffer.release();
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
    GeometryValidator validator;
    geom->getGeometry()->accept(validator);

    _data._geom = geom;
    if (geom)
    {
        Installer installer(&_data);
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

void
InstanceCloud::allocateGLObjects(osg::RenderInfo& ri, unsigned numTiles)
{
    _data.allocateGLObjects(ri.getState(), numTiles);
}

void
InstanceCloud::newFrame(osg::RenderInfo& ri)
{
    osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();
    _data._commandBuffer.bindLayout(ext);
    _data._tileBuffer.bindLayout(ext);
    _data._instanceBuffer.bindLayout(ext);
    _data._renderListBuffer.bindLayout(ext);
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
}

void
InstanceCloud::generate_begin(osg::RenderInfo& ri)
{
    //nop
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

    // read back the instance count from the generator.
    // should be quick; it's only 4 bytes.
    ext->glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT);
    _data._instanceBuffer.readHeader(ext);
    _data._numInstancesGenerated = _data._instanceBuffer._buf->_count;

    OE_DEBUG << LC << "Generated " << _data._numInstancesGenerated << " instances." << std::endl;
}

void
InstanceCloud::cull(osg::RenderInfo& ri)
{
    osg::State& state = *ri.getState();

    osg::GLExtensions* ext = state.get<osg::GLExtensions>();

    // find the "pass" uniform first time in
    if (_data._passUL < 1)
    {
        unsigned passUName = osg::Uniform::getNameID("oe_pass");
        const osg::Program::PerContextProgram* pcp = state.getLastAppliedProgramObject();
        if (!pcp) return;
        _data._passUL = pcp->getUniformLocation(passUName);
    }

    // reset the command buffer in prep for culling and sorting
    _data._commandBuffer.reset(ext);

    // update with the newest tile matrices
    _data._tileBuffer.update(ext);

    // need the projection matrix for frustum culling.
    if (state.getUseModelViewAndProjectionUniforms())
        state.applyModelViewAndProjectionUniformsIfRequired();

    // first pass: cull
    ext->glUniform1i(_data._passUL, (int)0);
    ext->glDispatchCompute(_data._numInstancesGenerated, 1, 1);

    // second pass: sort
    ext->glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    ext->glUniform1i(_data._passUL, (int)1);
    ext->glDispatchCompute(_data._numInstancesGenerated, 1, 1);

    // synchronize in preparation for rendering
    ext->glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_COMMAND_BARRIER_BIT);
}

void
InstanceCloud::draw(osg::RenderInfo& ri)
{
    osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();
   _data._geom->draw(ri);
}


InstanceCloud::Renderer::Renderer(InstancingData* data) :
    _data(data)
{
    // polyfill pre-OSG3.6 support
    osg::setGLExtensionFuncPtr(
        _glMultiDrawElementsIndirect, 
        "glMultiDrawElementsIndirect", "glMultiDrawElementsIndirectARB");
}

void
InstanceCloud::Renderer::drawImplementation(osg::RenderInfo& ri, const osg::Drawable* drawable) const
{
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
        osg::GLBufferObject* ebo = geom->getPrimitiveSet(0)->getOrCreateGLBufferObject(state.getContextID());
        state.bindElementBufferObject(ebo);
    }

    // GL_DRAW_INDIRECT_BUFFER buffing binding is NOT part of VAO state (per spec)
    // so we have to bind it here.
    ext->glBindBuffer(
        GL_DRAW_INDIRECT_BUFFER,
        _data->_commandBuffer._handle);

    // engage
    _glMultiDrawElementsIndirect(
        GL_TRIANGLES,
        GL_UNSIGNED_SHORT,
        NULL,                                // in GL_DRAW_INDIRECT_BUFFER on GPU
        _data->_geom->getNumDrawCommands(),  // number of commands to execute
        0);                                  // stride=0, commands are tightly packed
}


InstanceCloud::Installer::Installer(InstancingData* data) :
    osg::NodeVisitor(TRAVERSE_ALL_CHILDREN),
    _data(data)
{
    _callback = new Renderer(data);
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

TextureAtlas::TextureAtlas() :
    osg::Texture2DArray()
{
    setFilter(MIN_FILTER, NEAREST_MIPMAP_LINEAR);
    setFilter(MAG_FILTER, LINEAR);
    setWrap(WRAP_S, REPEAT);
    setWrap(WRAP_T, REPEAT);
    setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());
    //setMaxAnisotropy(4.0);

    // Let the GPU do it since we only download this at startup
    setUseHardwareMipMapGeneration(true);
}

void
TextureAtlas::setSize(int s, int t)
{
    setTextureSize(s, t, 1);
}

void
TextureAtlas::addImage(osg::Image* image)
{
    if (!image)
        return;

    osg::ref_ptr<osg::Image> temp;

    // make sure the texture array is POT - required now for mipmapping to work
    int s = getTextureWidth();
    int t = getTextureHeight();
    int r = getTextureDepth();

    if (s == 0)
    {
        s = osgEarth::nextPowerOf2(image->s());
        t = osgEarth::nextPowerOf2(image->t());
        setTextureSize(s, t, r);
    }

    if(image->getPixelFormat() != GL_RGBA ||
       image->getDataType() != GL_UNSIGNED_BYTE)
    {
        image = ImageUtils::convert(image, GL_RGBA, GL_UNSIGNED_BYTE);
    }

    if (image->s() != s || image->t() != t)
    {
        ImageUtils::resizeImage(image, s, t, temp);
    }
    else
    {
        temp = image;
    }

    temp->setInternalTextureFormat(GL_RGBA8);
    setImage(r, temp.get());

    setTextureSize(s, t, r+1);
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
}

void
GeometryCloud::setAtlas(TextureAtlas* atlas)
{
    _atlas = atlas;
}

int
GeometryCloud::add(osg::Node* node, unsigned alignment)
{
    unsigned prevAtlasSize = _atlas->getTextureDepth();

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
    node->accept(*this);
    _elementCounts.push_back(_numElements);

    if (_atlas->getTextureDepth() > prevAtlasSize)
        return prevAtlasSize;
    else
        return -1;
}

namespace
{
    template<typename T> void append(T* dest, const osg::Array* src, unsigned numVerts)
    {
        if (src == NULL)
        {
            dest->reserveArray(dest->size() + numVerts);
            for(unsigned i=0; i<numVerts; ++i)
                dest->push_back(T :: ElementDataType ());
        }
        else if (src->getBinding() == osg::Array::BIND_PER_VERTEX)
        {
            dest->reserveArray(dest->size() + numVerts); //src->getNumElements());
            const T* src_typed = static_cast<const T*>(src);
            std::copy(src_typed->begin(), src_typed->end(), std::back_inserter(*dest));

            // pad out to match
            if (src->getNumElements() < numVerts)
                for(unsigned i=0; i<numVerts-src->getNumElements(); ++i)
                    dest->push_back(T :: ElementDataType ());

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
            
            AtlasIndexLUT::iterator i = _atlasLUT.find(tex);
            if (i == _atlasLUT.end())
            {
                _atlasLUT[tex] = _atlas->getTextureDepth();
                _atlas->addImage(tex->getImage(0));
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
            AtlasIndexLUT::iterator i = _atlasLUT.find(_textureStack.back());
            if (i != _atlasLUT.end())
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
