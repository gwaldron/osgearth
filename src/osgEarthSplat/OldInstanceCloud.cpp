/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include <osgEarthSplat/OldInstanceCloud>
#include <osgEarth/ShaderLoader>
#include <osgEarth/Math>
#include <osgEarth/Registry>
#include <osg/Program>
#include <osg/GLExtensions>
#include <osg/GraphicsContext>
#include <osgUtil/Optimizer>
#include <iterator>

#ifndef GL_DYNAMIC_STORAGE_BIT
#define GL_DYNAMIC_STORAGE_BIT 0x0100
#endif

using namespace osgEarth;
using namespace osgEarth::Splat;

#undef LC
#define LC "[OldInstanceCloud] "

#define BINDING_COMMAND_BUFFER 0
#define BINDING_RENDER_BUFFER 1
#define BINDING_POINTS_BUFFER 2

// pre OSG-3.6 support
#ifndef GL_DRAW_INDIRECT_BUFFER
#define GL_DRAW_INDIRECT_BUFFER 0x8F3F
#endif

LegacyInstanceCloud::InstancingData::InstancingData() :
    commands(NULL),
    points(NULL),
    numTilesAllocated(0u),
    ssboOffsetAlignment(-1)
{
    // polyfill for pre-OSG 3.6 support
    osg::setGLExtensionFuncPtr(_glBufferStorage, "glBufferStorage", "glBufferStorageARB");
}

LegacyInstanceCloud::InstancingData::~InstancingData()
{
    if (commands)
    {
        delete[] commands;
        commands = NULL;
    }
    releaseGLObjects(NULL);
}

void
LegacyInstanceCloud::InstancingData::allocateGLObjects(osg::State* state, unsigned numTiles)
{
    if (numTilesAllocated < numTiles || commands == nullptr)
    {
        OE_DEBUG << LC << "Reallocate from " << numTilesAllocated << " to " << numTiles << " tiles" << std::endl;

        releaseGLObjects(state);

        // this is OK b/c there should only be one LegacyInstanceCloud per context id..
        // save it for release time.
        contextID = state->getContextID();

        osg::GLExtensions* ext = state->get<osg::GLExtensions>();

        numTilesAllocated = numTiles;

        commands = new DrawElementsIndirectCommand[numTilesAllocated];
        for (unsigned i = 0; i < numTilesAllocated; ++i)
        {
            commands[i].count = numIndices;
            commands[i].instanceCount = 0;
            commands[i].firstIndex = 0;
            commands[i].baseVertex = 0;
            commands[i].baseInstance = 0;
        }

        if (ssboOffsetAlignment < 0)
        {
            glGetIntegerv(GL_SHADER_STORAGE_BUFFER_OFFSET_ALIGNMENT, &ssboOffsetAlignment);
            OE_DEBUG << "SSBO ALIGN = " << ssboOffsetAlignment << std::endl;
        }

        GLuint numInstances = numX * numY;
        GLuint instanceSize = 48; // see RenderData in shader (must be a multiple of a vec4)

        commandBufferSize = numTilesAllocated * sizeof(DrawElementsIndirectCommand);
        commandBufferSize = align(commandBufferSize, ssboOffsetAlignment);

#if 1
        commandBuffer = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, *state);
        commandBuffer->bind();
        commandBuffer->debugLabel("LegacyInstanceCloud");
        commandBuffer->bufferStorage(
            commandBufferSize,
            nullptr,                 // uninitialized memory
            GL_DYNAMIC_STORAGE_BIT); // so we can reset each frame
        commandBuffer->unbind();
#else
        commandBuffer = new GLBuffer();
        ext->glGenBuffers(1, &commandBuffer->_handle);
        ext->glBindBuffer(GL_SHADER_STORAGE_BUFFER, commandBuffer->_handle);
        _glBufferStorage(
            GL_SHADER_STORAGE_BUFFER,
            commandBufferSize,
            NULL,                    // uninitialized memory
            GL_DYNAMIC_STORAGE_BIT); // so we can reset each frame
        state->getGraphicsContext()->add(new GLBufferReleaser(commandBuffer.get()));
#endif

        // Buffer for the output data (culled points, written by compute shader)
        // Align properly to satisfy glBindBufferRange
        renderBufferTileSize = align(numInstances*instanceSize, (GLuint)ssboOffsetAlignment);

        OE_DEBUG << "NumInstances=" << numInstances << ", renderBufferTileSize=" << renderBufferTileSize << ", cmdBufferSize=" << commandBufferSize << std::endl;

#if 1
        //renderBuffer = new GLBuffer(GL_SHADER_STORAGE_BUFFER, *state, "oe.ic.renderbuffer");
        renderBuffer = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, *state);
        renderBuffer->bind();
        renderBuffer->debugLabel("LegacyInstanceCloud");
        renderBuffer->bufferStorage(
            numTilesAllocated * renderBufferTileSize,
            nullptr,   // uninitialized memory
            0);        // only GPU will write to this buffer
        renderBuffer->unbind();
#else
        renderBuffer = new GLBuffer();
        ext->glGenBuffers(1, &renderBuffer->_handle);
        ext->glBindBuffer(GL_SHADER_STORAGE_BUFFER, renderBuffer->_handle);
        _glBufferStorage(
            GL_SHADER_STORAGE_BUFFER,
            numTilesAllocated * renderBufferTileSize,
            NULL,   // uninitialized memory
            0);     // only GPU will write to this buffer
        state->getGraphicsContext()->add(new GLBufferReleaser(renderBuffer.get()));
#endif
    }
}

void
LegacyInstanceCloud::InstancingData::releaseGLObjects(osg::State* state) const
{
    if (commands)
    {
        delete[] commands;
        commands = NULL;
    }

    commandBuffer = NULL;
    renderBuffer = NULL;
    numTilesAllocated = 0;
}

LegacyInstanceCloud::LegacyInstanceCloud()
{
    //nop
}

void
LegacyInstanceCloud::releaseGLObjects(osg::State* state) const
{
    _data.releaseGLObjects(state);
    _geom->releaseGLObjects(state);
}

void
LegacyInstanceCloud::setGeometry(osg::Geometry* geom)
{
    _geom = geom;

    if (geom)
    {
        Installer installer(&_data);
        geom->accept(installer);
    }
}

//void
//LegacyInstanceCloud::setPositions(osg::Vec4Array* value)
//{
//    _data.points = value;
//}

void
LegacyInstanceCloud::setNumInstances(unsigned x, unsigned y)
{
    // Enforce an even number of instances. For whatever reason that I cannot
    // figure out today, an odd number causes the shader to freak out.
    _data.numX = (x & 0x01) ? x + 1 : x;
    _data.numY = (y & 0x01) ? y + 1 : y;
}

osg::BoundingBox
LegacyInstanceCloud::computeBoundingBox() const
{
    osg::BoundingBox box;
    float radius = _geom.valid() ? _geom->getBound().radius() : 0.0f;
    osg::Vec3 radius3(radius, radius, radius);
    if (_data.points)
    {
        for (int i = 0; i < _data.points->size(); ++i)
        {
            const osg::Vec4& v = (*_data.points)[i];
            osg::Vec3 center(v.x() / v.w(), v.y() / v.w(), v.z() / v.w());
            osg::BoundingBox pbox(center - radius3, center + radius3);
            box.expandBy(pbox);
        }
    }
    return box;
}

void
LegacyInstanceCloud::allocateGLObjects(osg::RenderInfo& ri, unsigned numTiles)
{
    _data.allocateGLObjects(ri.getState(), numTiles);
}

void
LegacyInstanceCloud::preCull(osg::RenderInfo& ri)
{
    if (!_data.commandBuffer)
    {
        return;
    }

    osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();

    // Reset all the instance counts to zero by copying the empty
    // prototype buffer to the GPU
#if 1
    _data.commandBuffer->bind();
#else
    ext->glBindBuffer(GL_SHADER_STORAGE_BUFFER, _data.commandBuffer->_handle);
#endif

    ext->glBufferSubData(
        GL_SHADER_STORAGE_BUFFER,
        0,
        _data.numTilesAllocated * sizeof(DrawElementsIndirectCommand),
        &_data.commands[0]);

    // Bind our SSBOs to their respective layout indices in the shader
    ext->glBindBufferBase(
        GL_SHADER_STORAGE_BUFFER,
        BINDING_COMMAND_BUFFER,
        _data.commandBuffer->name());

    ext->glBindBufferBase(
        GL_SHADER_STORAGE_BUFFER,
        BINDING_RENDER_BUFFER,
        _data.renderBuffer->name());
}

void
LegacyInstanceCloud::cullTile(osg::RenderInfo& ri, unsigned tileNum)
{
    osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();

    ext->glDispatchCompute(_data.numX, _data.numY, 1);
}

void
LegacyInstanceCloud::postCull(osg::RenderInfo& ri)
{
    osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();

    ext->glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_COMMAND_BARRIER_BIT);
}

void
LegacyInstanceCloud::drawTile(osg::RenderInfo& ri, unsigned tileNum)
{
    _data.tileToDraw = tileNum;
    _geom->draw(ri);
}

void
LegacyInstanceCloud::endFrame(osg::RenderInfo& ri)
{
    // be a good citizen
    osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();
    ext->glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);
}

LegacyInstanceCloud::Renderer::Renderer(InstancingData* data) :
    _data(data)
{
    // pre-OSG3.6 support
    osg::setGLExtensionFuncPtr(_glDrawElementsIndirect, "glDrawElementsIndirect", "glDrawElementsIndirectARB");
}

void
LegacyInstanceCloud::Renderer::drawImplementation(osg::RenderInfo& ri, const osg::Drawable* drawable) const
{
    if (!_data->commandBuffer)
    {
        return;
    }

    osg::State& state = *ri.getState();

    osg::GLExtensions* ext = state.get<osg::GLExtensions>();

    if (_data->tileToDraw == 0)
    {
        const osg::Geometry* geom = drawable->asGeometry();

        geom->drawVertexArraysImplementation(ri);

        // TODO: support multiple primtsets....?
        osg::GLBufferObject* ebo = geom->getPrimitiveSet(0)->getOrCreateGLBufferObject(state.getContextID());
        state.bindElementBufferObject(ebo);

        ext->glBindBufferBase(
            GL_SHADER_STORAGE_BUFFER,
            BINDING_COMMAND_BUFFER,
            _data->commandBuffer->name());

        _data->commandBuffer->bind(GL_DRAW_INDIRECT_BUFFER);
        //ext->glBindBuffer(GL_DRAW_INDIRECT_BUFFER, _data->commandBuffer->_handle);
    }

    // activate the "nth" tile in the render buffer:
    ext->glBindBufferRange(
        GL_SHADER_STORAGE_BUFFER,
        BINDING_RENDER_BUFFER,
        _data->renderBuffer->name(),
        _data->tileToDraw * _data->renderBufferTileSize,
        _data->renderBufferTileSize);

    _glDrawElementsIndirect(
        _data->mode,
        _data->dataType,
        (const void*)(_data->tileToDraw * sizeof(DrawElementsIndirectCommand)));
}


LegacyInstanceCloud::Installer::Installer(InstancingData* data) :
    osg::NodeVisitor(TRAVERSE_ALL_CHILDREN),
    _data(data)
{
    _callback = new Renderer(data);
}

void
LegacyInstanceCloud::Installer::apply(osg::Drawable& drawable)
{
    osg::Geometry* geom = drawable.asGeometry();
    if (geom)
    {
        geom->setDrawCallback(_callback.get());
        geom->setCullingActive(false);

        osg::PrimitiveSet* p = geom->getPrimitiveSet(0);
        _data->numIndices = p->getNumIndices();
        _data->mode = p->getMode();
        _data->dataType = p->getDrawElements()->getDataType();
    }
}

LegacyInstanceCloud::ModelCruncher::ModelCruncher() :
    osg::NodeVisitor()
{
    setTraversalMode(TRAVERSE_ALL_CHILDREN);
    setNodeMaskOverride(~0);

    _geom = new osg::Geometry();
    _geom->setUseVertexBufferObjects(true);
    _geom->setUseDisplayList(false);

    _verts = new osg::Vec3Array();
    _geom->setVertexArray(_verts);

    _colors = new osg::Vec4Array(osg::Array::BIND_PER_VERTEX);
    _geom->setColorArray(_colors);

    _normals = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    _geom->setNormalArray(_normals);

    _texcoords = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    _geom->setTexCoordArray(7, _texcoords);

    _primset = new osg::DrawElementsUShort(GL_TRIANGLES);
    _geom->addPrimitiveSet(_primset);
}

void
LegacyInstanceCloud::ModelCruncher::add(osg::Node* node)
{
    // convert all primitive sets to GL_TRIANGLES
    osgUtil::Optimizer o;
    o.optimize(node, o.INDEX_MESH);

    // count total number of indices
    // todo

    // traverse the model, consolidating arrays and rewriting texture indices
    node->accept(*this);
}

namespace
{
    template<typename T> void append(T* dest, const osg::Array* src, int numVerts)
    {
        if (src->getBinding() == osg::Array::BIND_PER_VERTEX)
        {
            dest->reserveArray(dest->size() + src->getNumElements());
            const T* src_typed = static_cast<const T*>(src);
            std::copy(src_typed->begin(), src_typed->end(), std::back_inserter(*dest));
        }
        else if (src->getBinding() == osg::Array::BIND_OVERALL)
        {
            dest->reserveArray(dest->size() + numVerts);
            const T* src_typed = static_cast<const T*>(src);
            for (int i = 0; i < numVerts; ++i)
                dest->push_back((*src_typed)[0]);
        }
    }
}

void
LegacyInstanceCloud::ModelCruncher::finalize()
{
    _atlas = new osg::Texture2DArray();

    int s = -1, t = -1;
    for (unsigned i = 0; i < _imagesToAdd.size(); ++i)
    {
        osg::Image* image = _imagesToAdd[i].get();
        osg::ref_ptr<osg::Image> im;

        // make sure the texture array is POT - required now for mipmapping to work
        if (s < 0)
        {
            s = nextPowerOf2(image->s());
            t = nextPowerOf2(image->t());
            _atlas->setTextureSize(s, t, _imagesToAdd.size());
        }

        if (image->s() != s || image->t() != t)
        {
            ImageUtils::resizeImage(image, s, t, im);
        }
        else
        {
            im = image;
        }

        im->setInternalTextureFormat(GL_RGBA8);

        _atlas->setImage(i, im.get());
    }

    _atlas->setFilter(_atlas->MIN_FILTER, _atlas->NEAREST_MIPMAP_LINEAR);
    _atlas->setFilter(_atlas->MAG_FILTER, _atlas->LINEAR);
    _atlas->setWrap(_atlas->WRAP_S, _atlas->REPEAT);
    _atlas->setWrap(_atlas->WRAP_T, _atlas->REPEAT);
    _atlas->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());
    _atlas->setMaxAnisotropy(1.0);

    // Let the GPU do it since we only download this at startup
    //ImageUtils::generateMipmaps(tex);
    _atlas->setUseHardwareMipMapGeneration(true);

    _geom->getOrCreateStateSet()->setTextureAttribute(11, _atlas.get());
    _geom->getOrCreateStateSet()->addUniform(new osg::Uniform("oe_GroundCover_atlas", 11));
}

void
LegacyInstanceCloud::ModelCruncher::addTextureToAtlas(osg::Texture* t)
{
    osg::Texture2D* tex = dynamic_cast<osg::Texture2D*>(t);
    if (tex)
    {
        osg::ref_ptr<osg::Image> imageToAdd = tex->getImage();

        if (imageToAdd->getPixelFormat() != GL_RGBA)
        {
            imageToAdd = ImageUtils::convert(imageToAdd.get(), GL_RGBA, GL_UNSIGNED_BYTE); //firstImage->getDataType());
        }

        if (!_atlasLUT.empty())
        {
            //osg::Image* firstImage = _imagesToAdd[0].get();

        }

        int index = _atlasLUT.size();
        _atlasLUT[tex] = index;
        _imagesToAdd.push_back(imageToAdd.get());
    }
}

bool
LegacyInstanceCloud::ModelCruncher::pushStateSet(osg::Node& node)
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
                addTextureToAtlas(tex);
            }

            return true;
        }
    }
    return false;
}

void
LegacyInstanceCloud::ModelCruncher::popStateSet()
{
    _textureStack.pop_back();
}

void
LegacyInstanceCloud::ModelCruncher::apply(osg::Node& node)
{
    bool pushed = pushStateSet(node);
    traverse(node);
    if (pushed) popStateSet();
}

void
LegacyInstanceCloud::ModelCruncher::apply(osg::Geometry& node)
{
    bool pushed = pushStateSet(node);

    int offset = _verts->size();
    int size = node.getVertexArray()->getNumElements();

    append(_verts, node.getVertexArray(), size);
    append(_normals, node.getNormalArray(), size);
    append(_colors, node.getColorArray(), size);

    // find the current texture in the atlas
    int layer = 0;
    if (!_textureStack.empty())
    {
        AtlasIndexLUT::iterator i = _atlasLUT.find(_textureStack.back());
        if (i != _atlasLUT.end())
        {
            layer = i->second;
        }
    }

    osg::Vec2Array* texcoords = dynamic_cast<osg::Vec2Array*>(node.getTexCoordArray(0));
    if (texcoords)
    {
        // find the current texture in the atlas
        int layer = 0;
        if (!_textureStack.empty())
        {
            AtlasIndexLUT::iterator i = _atlasLUT.find(_textureStack.back());
            if (i != _atlasLUT.end())
            {
                layer = i->second;
            }
        }

        _texcoords->reserve(_texcoords->size() + texcoords->size());
        for (int i = 0; i < texcoords->size(); ++i)
        {
            _texcoords->push_back(osg::Vec3(
                (*texcoords)[i].x(),
                (*texcoords)[i].y(),
                layer));
        }
    }

    for (unsigned i = 0; i < node.getNumPrimitiveSets(); ++i)
    {
        osg::DrawElements* de = dynamic_cast<osg::DrawElements*>(node.getPrimitiveSet(i));
        if (de)
        {
            for (unsigned k = 0; k < de->getNumIndices(); ++k)
            {
                int index = de->getElement(k);
                _primset->addElement(offset + index);
            }
        }
    }

    if (pushed) popStateSet();
}
