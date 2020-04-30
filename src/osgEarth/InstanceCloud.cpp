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
#include <osg/Program>
#include <osg/GLExtensions>

#ifndef GL_DYNAMIC_STORAGE_BIT
#define GL_DYNAMIC_STORAGE_BIT 0x0100
#endif

using namespace osgEarth;

#undef LC
#define LC "[InstanceCloud] "

//...................................................................

#define BINDING_COMMAND_BUFFER 0
#define BINDING_RENDER_BUFFER 1
#define BINDING_POINTS_BUFFER 2

// pre OSG-3.6 support
#ifndef GL_DRAW_INDIRECT_BUFFER
#define GL_DRAW_INDIRECT_BUFFER 0x8F3F
#endif

InstanceCloud::InstancingData::InstancingData() :
    commands(NULL),
    points(NULL),
    commandBuffer(-1),
    pointsBuffer(-1),
    renderBuffer(-1),
    numTilesAllocated(0u),
    ssboOffsetAlignment(-1)
{
    // polyfill for pre-OSG 3.6 support
    osg::setGLExtensionFuncPtr(_glBufferStorage, "glBufferStorage", "glBufferStorageARB");
}

InstanceCloud::InstancingData::~InstancingData()
{
    if (commands)
    {
        delete [] commands;
        commands = NULL;
    }
}

void
InstanceCloud::InstancingData::allocateGLObjects(osg::State* state, unsigned numTiles)
{
    if (numTilesAllocated < numTiles)
    {
        OE_DEBUG << LC << "Reallocate from " << numTilesAllocated << " to " << numTiles << " tiles" << std::endl;

        releaseGLObjects(state);

        osg::GLExtensions* ext = state->get<osg::GLExtensions>();

        numTilesAllocated = numTiles;

        commands = new DrawElementsIndirectCommand[numTilesAllocated];
        for(unsigned i=0; i<numTilesAllocated; ++i)
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

        ext->glGenBuffers(1, &commandBuffer);
        ext->glBindBuffer(GL_SHADER_STORAGE_BUFFER, commandBuffer);
        _glBufferStorage(
            GL_SHADER_STORAGE_BUFFER,
            commandBufferSize,
            NULL,                    // uninitialized memory
            GL_DYNAMIC_STORAGE_BIT); // so we can reset each frame
        
        // Buffer for the output data (culled points, written by compute shader)
        // Align properly to satisfy glBindBufferRange
        renderBufferTileSize = align(numInstances*instanceSize, (GLuint)ssboOffsetAlignment);

        OE_DEBUG << "NumInstances="<<numInstances<< ", renderBufferTileSize=" << renderBufferTileSize << ", cmdBufferSize=" << commandBufferSize << std::endl;

        ext->glGenBuffers(1, &renderBuffer);
        ext->glBindBuffer(GL_SHADER_STORAGE_BUFFER, renderBuffer);
        _glBufferStorage(
            GL_SHADER_STORAGE_BUFFER, 
            numTilesAllocated * renderBufferTileSize,
            NULL,   // uninitialized memory
            0);     // only GPU will write to this buffer
    }
}

void
InstanceCloud::InstancingData::releaseGLObjects(osg::State* state) const
{
    if (state)
    {
        osg::GLExtensions* ext = state->get<osg::GLExtensions>();

        if (commands)
        {
            delete[] commands;
            commands = NULL;
        }

        if (commandBuffer >= 0)
            ext->glDeleteBuffers(1, &commandBuffer);

        if (renderBuffer >= 0)
            ext->glDeleteBuffers(1, &renderBuffer);
    }
    else
    {
        // wut?
    }
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
InstanceCloud::setGeometry(osg::Geometry* geom)
{
    _geom = geom;

    if (geom)
    {
        Installer installer(&_data);
        geom->accept(installer);
    }
}

//void
//InstanceCloud::setPositions(osg::Vec4Array* value)
//{
//    _data.points = value;
//}

void
InstanceCloud::setNumInstances(unsigned x, unsigned y)
{
    // Enforce an even number of instances. For whatever reason that I cannot
    // figure out today, an odd number causes the shader to freak out.
    _data.numX = (x & 0x01) ? x-1 : x;
    _data.numY = (y & 0x01) ? y-1 : y;
}

osg::BoundingBox
InstanceCloud::computeBoundingBox() const
{
    osg::BoundingBox box;
    float radius = _geom.valid() ? _geom->getBound().radius() : 0.0f;
    osg::Vec3 radius3(radius, radius, radius);
    if (_data.points)
    {
        for(int i=0; i<_data.points->size(); ++i)
        {
            const osg::Vec4& v = (*_data.points)[i];
            osg::Vec3 center(v.x()/v.w(), v.y()/v.w(), v.z()/v.w());
            osg::BoundingBox pbox(center-radius3, center+radius3);
            box.expandBy(pbox);
        }
    }
    return box;
}

void
InstanceCloud::allocateGLObjects(osg::RenderInfo& ri, unsigned numTiles)
{
    _data.allocateGLObjects(ri.getState(), numTiles);
}

void
InstanceCloud::preCull(osg::RenderInfo& ri)
{
    osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();

    // Reset all the instance counts to zero by copying the empty
    // prototype buffer to the GPU
    ext->glBindBuffer(GL_SHADER_STORAGE_BUFFER, _data.commandBuffer);

    ext->glBufferSubData(
        GL_SHADER_STORAGE_BUFFER, 
        0,
        _data.numTilesAllocated * sizeof(DrawElementsIndirectCommand),
        &_data.commands[0]);

    // Bind our SSBOs to their respective layout indices in the shader
    ext->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BINDING_COMMAND_BUFFER, _data.commandBuffer);

    ext->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BINDING_RENDER_BUFFER, _data.renderBuffer);
}

void
InstanceCloud::cullTile(osg::RenderInfo& ri, unsigned tileNum)
{
    osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();

    ext->glDispatchCompute(_data.numX, _data.numY, 1);
}

void
InstanceCloud::postCull(osg::RenderInfo& ri)
{
    osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();

    ext->glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_COMMAND_BARRIER_BIT);
}

void
InstanceCloud::drawTile(osg::RenderInfo& ri, unsigned tileNum)
{
    _data.tileToDraw = tileNum;
    _geom->draw(ri);
}


InstanceCloud::Renderer::Renderer(InstancingData* data) :
    _data(data)
{
    // pre-OSG3.6 support
    osg::setGLExtensionFuncPtr(_glDrawElementsIndirect, "glDrawElementsIndirect", "glDrawElementsIndirectARB");
}

void
InstanceCloud::Renderer::drawImplementation(osg::RenderInfo& ri, const osg::Drawable* drawable) const
{
    osg::State& state = *ri.getState();

    osg::GLExtensions* ext = state.get<osg::GLExtensions>();

    if (_data->tileToDraw == 0)
    {
        const osg::Geometry* geom = drawable->asGeometry();
        geom->drawVertexArraysImplementation(ri);

        // TODO: support multiple primtsets....?
        osg::GLBufferObject* ebo = geom->getPrimitiveSet(0)->getOrCreateGLBufferObject(state.getContextID());
        state.bindElementBufferObject(ebo);

        ext->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BINDING_COMMAND_BUFFER, _data->commandBuffer);

        ext->glBindBuffer(GL_DRAW_INDIRECT_BUFFER, _data->commandBuffer);
    }

    // activate the "nth" tile in the render buffer:
    ext->glBindBufferRange(
        GL_SHADER_STORAGE_BUFFER, 
        BINDING_RENDER_BUFFER,
        _data->renderBuffer, 
        _data->tileToDraw * _data->renderBufferTileSize,
        _data->renderBufferTileSize);

    _glDrawElementsIndirect(
        _data->mode,
        _data->dataType,
        (const void*)(_data->tileToDraw * sizeof(DrawElementsIndirectCommand)) );
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

        osg::PrimitiveSet* p = geom->getPrimitiveSet(0);
        _data->numIndices = p->getNumIndices();
        _data->mode = p->getMode();
        _data->dataType = p->getDrawElements()->getDataType();
    }
}
