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
#include "Chonk"
#include <osgUtil/Optimizer>
#include <osgUtil/MeshOptimizers>

#undef LC
#define LC "[Chonk] "

using namespace osgEarth;

namespace
{
    /**
     * Visitor that traverses a graph and generates a Chonk,
     * storing any discovered textures in the provided arena.
     */
    struct Ripper : public osg::NodeVisitor
    {
        Chonk::Ptr _result;
        TextureArena* _textures;
        std::stack<ChonkMaterial::Ptr> _materialStack;
        std::stack<osg::Matrix> _transformStack;
        std::unordered_map<osg::Texture*, ChonkMaterial::Ptr> _materialLUT;

        Ripper(TextureArena* textures) :
            _textures(textures)
        {
            setTraversalMode(TRAVERSE_ALL_CHILDREN);
            setNodeMaskOverride(~0);

            _materialStack.push(ChonkMaterial::create());
            _transformStack.push(osg::Matrix());

            _result = Chonk::create();
        }

        std::stack<int> _materialIndexStack;

        // record materials, and return true if we pushed one.
        bool pushStateSet(osg::StateSet* stateset)
        {
            if (stateset)
            {
                auto& material = _materialStack.top();

                osg::Texture* tex = dynamic_cast<osg::Texture*>(
                    stateset->getTextureAttribute(0, osg::StateAttribute::TEXTURE));

                if (tex && tex->getImage(0))
                {
                    auto i = _materialLUT.find(tex);

                    if (i == _materialLUT.end())
                    {
                        Texture::Ptr t = Texture::create();
                        t->_image = tex->getImage(0);
                        t->_uri = t->_image->getFileName();
                        t->_label = "Chonk texture";
                        
                        material->albedo = _textures->add(t);
                        if (material->albedo >= 0)
                        {
                            _materialLUT[tex] = material;
                        }
                    }
                    else
                    {
                        material = i->second;
                    }
                }
                _materialStack.push(material);
                return true; // yes push
            }
            return false; // no push
        }

        void popStateSet()
        {
            _materialStack.pop();
        }

        void apply(osg::Node& node)
        {
            bool pushed = pushStateSet(node.getStateSet());
            traverse(node);
            if (pushed) popStateSet();
        }

        void apply(osg::Transform& node)
        {
            osg::Matrix m = _transformStack.empty() ? osg::Matrix() : _transformStack.top();
            node.computeLocalToWorldMatrix(m, this);
            _transformStack.push(m);
            apply(static_cast<osg::Group&>(node));
            _transformStack.pop();
        }

        void apply(osg::Geometry& node)
        {
            bool pushed = pushStateSet(node.getStateSet());

            unsigned numVerts = node.getVertexArray()->getNumElements();

            _result->_vbo_store.reserve(_result->_vbo_store.size() + numVerts);
            unsigned vbo_offset = _result->_vbo_store.size();

            auto verts = dynamic_cast<osg::Vec3Array*>(node.getVertexArray());
            auto colors = dynamic_cast<osg::Vec4Array*>(node.getColorArray());
            auto normals = dynamic_cast<osg::Vec3Array*>(node.getNormalArray());
            auto uvs = dynamic_cast<osg::Vec2Array*>(node.getTexCoordArray(0));

            auto& material = _materialStack.top();

            for (unsigned i = 0; i < numVerts; ++i)
            {
                Chonk::VertexGPU v;

                if (verts) {
                    v.position = (*verts)[i] * _transformStack.top();
                }
                
                if (colors) {
                    if (colors->getBinding() == colors->BIND_PER_VERTEX)
                        //v.color = (*colors)[i];
                    v.color.set(
                        (char)((*colors)[i].r()*255.0f),
                        (char)((*colors)[i].g()*255.0f),
                        (char)((*colors)[i].b()*255.0f),
                        (char)((*colors)[i].a()*255.0f));
                    else
                        //v.color.set(1, 1, 1, 1);
                          v.color.set(255, 255, 255, 255);
                }
                else {
//                    v.color.set(1, 1, 1, 1);
                    v.color.set(255, 255, 255, 255);
                }

                if (normals) {
                    if (normals->getBinding() == normals->BIND_PER_VERTEX)
                        v.normal = (*normals)[i];
                    else
                        v.normal = (*normals)[0];
                }
                else {
                    v.normal.set(0, 0, 1);
                }

                if (uvs && uvs) {
                    if (uvs->getBinding() == normals->BIND_PER_VERTEX)
                        v.uv = (*uvs)[i];
                    else
                        v.uv = (*uvs)[0];
                }

                v.albedo = material ? material->albedo : -1;

                _result->_vbo_store.emplace_back(std::move(v));

                _result->_materials.push_back(_materialStack.top());
            }

            // assemble the elements set
            for (unsigned i = 0; i < node.getNumPrimitiveSets(); ++i)
            {
                osg::DrawElements* de = dynamic_cast<osg::DrawElements*>(node.getPrimitiveSet(i));
                if (de)
                {
                    for (unsigned k = 0; k < de->getNumIndices(); ++k)
                    {
                        int index = de->getElement(k);
                        // by using a "model-local" offset here, we can use UShort even
                        // if our vertex array size exceeds 65535 by storing the 
                        // baseVertex in our DrawElements structure
                        _result->_ebo_store.push_back(vbo_offset + index);
                    }
                }
            }

            if (pushed) popStateSet();
        }
    };
}

ChonkMaterial::Ptr
ChonkMaterial::create()
{
    return Ptr(new ChonkMaterial);
}

Chonk::Ptr
Chonk::create()
{
    return Ptr(new Chonk);
}

Chonk::Chonk()
{
    _gs.resize(1); // todo
}

const DrawElementsIndirectBindlessCommandNV&
Chonk::getOrCreateCommand(osg::State& state) const
{
    auto& gs = _gs[state.getContextID()];

    if (gs.vbo == nullptr)
    {
        gs.vbo = GLBuffer::create(GL_ARRAY_BUFFER_ARB, state, "Chonk VBO");
        gs.vbo->bufferStorage(
            _vbo_store.size() * sizeof(VertexGPU),
            _vbo_store.data(),
            0); // permanent
        gs.vbo->makeResident();

        gs.ebo = GLBuffer::create(GL_ELEMENT_ARRAY_BUFFER_ARB, state, "Chonk EBO");
        gs.ebo->bufferStorage(
            _ebo_store.size() * sizeof(GLushort),
            _ebo_store.data(),
            0); // permanent
        gs.ebo->makeResident();

        // describe the draw command
        gs.command.cmd.count = _ebo_store.size();

        gs.command.indexBuffer.address = gs.ebo->address();
        gs.command.indexBuffer.length = gs.ebo->size();

        gs.command.vertexBuffer.address = gs.vbo->address();
        gs.command.vertexBuffer.length = gs.vbo->size();
    }

    return gs.command;
}


ChonkFactory::ChonkFactory(TextureArena* textures) :
    _textures(textures)
{
    //nop
}

Chonk::Ptr
ChonkFactory::create(osg::Node* node)
{
    // convert all primitive sets to GL_TRIANGLES
    osgUtil::Optimizer o;
    o.optimize(node, o.INDEX_MESH);

    // Reorder indices for optimal cache usage.
    // DO NOT do this if alignment is set; using alignment
    // implies the verts are in a specific order for a good reason, which
    // is usually because the shader relies on gl_VertexID.
    //if (alignment == 0u)
    //{
    //    osgUtil::VertexCacheVisitor vcv;
    //    node->accept(vcv);
    //    vcv.optimizeVertices();
    //}

    // rip geometry and textures into a new Asset object
    Ripper ripper(_textures.get());
    node->accept(ripper);
    
    // put under management and prep for GPU upload
    _chonks.emplace_back(ripper._result);

    return ripper._result;
}


#ifndef GL_VERTEX_ATTRIB_ARRAY_UNIFIED_NV
#define GL_VERTEX_ATTRIB_ARRAY_UNIFIED_NV 0x8F1E
#define GL_ELEMENT_ARRAY_UNIFIED_NV 0x8F1F
#endif

#ifndef GL_INT64_NV
#define GL_INT64_NV 0x140E
#define GL_UNSIGNED_INT64_NV 0x140F
#endif

namespace {
    struct VADef {
        GLint size;
        GLenum type;
        GLboolean normalize;
        GLint offset;
    };
}

ChonkDrawable::ChonkDrawable() :
    osg::Drawable()
{
    //nop
}

void
ChonkDrawable::add(Chonk::Ptr value)
{
    if (value)
    {
        ScopedMutexLock lock(_m);

        _chonks.push_back(value);

        for (unsigned i = 0; i < _gs.size(); ++i)
            _gs[i]._dirty = true;
    }
}

void
ChonkDrawable::GCState::update(
    const std::vector<Chonk::Ptr>& chonks,
    osg::State& state)
{
    if (_commandBuf == nullptr)
    {
        _vao = GLVAO::create(state, "ChonkDrawable");

        _commandBuf = GLBuffer::create(GL_DRAW_INDIRECT_BUFFER, state, "ChonkDrawable");

        osg::setGLExtensionFuncPtr(
            _glMultiDrawElementsIndirectBindlessNV,
            "glMultiDrawElementsIndirectBindlessNV");
    }

    std::vector<DrawElementsIndirectBindlessCommandNV> temp;

    for (auto chonk : chonks)
    {
        temp.push_back(chonk->getOrCreateCommand(state));
    }

    _commandBuf->bind();
    _commandBuf->uploadData(
        temp.size() * sizeof(DrawElementsIndirectBindlessCommandNV),
        temp.data());

    _numCommands = chonks.size();

    _dirty = false;
}

void
ChonkDrawable::GCState::draw(osg::State& state)
{
    _vao->bind();

    _glMultiDrawElementsIndirectBindlessNV(
        GL_TRIANGLES,
        GL_UNSIGNED_SHORT,
        (const GLvoid*)0,
        _numCommands,
        sizeof(Chonk::VertexGPU),
        1);
    
    _vao->unbind();
}

void
ChonkDrawable::GCState::release()
{
    _vao = nullptr;
    _commandBuf = nullptr;
    _numCommands = 0u;
    _dirty = true;
}

void
ChonkDrawable::drawImplementation(osg::RenderInfo& ri) const
{
    osg::State& state = *ri.getState();
    GCState& gs = _gs[ri.getContextID()];

    if (gs._dirty)
    {
        ScopedMutexLock lock(_m);
        gs.update(_chonks, *ri.getState());
    }

    if (_chonks.size() > 0)
    {
        gs.draw(*ri.getState());
    }
}

void
ChonkDrawable::resizeGLObjectBuffers(unsigned size)
{
    _gs.resize(size);
}

void
ChonkDrawable::releaseGLObjects(osg::State* state) const
{
    if (state)
        _gs[state->getContextID()].release();
    else
        _gs.setAllElementsTo(GCState());
}

GLVAO::Ptr
ChonkDrawable::GCState::createAndRecordVAO(osg::State& state)
{
    GLVAO::Ptr vao = GLVAO::create(state, "ChonkDrawable");

    void(GL_APIENTRY * gl_VertexAttribFormat)(GLuint, GLint, GLenum, GLboolean, GLuint);
    osg::setGLExtensionFuncPtr(gl_VertexAttribFormat, "glVertexAttribFormat");

    void(GL_APIENTRY * gl_VertexAttribIFormat)(GLuint, GLint, GLenum, GLuint);
    osg::setGLExtensionFuncPtr(gl_VertexAttribIFormat, "glVertexAttribIFormat");

    void(GL_APIENTRY * gl_VertexAttribLFormat)(GLuint, GLint, GLenum, GLuint);
    osg::setGLExtensionFuncPtr(gl_VertexAttribLFormat, "glVertexAttribLFormatNV");

    // start recording...
    vao->bind();

    // required in order to use BindlessNV extension
    glEnableClientState(GL_VERTEX_ATTRIB_ARRAY_UNIFIED_NV);
    glEnableClientState(GL_ELEMENT_ARRAY_UNIFIED_NV);

    const VADef formats[5] = {
        {3, GL_FLOAT,         GL_FALSE, offsetof(Chonk::VertexGPU, position)},
        {3, GL_FLOAT,         GL_FALSE, offsetof(Chonk::VertexGPU, normal)},
        {4, GL_UNSIGNED_BYTE, GL_TRUE,  offsetof(Chonk::VertexGPU, color)},
        {2, GL_FLOAT,         GL_FALSE, offsetof(Chonk::VertexGPU, uv)},
        {1, GL_INT,           GL_FALSE, offsetof(Chonk::VertexGPU, albedo)}
    };

    // configure the format of each vertex attribute in our structure.
    for (unsigned location = 0; location < 5; ++location)
    {
        const VADef& d = formats[location];
        if (d.type == GL_INT || d.type == GL_INT)
            gl_VertexAttribIFormat(location, d.size, d.type, d.offset);
        else
            gl_VertexAttribFormat(location, d.size, d.type, d.normalize, d.offset);
        vao->ext()->glVertexAttribBinding(location, 0);
        vao->ext()->glEnableVertexAttribArray(location);
    }

    // bind a "dummy buffer" that will record the stride, which is
    // simply the size of our vertex structure.
    vao->ext()->glBindVertexBuffer(0, 0, 0, sizeof(Chonk::VertexGPU));

    // Finish recording
    _vao->unbind();
    
    return vao;
}
