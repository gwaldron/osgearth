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
#include "Color"

#include <osgUtil/Optimizer>
#include <osgUtil/MeshOptimizers>

#undef LC
#define LC "[Chonk] "

using namespace osgEarth;

namespace
{

    const char* s_chonk_cull_compute_shader = R"(

#version 460
#extension GL_ARB_gpu_shader_int64 : enable

layout(local_size_x=1, local_size_y=1, local_size_z=1) in;

struct DrawElementsIndirectCommand
{
    uint count;
    uint instanceCount;
    uint firstIndex;
    uint baseVertex;
    uint baseInstance;
};

struct BindlessPtrNV
{
    uint index;
    uint reserved;
    uint64_t address;
    uint64_t length;
};

struct DrawElementsIndirectBindlessCommandNV
{
    DrawElementsIndirectCommand cmd;
    uint reserved;
    BindlessPtrNV indexBuffer;
    BindlessPtrNV vertexBuffer;
};

struct ChonkVariant
{
    vec4 bs;
    float min_pixel_size;
    float max_pixel_size;
    uint num_variants;
    uint total_num_commands; // global
};

struct Instance
{
    mat4 xform;
    uint first_variant_cmd_index;
    uint visible_mask;
    float _padding[2];
};

layout(binding=0) buffer OutputBuffer
{
    Instance output_instances[];
};

layout(binding=1) buffer Commands
{
    DrawElementsIndirectBindlessCommandNV commands[];
};

layout(binding=2) buffer ChonkVariants
{
    ChonkVariant chonks[];
};

layout(binding=3) buffer InputBuffer
{
    Instance input_instances[];
};

uniform vec3 oe_Camera;

void cull()
{
    const uint i = gl_GlobalInvocationID.x; // instance
    const uint variant = gl_GlobalInvocationID.y; // variant

    // initialize by clearing the visibility bit for this variant:
    atomicAnd(input_instances[i].visible_mask, ~(1<<variant));

    // bail if our chonk does not have this variant
    uint v = input_instances[i].first_variant_cmd_index + variant;
    if (variant >= chonks[v].num_variants)
        return;

    // transform the bounding sphere to a view-space bbox.
    vec4 center = vec4(chonks[v].bs.xyz, 1);
    float radius = chonks[v].bs.w;
    vec4 center_view = gl_ModelViewMatrix * (input_instances[i].xform * center);
    vec4 LL_view = center_view - vec4(radius, radius, radius, 0);
    vec4 UR_view = center_view + vec4(radius, radius, radius, 0);

    // transform the bbox to clip and see if it intersects the frustum:
    vec4 LL = gl_ProjectionMatrix * LL_view;
    LL.xyz /= LL.w;
    if (LL.x > 1.0 || LL.y > 1.0 || LL.z > 1.0)
        return;
    vec4 UR = gl_ProjectionMatrix * UR_view;
    UR.xyz /= UR.w;
    if (UR.x < -1.0 || UR.y < -1.0 || UR.z < -1.0)
        return;

    // OK, it is in view - now check pixel size on screen for this variant:
    vec2 dims = 0.5*(UR.xy-LL.xy)*oe_Camera.xy;
    float pixelSize = max(dims.x, dims.y);
    if (pixelSize < chonks[v].min_pixel_size)
        return;
    if (pixelSize > chonks[v].max_pixel_size)
        return;

    // Pass! Set the visibility bit for this variant:
    atomicOr(input_instances[i].visible_mask, (1 << variant));

    // Bump all baseInstances following this one:
    const uint cmd_count = chonks[v].total_num_commands;
    for(uint i=v+1; i<cmd_count; ++i)
        atomicAdd(commands[i].cmd.baseInstance, 1);
}

// Copies the visible instances to a compacted output buffer.
void compact()
{
    const uint i = gl_GlobalInvocationID.x; // instance
    const uint variant = gl_GlobalInvocationID.y; // variant
    
    if ((input_instances[i].visible_mask & (1 << variant)) == 0)
        return;

    uint v = input_instances[i].first_variant_cmd_index + variant;
    uint offset = commands[v].cmd.baseInstance;
    uint index = atomicAdd(commands[v].cmd.instanceCount, 1);
    output_instances[offset+index] = input_instances[i];
}

// Entry point.
uniform int oe_pass;
void main()
{
    if (oe_pass == 0)
        cull();
    else
        compact();
}

)";

    /**
     * Visitor that traverses a graph and generates a Chonk,
     * storing any discovered textures in the provided arena.
     */
    struct Ripper : public osg::NodeVisitor
    {
        Chonk& _result;
        TextureArena* _textures;
        std::stack<ChonkMaterial::Ptr> _materialStack;
        std::stack<osg::Matrix> _transformStack;
        std::unordered_map<osg::Texture*, ChonkMaterial::Ptr> _materialLUT;

        Ripper(Chonk& chonk, TextureArena* textures) :
            _result(chonk),
            _textures(textures)
        {
            setTraversalMode(TRAVERSE_ALL_CHILDREN);
            setNodeMaskOverride(~0);

            _materialStack.push(ChonkMaterial::create());
            _transformStack.push(osg::Matrix());
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

            _result._vbo_store.reserve(_result._vbo_store.size() + numVerts);
            unsigned vbo_offset = _result._vbo_store.size();

            auto verts = dynamic_cast<osg::Vec3Array*>(node.getVertexArray());
            auto colors = dynamic_cast<osg::Vec4Array*>(node.getColorArray());
            auto normals = dynamic_cast<osg::Vec3Array*>(node.getNormalArray());
            auto uvs = dynamic_cast<osg::Vec2Array*>(node.getTexCoordArray(0));

            auto& material = _materialStack.top();

            for (unsigned i = 0; i < numVerts; ++i)
            {
                Chonk::VertexGPU v;

                if (verts)
                {
                    v.position = (*verts)[i] * _transformStack.top();
                }
                
                if (colors)
                {
                    if (colors->getBinding() == colors->BIND_PER_VERTEX)
                        v.color = Color((*colors)[i]).asNormalizedRGBA();
                    else
                        v.color = Color((*colors)[0]).asNormalizedRGBA();
                }
                else
                {
                    v.color.set(255, 255, 255, 255);
                }

                if (normals)
                {
                    if (normals->getBinding() == normals->BIND_PER_VERTEX)
                        v.normal = (*normals)[i];
                    else
                        v.normal = (*normals)[0];
                }
                else
                {
                    v.normal.set(0, 0, 1);
                }

                if (uvs && uvs)
                {
                    if (uvs->getBinding() == normals->BIND_PER_VERTEX)
                        v.uv = (*uvs)[i];
                    else
                        v.uv = (*uvs)[0];
                }

                v.albedo = material ? material->albedo : -1;

                _result._vbo_store.emplace_back(std::move(v));

                _result._materials.push_back(_materialStack.top());
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
                        _result._ebo_store.push_back(vbo_offset + index);
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

bool
Chonk::add(
    osg::Node* node,
    ChonkFactory& factory)
{
    OE_HARD_ASSERT(_variants.size() < 4);

    factory.load(node, *this);
    _variants.push_back({ 0u, _ebo_store.size(), 0.0f, FLT_MAX });

    _box.init();

    return true;
}

bool
Chonk::add(
    osg::Node* node,
    float min_pixel_size,
    float max_pixel_size,
    ChonkFactory& factory)
{
    OE_HARD_ASSERT(_variants.size() < 4);

    unsigned offset = _ebo_store.size();
    factory.load(node, *this);
    _variants.push_back({
        offset,
        (_ebo_store.size() - offset), // length of new variant
        min_pixel_size,
        max_pixel_size});

    _box.init();

    OE_INFO << LC << "Added variant with " << (_ebo_store.size() - offset) / 3 << " triangles" << std::endl;

    return true;
}

const Chonk::DrawCommands&
Chonk::getOrCreateCommands(osg::State& state) const
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

        gs.commands.reserve(_variants.size());

        // for each variant:
        for (auto& variant : _variants)
        {
            DrawCommand command;

            command.cmd.count = variant.length;
            command.cmd.firstIndex = variant.offset;
            command.indexBuffer.address = gs.ebo->address();
            command.indexBuffer.length = gs.ebo->size();
            command.vertexBuffer.address = gs.vbo->address();
            command.vertexBuffer.length = gs.vbo->size();

            gs.commands.push_back(std::move(command));
        }
    }

    return gs.commands;
}

const osg::BoundingBoxf&
Chonk::getBound()
{
    if (!_box.valid())
    {
        for (auto& vertex : _vbo_store)
            _box.expandBy(vertex.position);
    }
    return _box;
}

ChonkFactory::ChonkFactory(TextureArena* textures) :
    _textures(textures)
{
    //nop
}

void
ChonkFactory::load(
    osg::Node* node,
    Chonk& chonk)
{
    // convert all primitive sets to GL_TRIANGLES
    osgUtil::Optimizer o;
    o.optimize(node, o.INDEX_MESH);

    // Reorder indices for optimal cache usage.
    // DO NOT do this if alignment is set; using alignment
    // implies the verts are in a specific order for a good reason, which
    // is usually because the shader relies on gl_VertexID.
    //if (alignment == 0u)
    {
        osgUtil::VertexCacheVisitor vcv;
        node->accept(vcv);
        vcv.optimizeVertices();
    }

    // rip geometry and textures into a new Asset object
    Ripper ripper(chonk, _textures.get());
    node->accept(ripper);

    chonk._box.init();
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
    static const osg::Matrixf s_identity;
    add(value, s_identity);
}

void
ChonkDrawable::add(Chonk::Ptr chonk, const osg::Matrixf& xform)
{
    if (chonk)
    {
        ScopedMutexLock lock(_m);

        Instance instance;
        instance.xform = xform;
        instance.first_variant_cmd_index = 0;
        instance.visible_mask = 0;
        _batches[chonk].push_back(std::move(instance));

        for (unsigned i = 0; i < _gs.size(); ++i)
            _gs[i]._dirty = true;

        dirtyBound();
    }
}

void
ChonkDrawable::drawImplementation(osg::RenderInfo& ri) const
{
    osg::State& state = *ri.getState();
    GCState& gs = _gs[ri.getContextID()];

    if (gs._dirty)
    {
        ScopedMutexLock lock(_m);
        gs.update(_batches, *ri.getState());
    }

    if (_batches.size() > 0)
    {
        gs.draw(*ri.getState());
    }
}

osg::BoundingBox
ChonkDrawable::computeBoundingBox() const
{
    ScopedMutexLock lock(_m);
    
    osg::BoundingBox result;

    for(auto& batch : _batches)
    {
        auto& chonk = batch.first;   

        auto& box = chonk->getBound();
        if (box.valid())
        {
            float r = box.radius();
            auto& instances = batch.second;

            for (auto& instance : instances)
            {
                osg::Vec3f scale = instance.xform.getScale();
                float scalar_scale = std::max(scale.x(), std::max(scale.y(), scale.z()));
                float sr = r * scalar_scale;
                osg::Vec3f c = box.center() * instance.xform;
                osg::BoundingBox xbox(
                    c.x() - sr, c.y() - sr, c.z() - sr,
                    c.x() + sr, c.y() + sr, c.z() + sr);
                result.expandBy(xbox);
            }
        }
    }

    return result;
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



void
ChonkDrawable::GCState::initialize(osg::State& state)
{
    void(GL_APIENTRY * gl_VertexAttribFormat)(GLuint, GLint, GLenum, GLboolean, GLuint);
    osg::setGLExtensionFuncPtr(gl_VertexAttribFormat, "glVertexAttribFormat");

    void(GL_APIENTRY * gl_VertexAttribIFormat)(GLuint, GLint, GLenum, GLuint);
    osg::setGLExtensionFuncPtr(gl_VertexAttribIFormat, "glVertexAttribIFormat");

    void(GL_APIENTRY * gl_VertexAttribLFormat)(GLuint, GLint, GLenum, GLuint);
    osg::setGLExtensionFuncPtr(gl_VertexAttribLFormat, "glVertexAttribLFormatNV");

    // DrawElementsCommand buffer:
    _commandBuf = GLBuffer::create(GL_DRAW_INDIRECT_BUFFER, state, "ChonkDrawable/Cmd");

    // Per-culling instances:
    _instanceInputBuf = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state, "ChonkDrawable/Input");

    if (_cull)
    {
        // Culled instances (GPU only)
        _instanceOutputBuf = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state, "ChonkDrawable/Output");

        // Chonk data
        _chonkBuf = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state, "ChonkDrawable/Chonk");
    }

    // Multidraw command:
    osg::setGLExtensionFuncPtr(
        _glMultiDrawElementsIndirectBindlessNV,
        "glMultiDrawElementsIndirectBindlessNV");

    // VAO:
    _vao = GLVAO::create(state, "ChonkDrawable");

    // start recording...
    _vao->bind();

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
        _vao->ext()->glVertexAttribBinding(location, 0);
        _vao->ext()->glEnableVertexAttribArray(location);
    }

    // bind a "dummy buffer" that will record the stride, which is
    // simply the size of our vertex structure.
    _vao->ext()->glBindVertexBuffer(0, 0, 0, sizeof(Chonk::VertexGPU));

    // Finish recording
    _vao->unbind();



    // make the cull program
    // TODO: MOVE THIS ELSEWHERE.

    if (_cull)
    {
        _cullProgram = new osg::Program();
        _cullProgram->addShader(new osg::Shader(osg::Shader::COMPUTE, s_chonk_cull_compute_shader));
        _cullSS = new osg::StateSet();
        _cullSS->setAttribute(_cullProgram);
    }
}


void
ChonkDrawable::GCState::update(
    const Batches& batches,
    osg::State& state)
{
    if (_vao == nullptr)
    {
        initialize(state);
    }

    // build a list of draw commands, each of which will 
    // have N instances, one per chonk meta.
    _commands.clear();

    // build a LUT of all instances by (gl_InstanceID + gl_BaseInstance).
    std::vector<Instance> all_instances;

    std::vector<ChonkVariant> chonks;

    std::size_t max_variant_count = 0;

    for (auto& batch : batches)
    {
        auto& chonk = batch.first;
        auto& instances = batch.second;

        unsigned first_variant_cmd_index = _commands.size();

        // For each chonk variant, set up an instanced draw command:
        Chonk::DrawCommands variantCommands = chonk->getOrCreateCommands(state);

        for(unsigned i=0; i<variantCommands.size(); ++i)
        {
            auto& command = variantCommands[i];

            command.cmd.instanceCount = instances.size();
            command.cmd.baseInstance = all_instances.size();
            _commands.emplace_back(std::move(command));

            if (_cull)
            {
                // record the bounding box of this chonk:
                auto& bs = chonk->getBound();
                ChonkVariant v;
                v.center = bs.center();
                v.radius = bs.radius();
                v.min_pixel_size = chonk->_variants[i].minPixelSize;
                v.max_pixel_size = chonk->_variants[i].maxPixelSize;
                v.num_variants = chonk->_variants.size();
                chonks.push_back(std::move(v));
            }
        }

        // append the instance data (transforms) and set
        // the index of the first variant command, which the compute
        // shader will need.
        for (auto& instance : instances)
        {
            all_instances.push_back(instance);
            all_instances.back().first_variant_cmd_index = first_variant_cmd_index;
        }

        max_variant_count = std::max(max_variant_count, variantCommands.size());
    }

    // set globals
    for (auto& chonk : chonks)
    {
        chonk.total_num_commands = _commands.size();
    }

    // Send to the GPU:
    _commandBuf->uploadData(
        _commands.size() * sizeof(Chonk::DrawCommand),
        _commands.data());

    _instanceInputBuf->uploadData(
        all_instances.size() * sizeof(Instance),
        all_instances.data());

    if (_cull)
    {
        _chonkBuf->uploadData(
            chonks.size() * sizeof(ChonkVariant),
            chonks.data());

        // just reserve space:
        _instanceOutputBuf->uploadData(
            all_instances.size() * sizeof(Instance),
            nullptr);
    }

    _numInstances = all_instances.size();

    _maxNumVariants = max_variant_count;

    _dirty = false;
}

void
ChonkDrawable::GCState::cull(osg::State& state)
{
    if (_commands.empty())
        return;

    // save currently bound program
    auto prev_pcp = state.getLastAppliedProgramObject();
    state.apply(_cullSS.get());

    // transmit the uniforms
    state.applyModelViewAndProjectionUniformsIfRequired();

    auto ext = _vao->ext();

    if (_passUL < 0)
    {
        auto pcp = state.getLastAppliedProgramObject();
        OE_HARD_ASSERT(pcp != nullptr);
        _passUL = pcp->getUniformLocation("oe_pass");
    }

    // reset the command buffer values
    for (auto& command : _commands)
    {
        command.cmd.instanceCount = 0;
        command.cmd.baseInstance = 0;
    }

    // bind it as an SSBO so we can muck with it
    _commandBuf->uploadData(
        GL_SHADER_STORAGE_BUFFER,
        _commands.size() * sizeof(Chonk::DrawCommand),
        _commands.data());

    _instanceOutputBuf->bindBufferBase(0);
    _commandBuf->bindBufferBase(1);
    _chonkBuf->bindBufferBase(2);
    _instanceInputBuf->bindBufferBase(3);

    // cull:
    ext->glUniform1i(_passUL, 0);
    ext->glDispatchCompute(_numInstances, _maxNumVariants, 1);

    // compact:
    ext->glUniform1i(_passUL, 1);
    ext->glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    ext->glDispatchCompute(_numInstances, _maxNumVariants, 1);

    //state.popStateSet();
    //TODO: later this should be its own scene graph pass so
    // we don't need to do this
    if (prev_pcp)
        prev_pcp->useProgram();
}

void
ChonkDrawable::GCState::draw(osg::State& state)
{
    if (_cull)
    {
        cull(state);
    }

    // bind the command list
    _commandBuf->bind();

    // make the instance LUT visible in the shader
    // (use gl_InstanceID + gl_BaseInstance to access)
    if (_cull)
        _instanceOutputBuf->bindBufferBase(0);
    else
        _instanceInputBuf->bindBufferBase(0);

    // render
    _vao->bind();

    // sync to cull results
    _vao->ext()->glMemoryBarrier(
        GL_SHADER_STORAGE_BARRIER_BIT | GL_COMMAND_BARRIER_BIT);

    _glMultiDrawElementsIndirectBindlessNV(
        GL_TRIANGLES,
        GL_UNSIGNED_SHORT,
        (const GLvoid*)0,
        _commands.size(),
        sizeof(Chonk::DrawCommand),
        1);
    
    _vao->unbind();
}

void
ChonkDrawable::GCState::release()
{
    _vao = nullptr;
    _commandBuf = nullptr;
    _instanceInputBuf = nullptr;
    _instanceOutputBuf = nullptr;
    _chonkBuf = nullptr;
    _commands.clear();
    _dirty = true;
}
