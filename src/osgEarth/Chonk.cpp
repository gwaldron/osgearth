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
#include "GLUtils"
#include "Metrics"

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

#pragma import_defines(OE_USE_CUSTOM_CULL_FUNCTION)
#ifdef OE_USE_CUSTOM_CULL_FUNCTION
bool oe_custom_cull(in vec2);
#endif

#pragma import_defines(OE_IS_SHADOW_CAMERA)

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
    vec2 local_uv;
    float fade;
    float visibility[4];
    uint first_variant_cmd_index;
};

layout(binding=0) buffer OutputBuffer
{
    Instance output_instances[];
};

layout(binding=29) buffer Commands
{
    DrawElementsIndirectBindlessCommandNV commands[];
};

layout(binding=30) buffer ChonkVariants
{
    ChonkVariant chonks[];
};

layout(binding=31) buffer InputBuffer
{
    Instance input_instances[];
};

uniform vec3 oe_Camera;
uniform float oe_pixel_size_scale = 1.0;

void cull()
{
    const uint i = gl_GlobalInvocationID.x; // instance
    const uint variant = gl_GlobalInvocationID.y; // variant

    // initialize by clearing the visibility for this variant:
    input_instances[i].visibility[variant] = 0.0;

#ifdef OE_IS_SHADOW_CAMERA
    // only use the highest LOD for shadow-casting.
    // TODO: reevaluate this.
    if (variant > 0)
        return;
#endif

    // bail if our chonk does not have this variant
    uint v = input_instances[i].first_variant_cmd_index + variant;
    if (variant >= chonks[v].num_variants)
        return;

    // TODO: benchmark this before and after the SS cull below.
#ifdef OE_USE_CUSTOM_CULL_FUNCTION
    if (oe_custom_cull(input_instances[i].local_uv) == false)
        return;
#endif

    // transform the bounding sphere to a view-space bbox.
    vec4 center = input_instances[i].xform * vec4(chonks[v].bs.xyz, 1);
    vec4 center_view = gl_ModelViewMatrix * center;
    float radius = chonks[v].bs.w;
    vec4 LL_view = center_view - vec4(radius, radius, 0, 0);
    vec4 UR_view = center_view + vec4(radius, radius, 0, 0);

    // transform the bbox to clip and see if it intersects the frustum:
    vec4 LL = gl_ProjectionMatrix * LL_view;
    LL.xyz /= LL.w;
    if (LL.x > 1.0 || LL.y > 1.0 || LL.z > 1.0)
        return;
    vec4 UR = gl_ProjectionMatrix * UR_view;
    UR.xyz /= UR.w;
    // note: avoid testing against near clip since it may not be set yet
    if (UR.x < -1.0 || UR.y < -1.0) // || UR.z < -1.0)
        return;

    float fade = 1.0;

#ifndef OE_IS_SHADOW_CAMERA
    // OK, it is in view - now check pixel size on screen for this variant:
    vec2 dims = 0.5*(UR.xy-LL.xy)*oe_Camera.xy;

    float pixelSize = max(dims.x, dims.y);
    float pixelSizePad = pixelSize*0.1;

    float maxPixelSize = chonks[v].max_pixel_size * oe_pixel_size_scale;
    if (pixelSize > (maxPixelSize + pixelSizePad))
        return;

    float minPixelSize = chonks[v].min_pixel_size * oe_pixel_size_scale;
    if (pixelSize < (minPixelSize - pixelSizePad))
        return;

    if (pixelSize > maxPixelSize)
        fade = 1.0-(pixelSize-maxPixelSize)/pixelSizePad;
    if (pixelSize < minPixelSize)
        fade = 1.0-(minPixelSize-pixelSize)/pixelSizePad;
#endif

    // Pass! Set the visibility for this variant:
    input_instances[i].visibility[variant] = fade;

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
    
    float fade = input_instances[i].visibility[variant];
    if (fade < 0.15)
        return;

    uint v = input_instances[i].first_variant_cmd_index + variant;
    uint offset = commands[v].cmd.baseInstance;
    uint index = atomicAdd(commands[v].cmd.instanceCount, 1);

    // Lazy! Re-using the instance struct for render leaves..
    output_instances[offset+index] = input_instances[i];
    output_instances[offset+index].fade = fade;
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
        std::list<ChonkMaterial::Ptr> _materialCache;
        std::stack<ChonkMaterial::Ptr> _materialStack;
        std::stack<osg::Matrix> _transformStack;
        std::unordered_map<osg::Texture*, unsigned> _textureLUT;

        const unsigned ALBEDO = 0;
        const unsigned NORMAL = 1;

        ChonkMaterial::Ptr reuseOrCreateMaterial(int albedo, int normal)
        {
            for (auto& m : _materialCache)
            {
                if (m->albedo == albedo && m->normal == normal)
                    return m;
            }
            auto material = ChonkMaterial::create();
            material->albedo = albedo;
            material->normal = normal;

            // If our arena is in auto-release mode, we need to 
            // store a pointer to each texture we use so they do not
            // get deleted while the material is still in business:
            if (_textures && _textures->getAutoRelease() == true)
            {
                material->albedo_tex = _textures->find(material->albedo);
                material->normal_tex = _textures->find(material->normal);
            }

            _materialCache.push_back(material);
            return material;
        }

        Ripper(Chonk& chonk, TextureArena* textures) :
            _result(chonk),
            _textures(textures)
        {
            setTraversalMode(TRAVERSE_ALL_CHILDREN);
            setNodeMaskOverride(~0);

            _materialStack.push(reuseOrCreateMaterial(-1, -1));
            _transformStack.push(osg::Matrix());
        }

        std::stack<int> _materialIndexStack;

        // adds a teture to the arena and returns its index
        int addTexture(unsigned slot, osg::StateSet* stateset)
        {
            if (!_textures)
                return -1;

            int result = -1;

            osg::Texture* tex = dynamic_cast<osg::Texture*>(
                stateset->getTextureAttribute(slot, osg::StateAttribute::TEXTURE));

            if (tex && tex->getImage(0))
            {
                auto i = _textureLUT.find(tex);

                if (i == _textureLUT.end())
                {
                    Texture::Ptr t = Texture::create();
                    t->_image = tex->getImage(0);
                    t->_uri = t->_image->getFileName();
                    t->_label = "Chonk texture";

                    result = _textures->add(t);
                    if (result >= 0)
                    {
                        _textureLUT[tex] = result;
                    }
                }
                else
                {
                    result = i->second;
                }
            }

            return result;
        }

        // record materials, and return true if we pushed one.
        bool pushStateSet(osg::StateSet* stateset)
        {
            bool pushed = false;
            if (stateset)
            {
                int albedo = addTexture(ALBEDO, stateset);
                int normal = addTexture(NORMAL, stateset);

                if (albedo >= 0 || normal >= 0)
                {
                    ChonkMaterial::Ptr material = reuseOrCreateMaterial(albedo, normal);
                    _materialStack.push(material);
                    pushed = true;
                }
            }
            return pushed;
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
            auto flexers = dynamic_cast<osg::Vec3Array*>(node.getTexCoordArray(3));

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

                if (uvs)
                {
                    if (uvs->getBinding() == normals->BIND_PER_VERTEX)
                        v.uv = (*uvs)[i];
                    else
                        v.uv = (*uvs)[0];
                }

                if (flexers)
                {
                    if (flexers->getBinding() == flexers->BIND_PER_VERTEX)
                        v.flex = (*flexers)[i];
                    else
                        v.flex = (*flexers)[0];
                }

                v.albedo = material ? material->albedo : -1;
                v.normalmap = material ? material->normal : -1;

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

namespace std {
    // std::hash specialization for ChonkMaterial
    template<> struct hash<ChonkMaterial> {
        inline size_t operator()(const ChonkMaterial& value) const {
            return hash_value_unsigned(
                value.albedo,
                value.normal);
        }
    };
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
    OE_SOFT_ASSERT_AND_RETURN(node != nullptr, false);
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
    OE_SOFT_ASSERT_AND_RETURN(node != nullptr, false);
    OE_HARD_ASSERT(_variants.size() < 4);

    unsigned offset = _ebo_store.size();
    factory.load(node, *this);
    _variants.push_back({
        offset,
        (_ebo_store.size() - offset), // length of new variant
        min_pixel_size,
        max_pixel_size});

    _box.init();

    OE_DEBUG << LC << "Added variant with " << (_ebo_store.size() - offset) / 3 << " triangles" << std::endl;

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

    // rip geometry and textures into a new Asset object
    Ripper ripper(chonk, _textures.get());
    node->accept(ripper);

    // dirty its bounding box
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

namespace
{
    struct VADef {
        GLint size;
        GLenum type;
        GLboolean normalize;
        GLint offset;
    };

    Mutex _s_cullStateSetMutex;
    using CullSSLUT = std::unordered_map<osg::Shader*, osg::ref_ptr<osg::StateSet>>;
    osg::ref_ptr<osg::Shader> _s_baseShader;
    CullSSLUT _s_cullStateSets;
}

ChonkDrawable::ChonkDrawable() :
    osg::Drawable(),
    _proxy_dirty(true),
    _cull(true)
{
    setName(typeid(*this).name());
    setCustomCullingShader(nullptr);
}

ChonkDrawable::~ChonkDrawable()
{
    //nop
}

void
ChonkDrawable::setCullPerChonk(bool value)
{
    _cull = value;
}

void
ChonkDrawable::add(
    Chonk::Ptr value)
{
    static const osg::Matrixf s_identity_xform;
    static const osg::Vec2f s_def_uv(0.0f, 0.0f);
    add(value, s_identity_xform, s_def_uv);
}

void
ChonkDrawable::add(
    Chonk::Ptr value,
    const osg::Matrixf& xform)
{
    static const osg::Vec2f s_def_uv(0.0f, 0.0f);
    add(value, xform, s_def_uv);
}

void
ChonkDrawable::add(
    Chonk::Ptr chonk,
    const osg::Matrixf& xform,
    const osg::Vec2f& local_uv)
{
    if (chonk)
    {
        ScopedMutexLock lock(_m);

        Instance instance;
        instance.xform = xform;
        instance.uv = local_uv;
        instance.first_variant_cmd_index = 0;
        //instance.visibility_mask = 0;
        _batches[chonk].push_back(std::move(instance));

        for (unsigned i = 0; i < _gs.size(); ++i)
            _gs[i]._dirty = true;

        dirtyBound();
    }
}

void
ChonkDrawable::setDrawStateSet(osg::StateSet* value)
{
    _drawSS = value;
}

void
ChonkDrawable::setChildren(const Vector& value)
{
    _children = value;
}

void
ChonkDrawable::setModelViewMatrix(const osg::Matrix& value)
{
    _mvm = value;
}

void
ChonkDrawable::setCustomCullingShader(osg::Shader* value)
{
    ScopedMutexLock lock(_s_cullStateSetMutex);

    if (_s_baseShader.valid() == false)
    {
        _s_baseShader = new osg::Shader(
            osg::Shader::COMPUTE, 
            s_chonk_cull_compute_shader);
    }

    osg::ref_ptr<osg::StateSet>& ss = _s_cullStateSets[value];
    if (ss.valid() == false)
    {
        auto program = new osg::Program();
        program->addShader(_s_baseShader);
        if (value != nullptr)
            program->addShader(value);
        ss = new osg::StateSet();
        ss->setAttribute(program);
        if (value != nullptr)
            ss->setDefine("OE_USE_CUSTOM_CULL_FUNCTION");
    }
    _cullSS = ss;
}

void
ChonkDrawable::drawImplementation(osg::RenderInfo& ri) const
{
    OE_PROFILING_ZONE;
    OE_GL_ZONE_NAMED(getName().c_str());

    osg::State& state = *ri.getState();

    GCState& gs = _gs[state.getContextID()];
    if (!gs._vao)
    {
        gs._cull = _cull;
        gs.initialize(state);
    }

    if (!_children.empty())
    {
        update_and_cull_children(state);
        draw_children(gs, state);
    }

    else if (!_batches.empty())
    {
        if (_cull)
        {
            // activate the culling compute shader and cull
            state.apply(_cullSS.get());

            update_and_cull_batches(state);

            // apply the stateset with our rendering shader:
            if (_drawSS.valid())
                state.apply(_drawSS.get());
            else
                state.apply();
        }
        else
        {
            update_and_cull_batches(state);
        }

        // render
        gs._vao->bind();

        // sync to cull results:
        gs._ext->glMemoryBarrier(
            GL_SHADER_STORAGE_BARRIER_BIT | GL_COMMAND_BARRIER_BIT);

        draw_batches(state);

        gs._vao->unbind();
    }
}

void
ChonkDrawable::update_and_cull_children(osg::State& state) const
{
    OE_PROFILING_ZONE;
    OE_GL_ZONE_NAMED("GPU Cull");

    // activate the culling compute shader and cull all subs
    if (_cull)
        state.apply(_cullSS.get());

    for (auto& child : _children)
    {
        child->update_and_cull_batches(state);
    }

    // restore state before cull
    if (_cull && !_drawSS.valid())
        state.apply();
}

void
ChonkDrawable::draw_children(GCState& gs, osg::State& state) const
{
    OE_PROFILING_ZONE;
    OE_GL_ZONE_NAMED("Draw");

    // apply the stateset with our rendering shader:
    if (_drawSS.valid())
        state.apply(_drawSS.get());

    // activate the VAO and draw all subs.
    gs._vao->bind();

    // sync to cull results (call as late as possible)
    gs._ext->glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_COMMAND_BARRIER_BIT);

    for (auto& child : _children)
    {
        child->draw_batches(state);
    }
    gs._vao->unbind();
}

void
ChonkDrawable::update_and_cull_batches(osg::State& state) const
{
    GCState& gs = _gs[state.getContextID()];
    //if (gs._dirty)
    {
        ScopedMutexLock lock(_m);
        gs.update(_batches, state);
    }

    osg::Matrix saved_mvm;
    if (!_mvm.isIdentity())
    {
        saved_mvm = state.getModelViewMatrix();
        state.applyModelViewMatrix(_mvm);
    }

    if (_cull)
    {
        gs.cull(state);
    }
}

void
ChonkDrawable::draw_batches(osg::State& state) const
{
    GCState& gs = _gs[state.getContextID()];

    osg::Matrix saved_mvm;
    if (!_mvm.isIdentity())
    {
        saved_mvm = state.getModelViewMatrix();
        state.applyModelViewMatrix(_mvm);
    }

    gs.draw(state);

    if (!saved_mvm.isIdentity())
        state.applyModelViewMatrix(saved_mvm);
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
ChonkDrawable::refreshProxy() const
{
    if (_proxy_dirty)
    {
        ScopedMutexLock lock(_m);

        _proxy_verts.clear();
        _proxy_indices.clear();

        unsigned offset = 0;
        for (auto& batch : _batches)
        {
            Chonk::Ptr c = batch.first;
            for (auto& instance : batch.second)
            {
                for (auto& vert : c->_vbo_store)
                {
                    _proxy_verts.push_back(vert.position * instance.xform);
                }
                for (auto& index : c->_ebo_store)
                {
                    _proxy_indices.push_back(index + offset);
                }
            }
            offset += c->_vbo_store.size();
        }

        _proxy_dirty = false;
    }
}

void
ChonkDrawable::accept(osg::PrimitiveFunctor& f) const
{
    for (auto& child : _children)
        child->accept(f);

    if (!_batches.empty())
    {
        refreshProxy();
        f.setVertexArray(_proxy_verts.size(), _proxy_verts.data());
        f.drawElements(GL_TRIANGLES, _proxy_indices.size(), _proxy_indices.data());
    }
}


void
ChonkDrawable::GCState::initialize(osg::State& state)
{
    _ext = state.get<osg::GLExtensions>();

    void(GL_APIENTRY * gl_VertexAttribFormat)(GLuint, GLint, GLenum, GLboolean, GLuint);
    osg::setGLExtensionFuncPtr(gl_VertexAttribFormat, "glVertexAttribFormat");

    void(GL_APIENTRY * gl_VertexAttribIFormat)(GLuint, GLint, GLenum, GLuint);
    osg::setGLExtensionFuncPtr(gl_VertexAttribIFormat, "glVertexAttribIFormat");

    void(GL_APIENTRY * gl_VertexAttribLFormat)(GLuint, GLint, GLenum, GLuint);
    osg::setGLExtensionFuncPtr(gl_VertexAttribLFormat, "glVertexAttribLFormatNV");

    // DrawElementsCommand buffer:
    _commandBuf = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state, "ChonkDrawable/Cmd");

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
    OE_HARD_ASSERT(_glMultiDrawElementsIndirectBindlessNV != nullptr);

    void(GL_APIENTRY * glEnableClientState_)(GLenum);
    osg::setGLExtensionFuncPtr(glEnableClientState_, "glEnableClientState");
    OE_HARD_ASSERT(glEnableClientState_ != nullptr);

    // VAO:
    _vao = GLVAO::create(state, "ChonkDrawable");

    // start recording...
    _vao->bind();

    // required in order to use BindlessNV extension
    glEnableClientState_(GL_VERTEX_ATTRIB_ARRAY_UNIFIED_NV);
    glEnableClientState_(GL_ELEMENT_ARRAY_UNIFIED_NV);

    const VADef formats[7] = {
        {3, GL_FLOAT,         GL_FALSE, offsetof(Chonk::VertexGPU, position)},
        {3, GL_FLOAT,         GL_FALSE, offsetof(Chonk::VertexGPU, normal)},
        {4, GL_UNSIGNED_BYTE, GL_TRUE,  offsetof(Chonk::VertexGPU, color)},
        {2, GL_FLOAT,         GL_FALSE, offsetof(Chonk::VertexGPU, uv)},
        {3, GL_FLOAT,         GL_FALSE, offsetof(Chonk::VertexGPU, flex)},
        {1, GL_INT,           GL_FALSE, offsetof(Chonk::VertexGPU, albedo)},
        {1, GL_INT,           GL_FALSE, offsetof(Chonk::VertexGPU, normalmap)}
    };

    // configure the format of each vertex attribute in our structure.
    for (unsigned location = 0; location < 7; ++location)
    {
        const VADef& d = formats[location];
        if (d.type == GL_INT || d.type == GL_INT)
            gl_VertexAttribIFormat(location, d.size, d.type, d.offset);
        else
            gl_VertexAttribFormat(location, d.size, d.type, d.normalize, d.offset);
        _ext->glVertexAttribBinding(location, 0);
        _ext->glEnableVertexAttribArray(location);
    }

    // bind a "dummy buffer" that will record the stride, which is
    // simply the size of our vertex structure.
    _ext->glBindVertexBuffer(0, 0, 0, sizeof(Chonk::VertexGPU));

    // Finish recording
    _vao->unbind();
}


void
ChonkDrawable::GCState::update(
    const Batches& batches,
    osg::State& state)
{
    OE_PROFILING_ZONE;
    OE_GL_ZONE_NAMED("Update");

    if (_vao == nullptr)
    {
        initialize(state);
    }

    // build a list of draw commands, each of which will 
    // have N instances, one per chonk meta.
    _commands.clear();

    // record for each variant (LOD) of each chonk
    _chonk_variants.clear();

    // build a LUT of all instances by (gl_InstanceID + gl_BaseInstance).
    _all_instances.clear();

    std::size_t max_variant_count = 0;

    for (auto& batch : batches)
    {
        auto& chonk = batch.first;
        auto& instances = batch.second;

        unsigned first_variant_cmd_index = _commands.size();

        // For each chonk variant, set up an instanced draw command:
        const Chonk::DrawCommands& variantCommands = chonk->getOrCreateCommands(state);

        for(unsigned i=0; i<variantCommands.size(); ++i)
        {
            _commands.emplace_back(variantCommands[i]);

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
                _chonk_variants.push_back(std::move(v));
            }
        }

        // append the instance data (transforms) and set
        // the index of the first variant command, which the compute
        // shader will need.
        for (auto& instance : instances)
        {
            _all_instances.push_back(instance);
            _all_instances.back().first_variant_cmd_index = first_variant_cmd_index;
        }

        max_variant_count = std::max(max_variant_count, variantCommands.size());
    }

    // set globals
    for (auto& cv : _chonk_variants)
    {
        cv.total_num_commands = _commands.size();
    }

    // Send to the GPU:
    _commandBuf->uploadData(_commands);
    _instanceInputBuf->uploadData(_all_instances);

    if (_cull)
    {
        _chonkBuf->uploadData(_chonk_variants);
        
        // just reserve space if necessary.
        // this is a NOP if the buffer is already sized properly
        _instanceOutputBuf->uploadData(_instanceInputBuf->size(), nullptr);
    }

    _numInstances = _all_instances.size();
    _maxNumVariants = max_variant_count;

    _dirty = false;
}

void
ChonkDrawable::GCState::cull(osg::State& state)
{
    if (_commands.empty())
        return;

    OE_GL_ZONE_NAMED("Cull Dispatch");

    // transmit the uniforms
    state.applyModelViewAndProjectionUniformsIfRequired();

    auto ext = _vao->ext();

    auto pcp = state.getLastAppliedProgramObject();
    OE_HARD_ASSERT(pcp != nullptr, "Check for shader errors!");
    PCPState& ps = _pcps[pcp];
    if (ps._passUL < 0)
    {
        ps._passUL = pcp->getUniformLocation("oe_pass");
    }

    // reset the command buffer values
    for (auto& command : _commands)
    {
        command.cmd.instanceCount = 0;
        command.cmd.baseInstance = 0;
    }

    // bind it as an SSBO so we can muck with it
    _commandBuf->uploadData(_commands);

    _instanceOutputBuf->bindBufferBase(0);
    _commandBuf->bindBufferBase(29);
    _chonkBuf->bindBufferBase(30);
    _instanceInputBuf->bindBufferBase(31);

    // cull:
    ext->glUniform1i(ps._passUL, 0);
    ext->glDispatchCompute(_numInstances, _maxNumVariants, 1);

    // compact:
    ext->glUniform1i(ps._passUL, 1);
    ext->glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    ext->glDispatchCompute(_numInstances, _maxNumVariants, 1);
}

void
ChonkDrawable::GCState::draw(osg::State& state)
{
    // transmit the uniforms
    state.applyModelViewAndProjectionUniformsIfRequired();

    // bind the command list for drawing
    _commandBuf->bind(GL_DRAW_INDIRECT_BUFFER);

    // make the instance LUT visible in the shader
    // (use gl_InstanceID + gl_BaseInstance to access)
    if (_cull)
        _instanceOutputBuf->bindBufferBase(0);
    else
        _instanceInputBuf->bindBufferBase(0);

    _glMultiDrawElementsIndirectBindlessNV(
        GL_TRIANGLES,
        GL_UNSIGNED_SHORT,
        (const GLvoid*)0,
        _commands.size(),
        sizeof(Chonk::DrawCommand),
        1);
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
