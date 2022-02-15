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
#include "VirtualProgram"
#include "ShaderLoader"

#include <osg/Switch>
#include <osg/LOD>
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
#pragma import_defines(OE_GPUCULL_DEBUG)

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

struct ChonkLOD
{
    vec4 bs;
    float far_pixel_scale;
    float near_pixel_scale;
    uint num_lods;
    uint total_num_commands; // global
};

struct Instance
{
    mat4 xform;
    vec2 local_uv;
    float fade;
    float visibility[4];
    uint first_lod_cmd_index;
};

layout(binding=0) buffer OutputBuffer
{
    Instance output_instances[];
};

layout(binding=29) buffer Commands
{
    DrawElementsIndirectBindlessCommandNV commands[];
};

layout(binding=30) buffer ChonkLODs
{
    ChonkLOD chonks[];
};

layout(binding=31) buffer InputBuffer
{
    Instance input_instances[];
};

uniform vec3 oe_Camera;
uniform float oe_sse;

#ifdef OE_GPUCULL_DEBUG
#define REJECT(X) if (fade==1.0) fade=X
#else
#define REJECT(X) return
#endif
#define REASON_FRUSTUM 1.5
#define REASON_SSE 2.5
#define REASON_NEARCLIP 3.5

void cull()
{
    const uint i = gl_GlobalInvocationID.x; // instance
    const uint lod = gl_GlobalInvocationID.y; // lod

    // initialize by clearing the visibility for this LOD:
    input_instances[i].visibility[lod] = 0.0;

#ifdef OE_IS_SHADOW_CAMERA
    // only use the highest LOD for shadow-casting.
    // TODO: reevaluate this.
    if (lod > 0)
        return;
#endif

    // bail if our chonk does not have this LOD
    uint v = input_instances[i].first_lod_cmd_index + lod;
    if (lod >= chonks[v].num_lods)
        return;

    // TODO: benchmark this before and after the SS cull below.
#ifdef OE_USE_CUSTOM_CULL_FUNCTION
    if (oe_custom_cull(input_instances[i].local_uv) == false)
        return;
#endif

    // intialize:
    float fade = 1.0;

    // transform the bounding sphere to a view-space bbox.
    vec4 center = input_instances[i].xform * vec4(chonks[v].bs.xyz, 1);
    vec4 center_view = gl_ModelViewMatrix * center;

    float r = chonks[v].bs.w;

#ifdef OE_GPUCULL_DEBUG
    r = max(1.0, r-20.0);
#endif

    // Trivially accept (at the highest LOD) anything whose bounding sphere
    // intersects the near clip plane:
    float near = gl_ProjectionMatrix[2][3] / (gl_ProjectionMatrix[2][2]-1.0);
    if (-(center_view.z + r) <= near)
    {
        if (lod > 0) // reject all lower LODs
            REJECT(REASON_NEARCLIP);
    }
    else
    {
        // find the clip-space MBR and intersect with the clip frustum:
        vec4 LL, UR, temp;
        temp = gl_ProjectionMatrix * (center_view + vec4(-r,-r,-r,0)); temp /= temp.w;
        LL = temp; UR = temp;
        temp = gl_ProjectionMatrix * (center_view + vec4(-r,-r,+r,0)); temp /= temp.w;
        LL = min(LL, temp); UR = max(UR, temp);
        temp = gl_ProjectionMatrix * (center_view + vec4(-r,+r,-r,0)); temp /= temp.w;
        LL = min(LL, temp); UR = max(UR, temp);
        temp = gl_ProjectionMatrix * (center_view + vec4(-r,+r,+r,0)); temp /= temp.w;
        LL = min(LL, temp); UR = max(UR, temp);
        temp = gl_ProjectionMatrix * (center_view + vec4(+r,-r,-r,0)); temp /= temp.w;
        LL = min(LL, temp); UR = max(UR, temp);
        temp = gl_ProjectionMatrix * (center_view + vec4(+r,-r,+r,0)); temp /= temp.w;
        LL = min(LL, temp); UR = max(UR, temp);
        temp = gl_ProjectionMatrix * (center_view + vec4(+r,+r,-r,0)); temp /= temp.w;
        LL = min(LL, temp); UR = max(UR, temp);
        temp = gl_ProjectionMatrix * (center_view + vec4(+r,+r,+r,0)); temp /= temp.w;
        LL = min(LL, temp); UR = max(UR, temp);

        if (LL.x > LL.w || LL.y > LL.w || LL.z > LL.w)
            REJECT(REASON_FRUSTUM);

        if (UR.x < -UR.w || UR.y < -UR.w || UR.z < -UR.w)
            REJECT(REASON_FRUSTUM);

#ifndef OE_IS_SHADOW_CAMERA

        if (oe_sse > 0)
        {
            // OK, it is in view - now check pixel size on screen for this LOD:
            LL.xy /= LL.w;
            UR.xy /= UR.w;
            vec2 dims = 0.5*(UR.xy-LL.xy)*oe_Camera.xy;
    
            float pixelSize = max(dims.x, dims.y);
            float pixelSizePad = pixelSize*0.1;

            float minPixelSize = oe_sse * chonks[v].far_pixel_scale;
            if (pixelSize < (minPixelSize - pixelSizePad))
                REJECT(REASON_SSE);

            float maxPixelSize = oe_sse * chonks[v].near_pixel_scale;
            if (pixelSize > (maxPixelSize + pixelSizePad))
                REJECT(REASON_SSE);

            if (fade == 1.0) // good to go, set the proper fade:
            {
                if (pixelSize > maxPixelSize)
                    fade = 1.0-(pixelSize-maxPixelSize)/pixelSizePad;
                else if (pixelSize < minPixelSize)
                    fade = 1.0-(minPixelSize-pixelSize)/pixelSizePad;
            }
        }
#endif
    }

    // Pass! Set the visibility for this LOD:
    input_instances[i].visibility[lod] = fade;

    // Bump all baseInstances following this one:
    const uint cmd_count = chonks[v].total_num_commands;
    for(uint i=v+1; i<cmd_count; ++i)
        atomicAdd(commands[i].cmd.baseInstance, 1);
}

// Copies the visible instances to a compacted output buffer.
void compact()
{
    const uint i = gl_GlobalInvocationID.x; // instance
    const uint lod = gl_GlobalInvocationID.y; // lod
    
    float fade = input_instances[i].visibility[lod];
    if (fade < 0.15)
        return;

    uint v = input_instances[i].first_lod_cmd_index + lod;
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

const char* oe_chonk_default_shaders = R"(

#version 460
#extension GL_ARB_gpu_shader_int64 : enable
#pragma vp_function oe_chonk_default_vertex_model, vertex_model, 0.0
#pragma import_defines(OE_IS_SHADOW_CAMERA)

struct Instance {
    mat4 xform;
    vec2 local_uv;
    float fade;
    float visibility[4];
    uint first_variant_cmd_index;
};
layout(binding=0, std430) buffer Instances {
    Instance instances[];
};
layout(binding=1, std430) buffer TextureArena {
    uint64_t textures[];
};

layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;
layout(location=2) in vec4 color;
layout(location=3) in vec2 uv;
layout(location=4) in vec3 flex;
layout(location=5) in int albedo; // todo: material LUT index
layout(location=6) in int normalmap; // todo: material LUT index

// stage global
mat3 xform3;

// outputs
out vec3 vp_Normal;
out vec4 vp_Color;
out float oe_fade;
out vec2 oe_tex_uv;
flat out uint64_t oe_albedo_tex;
flat out uint64_t oe_normal_tex;

void oe_chonk_default_vertex_model(inout vec4 vertex)
{
    int i = gl_BaseInstance + gl_InstanceID;
    vertex = instances[i].xform * vec4(position, 1);
    vp_Color = color;
    xform3 = mat3(instances[i].xform);
    vp_Normal = xform3 * normal;
    oe_tex_uv = uv;
    oe_albedo_tex = albedo >= 0 ? textures[albedo] : 0;
    oe_normal_tex = normalmap >= 0 ? textures[normalmap] : 0;

#ifndef OE_IS_SHADOW_CAMERA
    oe_fade = instances[i].fade;
#else
    oe_fade = 1.0;
#endif
}

[break]

#version 460
#extension GL_ARB_gpu_shader_int64 : enable
#pragma vp_function oe_chonk_default_vertex_view, vertex_view, 0.0

// stage
mat3 xform3;

// output
out vec3 vp_Normal;
out vec3 oe_tangent;
flat out uint64_t oe_normal_tex;

void oe_chonk_default_vertex_view(inout vec4 vertex)
{
    if (oe_normal_tex > 0)
    {
        vec3 ZAXIS = gl_NormalMatrix * vec3(0,0,1);
        if (dot(ZAXIS, vp_Normal) > 0.95)
            oe_tangent = gl_NormalMatrix * (xform3 * vec3(1,0,0));
        else
            oe_tangent = cross(ZAXIS, vp_Normal);
    }
}


[break]

#version 460
#extension GL_ARB_gpu_shader_int64 : enable
#pragma vp_function oe_chonk_default_fragment, fragment, 0.0
#pragma import_defines(OE_COMPRESSED_NORMAL)
#pragma import_defines(OE_GPUCULL_DEBUG)

// inputs
in float oe_fade;
in vec2 oe_tex_uv;
in vec3 oe_tangent;
in vec3 vp_Normal;
flat in uint64_t oe_albedo_tex;
flat in uint64_t oe_normal_tex;

void oe_chonk_default_fragment(inout vec4 color)
{
    if (oe_albedo_tex > 0)
    {
        vec4 texel = texture(sampler2D(oe_albedo_tex), oe_tex_uv);
        color *= texel;
    }

    // apply the high fade from the instancer
#ifdef OE_GPUCULL_DEBUG
    if (oe_fade <= 1.0) color.a *= oe_fade;
    else if (oe_fade <= 2.0) color = vec4(1,0,0,1);
    else if (oe_fade <= 3.0) color = vec4(1,1,0,1);
    else if (oe_fade <= 4.0) color = vec4(0,1,0,1);
    else color = vec4(1,0,1,1); // should never happen :)
#else
    color.a *= oe_fade;
#endif

    if (oe_normal_tex > 0)
    {
        vec4 n = texture(sampler2D(oe_normal_tex), oe_tex_uv);

#ifdef OE_COMPRESSED_NORMAL
        n.xyz = n.xyz*2.0 - 1.0;
        n.z = 1.0 - abs(n.x) - abs(n.y);
        float t = clamp(-n.z, 0, 1);
        n.x += (n.x > 0) ? -t : t;
        n.y += (n.y > 0) ? -t : t;
#else
        n.xyz = normalize(n.xyz*2.0-1.0);
#endif

        // construct the TBN, reflecting the normal on back-facing polys
        mat3 tbn = mat3(
            normalize(oe_tangent),
            normalize(cross(vp_Normal, oe_tangent)),
            normalize(gl_FrontFacing ? vp_Normal : -vp_Normal));
        
        vp_Normal = normalize(tbn * n.xyz);
    }
}

)";

    /**
     * Visitor that counts the verts and elements in a scene graph.
     */
    struct Counter : public osg::NodeVisitor
    {
        Counter() :
            _numVerts(0),
            _numElements(0)
        {
            // Use the "active chidren" mode to only bring in default switch
            // and osgSim::MultiSwitch children for now. -gw
            setTraversalMode(TRAVERSE_ACTIVE_CHILDREN);
            setNodeMaskOverride(~0);
        }

        void apply(osg::Geometry& node) override
        {
            auto verts = dynamic_cast<osg::Vec3Array*>(node.getVertexArray());
            if (verts) _numVerts += verts->size();
            
            for (unsigned i = 0; i < node.getNumPrimitiveSets(); ++i) {
                auto p = node.getPrimitiveSet(i);
                if (p) _numElements += p->getNumIndices();
            }
        }

        unsigned _numVerts;
        unsigned _numElements;
    };

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
        std::unordered_map<osg::Texture*, Texture::Ptr> _textureLUT;

        const unsigned ALBEDO = 0;
        const unsigned NORMAL = 1;

        ChonkMaterial::Ptr reuseOrCreateMaterial(
            Texture::Ptr albedo_tex,
            Texture::Ptr normal_tex)
        {
            int albedo = _textures->find(albedo_tex);
            int normal = _textures->find(normal_tex);

            for (auto& m : _materialCache)
            {
                if (m->albedo == albedo &&
                    m->normal == normal)
                {
                    return m;
                }
            }
            auto material = ChonkMaterial::create();
            material->albedo = albedo;
            material->normal = normal;

            // If our arena is in auto-release mode, we need to 
            // store a pointer to each texture we use so they do not
            // get deleted while the material is still in business:
            if (_textures && _textures->getAutoRelease() == true)
            {
                material->albedo_tex = albedo_tex;
                material->normal_tex = normal_tex;
            }

            _materialCache.push_back(material);
            return material;
        }

        Ripper(Chonk& chonk, TextureArena* textures) :
            _result(chonk),
            _textures(textures)
        {
            // Use the "active chidren" mode to only bring in default switch
            // and osgSim::MultiSwitch children for now. -gw
            setTraversalMode(TRAVERSE_ACTIVE_CHILDREN);
            setNodeMaskOverride(~0);

            _materialStack.push(reuseOrCreateMaterial(
                nullptr, nullptr));
            _transformStack.push(osg::Matrix());
        }

        std::stack<int> _materialIndexStack;

        // adds a teture to the arena and returns its index
        Texture::Ptr addTexture(unsigned slot, osg::StateSet* stateset)
        {
            if (!_textures)
                return nullptr;

            //int result = -1;
            Texture::Ptr result;

            osg::Texture* tex = dynamic_cast<osg::Texture*>(
                stateset->getTextureAttribute(slot, osg::StateAttribute::TEXTURE));

            if (tex && tex->getImage(0))
            {
                auto i = _textureLUT.find(tex);

                if (i == _textureLUT.end())
                {
                    result = Texture::create();
                    result->_image = tex->getImage(0);
                    result->_uri = tex->getImage(0)->getFileName();
                    result->_label = "Chonk texture";

                    int index = _textures->add(result);
                    if (index >= 0)
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
                Texture::Ptr albedo_tex = addTexture(ALBEDO, stateset);
                Texture::Ptr normal_tex = addTexture(NORMAL, stateset);

                if (albedo_tex || normal_tex)
                {
                    ChonkMaterial::Ptr material = reuseOrCreateMaterial(
                        albedo_tex, normal_tex);
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

            unsigned vbo_offset = _result._vbo_store.size();

            auto verts = dynamic_cast<osg::Vec3Array*>(node.getVertexArray());
            auto colors = dynamic_cast<osg::Vec4Array*>(node.getColorArray());
            auto normals = dynamic_cast<osg::Vec3Array*>(node.getNormalArray());
            auto flexers = dynamic_cast<osg::Vec3Array*>(node.getTexCoordArray(3));

            // support either 2- or 3-component tex coords, but only read the xy components!
            auto uv2s = dynamic_cast<osg::Vec2Array*>(node.getTexCoordArray(0));
            auto uv3s = dynamic_cast<osg::Vec3Array*>(node.getTexCoordArray(0));

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

                if (uv2s)
                {
                    if (uv2s->getBinding() == normals->BIND_PER_VERTEX)
                        v.uv = (*uv2s)[i];
                    else
                        v.uv = (*uv2s)[0];
                }
                else if (uv3s)
                {
                    if (uv3s->getBinding() == normals->BIND_PER_VERTEX)
                        v.uv.set((*uv3s)[i].x(), (*uv3s)[i].y());
                    else
                        v.uv.set((*uv3s)[0].x(), (*uv3s)[0].y());
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

                // per-vert materials:
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
    OE_HARD_ASSERT(_lods.size() < 4);

    factory.load(node, *this);
    _lods.push_back({ 0u, _ebo_store.size(), 0.0f, FLT_MAX });

    _box.init();

    return true;
}

bool
Chonk::add(
    osg::Node* node,
    float far_pixel_scale,
    float near_pixel_scale,
    ChonkFactory& factory)
{
    OE_SOFT_ASSERT_AND_RETURN(node != nullptr, false);
    OE_HARD_ASSERT(_lods.size() < 4);

    unsigned offset = _ebo_store.size();
    factory.load(node, *this);
    _lods.push_back({
        offset,
        (_ebo_store.size() - offset), // length of new variant
        far_pixel_scale,
        near_pixel_scale });

    _box.init();

    OE_DEBUG << LC << "Added LOD with " << (_ebo_store.size() - offset) / 3 << " triangles" << std::endl;

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
            _ebo_store.size() * sizeof(element_t),
            _ebo_store.data(),
            0); // permanent
        gs.ebo->makeResident();

        gs.commands.reserve(_lods.size());

        // for each variant:
        for (auto& lod : _lods)
        {
            DrawCommand command;

            command.cmd.count = lod.length;
            command.cmd.firstIndex = lod.offset;
            command.cmd.instanceCount = 1;
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
        for (auto index : _ebo_store)
            _box.expandBy(_vbo_store[index].position);
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
    
    // first count up the memory we need and allocate it
    Counter counter;
    node->accept(counter);
    chonk._vbo_store.reserve(counter._numVerts);
    chonk._ebo_store.reserve(counter._numElements);

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
    osg::ref_ptr<osg::Program> _s_cullProgram;
    osg::ref_ptr<osg::StateSet> _s_cullSS;
    CullSSLUT _s_cullStateSets;
}

ChonkDrawable::ChonkDrawable() :
    osg::Drawable(),
    _proxy_dirty(true),
    _gpucull(true)
{
    setName(typeid(*this).name());
    setCustomCullingShader(nullptr);
    setUseDisplayList(false);
    setUseVertexBufferObjects(false);
    setUseVertexArrayObject(false);

    getOrCreateStateSet()->setRenderBinDetails(
        818,
        "ChonkBin",
        osg::StateSet::OVERRIDE_PROTECTED_RENDERBIN_DETAILS);
}

ChonkDrawable::~ChonkDrawable()
{
    //nop
}

void
ChonkDrawable::installDefaultShader(osg::StateSet* ss)
{
    OE_SOFT_ASSERT_AND_RETURN(ss != nullptr, void());
    VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
    vp->addGLSLExtension("GL_ARB_gpu_shader_int64");
    ShaderLoader::load(vp, oe_chonk_default_shaders);
}

void
ChonkDrawable::setUseGPUCulling(bool value)
{
    _gpucull = value;
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
        instance.fade = 1.0f;
        instance.first_lod_cmd_index = 0;
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
        ss->setAttribute(program, osg::StateAttribute::OVERRIDE);
        if (value != nullptr)
            ss->setDefine("OE_USE_CUSTOM_CULL_FUNCTION");
    }
    _cullSS = ss;
    _cullProgram = dynamic_cast<osg::Program*>(ss->getAttribute(osg::StateAttribute::PROGRAM));
}

struct StateEx : public osg::State
{
    void applyUniforms()
    {
        OE_HARD_ASSERT(_lastAppliedProgramObject != nullptr);

        for (auto iter : _uniformMap)
        {
            auto& stack = iter.second;
            if (!stack.uniformVec.empty())
            {
                auto& pair = stack.uniformVec.back();
                _lastAppliedProgramObject->apply(*pair.first);
                //OE_INFO << "...applied " << iter.first << std::endl;
            }
            else {
               // OE_INFO << "...empty stack for " << iter.first << std::endl;
            }
        }
    }
};

void
ChonkDrawable::drawImplementation(osg::RenderInfo& ri) const
{
    OE_PROFILING_ZONE;
    OE_GL_ZONE_NAMED(getName().c_str());

    if (_batches.empty())
        return;

    osg::State& state = *ri.getState();
    GCState& gs = _gs[state.getContextID()];
    if (!gs._vao)
    {
        gs._cull = _gpucull;
        gs.initialize(state);
    }
        
    const osg::Program::PerContextProgram* pcp = nullptr; 

    if (_gpucull)
    {
        // save bound program:
        pcp = state.getLastAppliedProgramObject();

        // activate the culling compute shader and cull
        _cullProgram->apply(state);
        reinterpret_cast<StateEx*>(&state)->applyUniforms();
    }

    update_and_cull_batches(state);

    // restore the draw state (or use the custom one):
    if (_drawSS.valid()) {
        state.apply(_drawSS.get());
    }
    else if (pcp) {
        pcp->getProgram()->apply(state);
    }

    // render
    gs._vao->bind();

    // sync to cull results:
    gs._ext->glMemoryBarrier(
        GL_SHADER_STORAGE_BARRIER_BIT | GL_COMMAND_BARRIER_BIT);

    draw_batches(state);

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

    if (!_mvm.isIdentity())
    {
        state.applyModelViewMatrix(_mvm);
    }

    if (_gpucull)
    {
        gs.cull(state);
    }
}

void
ChonkDrawable::draw_batches(osg::State& state) const
{
    GCState& gs = _gs[state.getContextID()];

    if (!_mvm.isIdentity())
    {
        state.applyModelViewMatrix(_mvm);
    }

    gs.draw(state);
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
            auto& instances = batch.second;
            for (auto& instance : instances)
            {
                for (unsigned i = 0; i < 8; ++i)
                {
                    result.expandBy(box.corner(i) * instance.xform);
                }
            }
        }
    }

    return result;
}

osg::BoundingSphere
ChonkDrawable::computeBound() const
{
    return osg::BoundingSphere(computeBoundingBox());
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
    _commandBuf = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state, "ChonkDrawable");

    // Per-culling instances:
    _instanceInputBuf = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state, "ChonkDrawable");

    if (_cull)
    {
        // Culled instances (GPU only)
        _instanceOutputBuf = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state, "ChonkDrawable");

        // Chonk data
        _chonkBuf = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, state, "ChonkDrawable");
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
    _chonk_lods.clear();

    // build a LUT of all instances by (gl_InstanceID + gl_BaseInstance).
    _all_instances.clear();

    std::size_t max_lod_count = 0;

    for (auto& batch : batches)
    {
        auto& chonk = batch.first;
        auto& instances = batch.second;

        unsigned first_lod_cmd_index = _commands.size();

        // For each chonk variant, set up an instanced draw command:
        const Chonk::DrawCommands& lod_commands = chonk->getOrCreateCommands(state);

        for(unsigned i=0; i< lod_commands.size(); ++i)
        {
            _commands.emplace_back(lod_commands[i]);

            if (_cull)
            {
                // record the bounding box of this chonk:
                auto& bs = chonk->getBound();
                ChonkLOD v;
                v.center = bs.center();
                v.radius = bs.radius();
                v.far_pixel_scale = chonk->_lods[i].far_pixel_scale;
                v.near_pixel_scale = chonk->_lods[i].near_pixel_scale;
                v.num_lods = chonk->_lods.size();
                _chonk_lods.push_back(std::move(v));
            }
        }

        // append the instance data (transforms) and set
        // the index of the first variant command, which the compute
        // shader will need.
        for (auto& instance : instances)
        {
            _all_instances.push_back(instance);
            _all_instances.back().first_lod_cmd_index = first_lod_cmd_index;
        }

        max_lod_count = std::max(max_lod_count, lod_commands.size());
    }

    // set globals
    for (auto& lod : _chonk_lods)
    {
        lod.total_num_commands = _commands.size();
    }

    // Send to the GPU:
    _commandBuf->uploadData(_commands);
    _instanceInputBuf->uploadData(_all_instances);

    if (_cull)
    {
        _chonkBuf->uploadData(_chonk_lods);
        
        // just reserve space if necessary.
        // this is a NOP if the buffer is already sized properly
        _instanceOutputBuf->uploadData(_instanceInputBuf->size(), nullptr);
    }

    _numInstances = _all_instances.size();
    _maxNumLODs = max_lod_count;

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
    ext->glDispatchCompute(_numInstances, _maxNumLODs, 1);

    // compact:
    ext->glUniform1i(ps._passUL, 1);
    ext->glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    ext->glDispatchCompute(_numInstances, _maxNumLODs, 1);
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

    GLenum elementType = sizeof(Chonk::element_t) == sizeof(GLushort) ?
        GL_UNSIGNED_SHORT :
        GL_UNSIGNED_INT;

    _glMultiDrawElementsIndirectBindlessNV(
        GL_TRIANGLES,
        elementType,
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

ChonkRenderBin::ChonkRenderBin() :
    osgUtil::RenderBin()
{
    setName("ChonkBin");

    _cullProgram = new osg::Program();
    _cullProgram->addShader(new osg::Shader(
        osg::Shader::COMPUTE,
        s_chonk_cull_compute_shader));

    _cullSS = new osg::StateSet();
    _cullSS->setAttribute(_cullProgram);

    _stateset = new osg::StateSet();
    ChonkDrawable::installDefaultShader(_stateset.get());
}

ChonkRenderBin::ChonkRenderBin(const ChonkRenderBin& rhs, const osg::CopyOp& op) :
    osgUtil::RenderBin(rhs, op),
    _cullSS(rhs._cullSS),
    _cullProgram(rhs._cullProgram)
{
    //nop
}

void
ChonkRenderBin::robert(
    osg::State& state,
    osgUtil::RenderLeaf* leaf,
    osgUtil::RenderLeaf*& previous) const
{
    // mysterious state-fu from osgUtil::RenderLeaf
    if (previous)
    {
        osgUtil::StateGraph* prev_rg = previous->_parent;
        osgUtil::StateGraph* prev_rg_parent = prev_rg->_parent;
        osgUtil::StateGraph* rg = leaf->_parent;
        if (prev_rg_parent != rg->_parent)
        {
            osgUtil::StateGraph::moveStateGraph(state, prev_rg_parent, rg->_parent);
            state.apply(rg->getStateSet());

        }
        else if (rg != prev_rg)
        {
            state.apply(rg->getStateSet());
        }
    }
    else
    {
        osgUtil::StateGraph::moveStateGraph(state, nullptr, leaf->_parent->_parent);
        state.apply(_parent->getStateSet());
    }

    previous = leaf;
}

void
ChonkRenderBin::drawImplementation(
    osg::RenderInfo& ri,
    osgUtil::RenderLeaf*& previous)
{
    OE_GL_ZONE_NAMED("ChonkRenderBin");

    osg::State& state = *ri.getState();    
    
    // apply this bin's stateset....somewhere
    unsigned numToPop = (previous ? osgUtil::StateGraph::numToPop(previous->_parent) : 0);
    if (numToPop > 1) --numToPop;
    unsigned insertPos = state.getStateSetStackSize() - numToPop;
    if (_stateset.valid())
    {
        state.insertStateSet(insertPos, _stateset.get());
    }

    // copy everything to one level:
    copyLeavesFromStateGraphListToRenderLeafList();

    // apply the cull program:
    state.apply(_cullSS.get());

    // draw top-level render leaves
    {
        OE_GL_ZONE_NAMED("cull");
        for (auto& leaf : _renderLeafList)
        {
            auto d = static_cast<const ChonkDrawable*>(leaf->getDrawable());

            state.applyModelViewMatrix(leaf->_modelview);
            state.applyProjectionMatrix(leaf->_projection);

            d->update_and_cull_batches(state);
        }
    }

    // render all leaves
    {
        OE_GL_ZONE_NAMED("draw");

        GLVAO::Ptr vao;
        for (auto& leaf : _renderLeafList)
        {
            auto d = static_cast<const ChonkDrawable*>(leaf->getDrawable());

            if (!vao)
            {
                vao = d->_gs[state.getContextID()]._vao;

                OE_HARD_ASSERT(vao != nullptr);

                vao->bind();
                vao->ext()->glMemoryBarrier(
                    GL_SHADER_STORAGE_BARRIER_BIT | GL_COMMAND_BARRIER_BIT);
            }

            state.applyModelViewMatrix(leaf->_modelview);
            state.applyProjectionMatrix(leaf->_projection);

            robert(state, leaf, previous);

            d->draw_batches(state);
        }

        if (vao) vao->unbind();
    }

    if (_stateset.valid())
    {
        state.removeStateSet(insertPos);
    }
}
