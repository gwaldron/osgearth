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
#include "Utils"

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
#pragma import_defines(OE_GPUCULL_DEBUG)
#pragma import_defines(OE_IS_SHADOW_CAMERA)

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
uniform vec4 oe_lod_scale;

#if OE_GPUCULL_DEBUG
//#ifdef OE_GPUCULL_DEBUG
#define REJECT(X) if (fade==1.0) { fade=(X);}
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

    // bail if our chonk does not have this LOD
    uint v = input_instances[i].first_lod_cmd_index + lod;
    if (lod >= chonks[v].num_lods)
        return;

#ifdef OE_IS_SHADOW_CAMERA
    // only the lowest LOD for shadow-casting.
    if (lod < chonks[v].num_lods-1)
        return;
#endif

    // intialize:
    float fade = 1.0;

    // transform the bounding sphere to a view-space bbox.
    mat4 xform = input_instances[i].xform;
    vec4 center = xform * vec4(chonks[v].bs.xyz, 1);
    vec4 center_view = gl_ModelViewMatrix * center;

    float max_scale = max(xform[0][0], max(xform[1][1], xform[2][2]));
    float r = chonks[v].bs.w * max_scale;

    // Trivially accept (at the highest LOD) anything whose bounding sphere
    // intersects the near clip plane:
    bool is_perspective = gl_ProjectionMatrix[3][3] < 0.01;
    float near = gl_ProjectionMatrix[2][3] / (gl_ProjectionMatrix[2][2]-1.0);
    if (is_perspective && -(center_view.z + r) <= near)
    {
        if (lod > 0) { // reject all lower LODs
            REJECT(REASON_NEARCLIP);
        }
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

#if OE_GPUCULL_DEBUG
        float threshold = 0.75;
#else
        float threshold = 1.0;
#endif

        if (LL.x > threshold || LL.y > threshold)
            REJECT(REASON_FRUSTUM);

        if (UR.x < -threshold || UR.y < -threshold)
            REJECT(REASON_FRUSTUM);

#ifndef OE_IS_SHADOW_CAMERA

        // OK, it is in view - now check pixel size on screen for this LOD:
        vec2 dims = 0.5*(UR.xy-LL.xy)*oe_Camera.xy;
    
        float pixelSize = max(dims.x, dims.y);
        float pixelSizePad = pixelSize*0.1;

#if 0
        float minPixelSize = oe_sse * chonks[v].far_pixel_scale;
        if (pixelSize < (minPixelSize - pixelSizePad))
            REJECT(REASON_SSE);

        float maxPixelSize = oe_sse * chonks[v].near_pixel_scale;
        if (pixelSize > (maxPixelSize + pixelSizePad))
            REJECT(REASON_SSE);
#else
        float minPixelSize = oe_sse * chonks[v].far_pixel_scale * oe_lod_scale[lod];
        if (pixelSize < (minPixelSize - pixelSizePad))
            REJECT(REASON_SSE);

        float near_scale = lod > 0 ? chonks[v].near_pixel_scale * oe_lod_scale[lod-1] : 99999.0;
        float maxPixelSize = oe_sse * near_scale;
        if (pixelSize > (maxPixelSize + pixelSizePad))
            REJECT(REASON_SSE);
#endif

        if (fade==1.0)  // good to go, set the proper fade:
        {
            if (pixelSize > maxPixelSize)
                fade = 1.0-(pixelSize-maxPixelSize)/pixelSizePad;
            else if (pixelSize < minPixelSize)
                fade = 1.0-(minPixelSize-pixelSize)/pixelSizePad;
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
    if (fade < 0.1)
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
#if OE_GPUCULL_DEBUG
    if (oe_fade <= 1.0) color.a *= oe_fade;
    else if (oe_fade <= 2.0) color.rgb = vec3(1,0,0);
    else if (oe_fade <= 3.0) color.rgb = vec3(1,1,0);
    else if (oe_fade <= 4.0) color.rgb = vec3(0,1,0);
    else color.rgb = vec3(1,0,1); // should never happen :)
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
        ChonkFactory::GetOrCreateFunction _getOrCreateTexture;
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

        Ripper(Chonk& chonk, TextureArena* textures, ChonkFactory::GetOrCreateFunction func) :
            _result(chonk),
            _textures(textures),
            _getOrCreateTexture(func)
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

            Texture::Ptr arena_tex;

            osg::Texture* tex = dynamic_cast<osg::Texture*>(
                stateset->getTextureAttribute(slot, osg::StateAttribute::TEXTURE));

            if (tex && tex->getImage(0))
            {
                auto i = _textureLUT.find(tex);

                if (i == _textureLUT.end())
                {
                    bool isNew = true;

                    if (_getOrCreateTexture)
                        arena_tex = _getOrCreateTexture(tex, isNew);
                    else
                        arena_tex = Texture::create();

                    if (isNew)
                    {
                        arena_tex->_image = tex->getImage(0);
                        arena_tex->_uri = tex->getImage(0)->getFileName();
                        arena_tex->_label = "Chonk texture";
                    }

                    int index = _textures->add(arena_tex);
                    if (index >= 0)
                    {
                        _textureLUT[tex] = arena_tex;
                    }
                }
                else
                {
                    arena_tex = i->second;
                }
            }

            return arena_tex;
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
ChonkFactory::setGetOrCreateFunction(GetOrCreateFunction value)
{
    _getOrCreateTexture = value;
}

void
ChonkFactory::load(
    osg::Node* node,
    Chonk& chonk)
{
    OE_PROFILING_ZONE;

    // convert all primitive sets to GL_TRIANGLES
    osgUtil::Optimizer o;
    o.optimize(node, o.INDEX_MESH);
    
    // first count up the memory we need and allocate it
    Counter counter;
    node->accept(counter);
    chonk._vbo_store.reserve(counter._numVerts);
    chonk._ebo_store.reserve(counter._numElements);

    // rip geometry and textures into a new Asset object
    Ripper ripper(chonk, _textures.get(), _getOrCreateTexture);
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
}

ChonkDrawable::ChonkDrawable() :
    osg::Drawable(),
    _proxy_dirty(true),
    _gpucull(true)
{
    setName(typeid(*this).name());
    setUseDisplayList(false);
    setUseVertexBufferObjects(false);
    setUseVertexArrayObject(false);

    installRenderBin(this);
}

ChonkDrawable::~ChonkDrawable()
{
    //nop
}

void
ChonkDrawable::installRenderBin(ChonkDrawable* d)
{
    static osg::ref_ptr<osg::StateSet> s_ss;
    static osg::ref_ptr<VirtualProgram> s_vp;
    static Mutex s_mutex;

    if (!s_vp.valid())
    {
        ScopedMutexLock lock(s_mutex);
        if (!s_vp.valid())
        {
            s_ss = new osg::StateSet();
            s_ss->setDataVariance(s_ss->STATIC);
            s_ss->setRenderBinDetails(818, "ChonkBin");

            s_vp = VirtualProgram::getOrCreate(s_ss.get());
            s_vp->setName("ChonkDrawable");
            s_vp->addGLSLExtension("GL_ARB_gpu_shader_int64");
            ShaderLoader::load(s_vp.get(), oe_chonk_default_shaders);
        }
    }

    d->setStateSet(s_ss.get());    
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

        // flag all graphics states as requiring an update:
        for (unsigned i = 0; i < _gs.size(); ++i)
            _gs[i]._dirty = true;

        // flag the bounds for recompute
        dirtyBound();
    }
}

void
ChonkDrawable::setModelViewMatrix(const osg::Matrix& value)
{
    _mvm = value;
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
    OE_HARD_ASSERT(false, "ChonkRenderBin::drawImplementation should never be called WHAT ARE YOU DOING");
}

void
ChonkDrawable::update_and_cull_batches(osg::State& state) const
{
    GCState& gs = _gs[state.getContextID()];

    if (gs._dirty)
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
    OE_GL_ZONE_NAMED("update");

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

    // no need to do this since it gets sent in cull()
    //_commandBuf->uploadData(_commands);

    // Send to the GPU:
    _instanceInputBuf->uploadData(_all_instances, GL_STATIC_DRAW);

    if (_cull)
    {
        _chonkBuf->uploadData(_chonk_lods, GL_STATIC_DRAW);
        
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

    OE_GL_ZONE_NAMED("cull");

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
    _commandBuf->uploadData(_commands);

    _instanceOutputBuf->bindBufferBase(0);
    _commandBuf->bindBufferBase(29);
    _chonkBuf->bindBufferBase(30);
    _instanceInputBuf->bindBufferBase(31);

    // 2-pass culling compute. 
    // Note. I tried separating this out so that all "pass ones"
    // would run, and thena ll "pass twos" afterwards, in an attempt
    // to avoid the memory barrier and multiple uniform sets.
    // It was slower. Maybe because if was offset by having to 
    // double-set the the matrix uniforms and the bindBufferBase
    // calls for each tile.
    // Also, removing the memory barrier seems to make no difference,
    // but it's the right thing to do

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
    OE_GL_ZONE_NAMED("draw");

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

    _cullSS = new osg::StateSet();

    // culling program
    osg::Program* p = new osg::Program();
    p->addShader(new osg::Shader(
        osg::Shader::COMPUTE,
        s_chonk_cull_compute_shader));
    _cullSS->setAttribute(p);

    // Default far pixel scales per LOD.
    // We expect this to be overriden from above.
    _cullSS->addUniform(new osg::Uniform("oe_lod_scale", osg::Vec4f(1, 1, 1, 1)));
}

ChonkRenderBin::ChonkRenderBin(const ChonkRenderBin& rhs, const osg::CopyOp& op) :
    osgUtil::RenderBin(rhs, op),
    _cullSS(rhs._cullSS)
{
    // for each render bin instance, create a StateGraph that we
    // will use to track the OSG state properly when applying the
    // cull program
    _cull_sg = new osgUtil::StateGraph();
    _cull_sg->_stateset = _cullSS.get();
}

namespace
{
    void apply_state_with_roberts_blessing(
        osg::State& state,
        osgUtil::StateGraph* new_rg,
        osgUtil::StateGraph* prev_rg)
    {
        // mysterious state-fu adapted from osgUtil::RenderLeaf
        if (prev_rg)
        {
            osgUtil::StateGraph* prev_rg_parent = prev_rg->_parent;
            osgUtil::StateGraph* rg = new_rg;
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
            osgUtil::StateGraph::moveStateGraph(state, nullptr, new_rg->_parent);
            state.apply(new_rg->getStateSet());
        }
    }
}


ChonkRenderBin::CullLeaf::CullLeaf(osgUtil::RenderLeaf* leaf) :
    CustomRenderLeaf(leaf)
{
    //nop
}

void 
ChonkRenderBin::CullLeaf::draw(osg::State& state)
{
    auto d = static_cast<const ChonkDrawable*>(getDrawable());
    d->update_and_cull_batches(state);
}

ChonkRenderBin::DrawLeaf::DrawLeaf(osgUtil::RenderLeaf* leaf, bool first, bool last) :
    CustomRenderLeaf(leaf),
    _first(first),
    _last(last)
{
    //nop
}

void
ChonkRenderBin::DrawLeaf::draw(osg::State& state)
{
    auto d = static_cast<const ChonkDrawable*>(getDrawable());

    if (_first)
    {
        GLVAO::Ptr vao = d->_gs[state.getContextID()]._vao;
        vao->bind();
        vao->ext()->glMemoryBarrier(
            GL_SHADER_STORAGE_BARRIER_BIT | GL_COMMAND_BARRIER_BIT);
    }

    d->draw_batches(state);

    if (_last)
    {
        GLVAO::Ptr vao = d->_gs[state.getContextID()]._vao;
        vao->unbind();
    }

}

void
ChonkRenderBin::drawImplementation(
    osg::RenderInfo& ri,
    osgUtil::RenderLeaf*& previous)
{
    OE_GL_ZONE_NAMED("ChonkRenderBin");

    OE_SOFT_ASSERT(
        _stateGraphList.size() == 1,
        "StateGraphList is not size=1, you need to write some code");

    // copy everything to one level:
    copyLeavesFromStateGraphListToRenderLeafList();
    if (_renderLeafList.empty())
        return;

    osg::State& state = *ri.getState();

    // rearrange the bin so we have cull leaves followed by draw leaves,
    // each with the proper shader.

    // draw graph:
    osgUtil::StateGraph* draw_sg = _renderLeafList.front()->_parent;
    draw_sg->_leaves.clear();

    // cull graph:
    _cull_sg->_parent = draw_sg;
    _cull_sg->_leaves.clear();

    for(unsigned i=0; i<_renderLeafList.size(); ++i)
    {
        auto leaf = _renderLeafList[i];
        bool first = (i == 0);
        bool last = (i == _renderLeafList.size() - 1);

        _cull_sg->addLeaf(new CullLeaf(leaf));
        draw_sg->addLeaf(new DrawLeaf(leaf, first, last));
    }

    // install the new state graphs:
    _renderLeafList.clear();
    _stateGraphList.clear();
    _stateGraphList.push_back(_cull_sg.get());
    _stateGraphList.push_back(draw_sg);

    // dispatch.
    osgUtil::RenderBin::drawImplementation(ri, previous);
}
