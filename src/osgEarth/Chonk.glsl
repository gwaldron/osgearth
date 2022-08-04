#pragma vp_function oe_chonk_default_vertex_model, vertex_model, 0.0
#pragma import_defines(OE_IS_SHADOW_CAMERA)

struct Instance {
    mat4 xform;
    vec2 local_uv;
    uint lod;
    float visibility[4];
    uint first_variant_cmd_index;
};
layout(binding = 0, std430) buffer Instances {
    Instance instances[];
};
layout(binding = 1, std430) buffer TextureArena {
    uint64_t textures[];
};

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec4 color;
layout(location = 3) in vec2 uv;
layout(location = 4) in vec3 flex;
layout(location = 5) in int albedo; // todo: material LUT index
layout(location = 6) in int normalmap; // todo: material LUT index

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

    uint lod = instances[i].lod;

#ifndef OE_IS_SHADOW_CAMERA
    oe_fade = instances[i].visibility[lod];
#else
    oe_fade = 1.0;
#endif

    vertex = instances[i].xform * vec4(position, 1.0);
    vp_Color = color;
    xform3 = mat3(instances[i].xform);
    vp_Normal = xform3 * normal;
    oe_tex_uv = uv;
    oe_albedo_tex = albedo >= 0 ? textures[albedo] : 0;
    oe_normal_tex = normalmap >= 0 ? textures[normalmap] : 0;
}

[break]
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
        vec3 ZAXIS = gl_NormalMatrix * vec3(0, 0, 1);
        if (dot(ZAXIS, vp_Normal) > 0.95)
            oe_tangent = gl_NormalMatrix * (xform3 * vec3(1, 0, 0));
        else
            oe_tangent = cross(ZAXIS, vp_Normal);
    }
}

[break]
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
    else if (oe_fade <= 2.0) color.rgb = vec3(1, 0, 0);
    else if (oe_fade <= 3.0) color.rgb = vec3(1, 1, 0);
    else if (oe_fade <= 4.0) color.rgb = vec3(0, 1, 0);
    else color.rgb = vec3(1, 0, 1); // should never happen :)
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
        n.xyz = normalize(n.xyz*2.0 - 1.0);
#endif

        // construct the TBN, reflecting the normal on back-facing polys
        mat3 tbn = mat3(
            normalize(oe_tangent),
            normalize(cross(vp_Normal, oe_tangent)),
            vp_Normal);
            //normalize(gl_FrontFacing ? vp_Normal : -vp_Normal));

        vp_Normal = normalize(tbn * n.xyz);
    }
}
