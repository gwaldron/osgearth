#pragma vp_function oe_chonk_default_vertex_model, vertex_model, 0.0
#pragma import_defines(OE_IS_SHADOW_CAMERA)

struct Instance {
    mat4 xform;
    vec2 local_uv;
    uint lod;
    float visibility[3]; // per LOD
    float alpha_cutoff;
    uint first_lod_cmd_index;
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
flat out float oe_alpha_cutoff;

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
    oe_alpha_cutoff = instances[i].alpha_cutoff;
}


[break]
#pragma vp_function oe_chonk_default_vertex_view, vertex_view, 0.0

// stage
mat3 xform3; // set in vertex_model

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
#pragma vp_function oe_chonk_default_fragment, fragment
#pragma import_defines(OE_IS_SHADOW_CAMERA)
#pragma import_defines(OE_IS_DEPTH_CAMERA)
#pragma import_defines(OE_USE_ALPHA_TO_COVERAGE)
#pragma import_defines(OE_COMPRESSED_NORMAL)
#pragma import_defines(OE_GPUCULL_DEBUG)

// inputs
in float oe_fade;
in vec2 oe_tex_uv;
in vec3 oe_tangent;
in vec3 vp_Normal;
flat in uint64_t oe_albedo_tex;
flat in uint64_t oe_normal_tex;
flat in float oe_alpha_cutoff;

void oe_chonk_default_fragment(inout vec4 color)
{
    if (oe_albedo_tex > 0)
    {
        vec4 texel = texture(sampler2D(oe_albedo_tex), oe_tex_uv);
        color *= texel;
    }

#if defined(OE_IS_SHADOW_CAMERA) || defined(OE_IS_DEPTH_CAMERA)

    // force alpha to 0 or 1 and threshold it.
    color.a = step(oe_alpha_cutoff, color.a * oe_fade);
    if (color.a < oe_alpha_cutoff)
        discard;

#else // !OE_IS_SHADOW_CAMERA

  #if OE_GPUCULL_DEBUG

    // apply the high fade from the instancer
    if (oe_fade <= 1.0) color.a *= oe_fade;
    else if (oe_fade <= 2.0) color.rgb = vec3(1, 0, 0);
    else if (oe_fade <= 3.0) color.rgb = vec3(1, 1, 0);
    else if (oe_fade <= 4.0) color.rgb = vec3(0, 1, 0);
    else color.rgb = vec3(1, 0, 1); // should never happen :)

  #elif defined(OE_USE_ALPHA_TO_COVERAGE)

    // Adjust the alpha based on the calculated mipmap level.
    // Looks better and actually helps performance a bit as well.
    // https://bgolus.medium.com/anti-aliased-alpha-test-the-esoteric-alpha-to-coverage-8b177335ae4f
    // https://tinyurl.com/fhu4zdxz
    if (oe_albedo_tex > 0UL)
    {
        //color.a = (color.a - oe_veg_alphaCutoff) / max(fwidth(color.a), 0.0001) + 0.5;
        ivec2 tsize = textureSize(sampler2D(oe_albedo_tex), 0);
        vec2 cf = vec2(float(tsize.x)*oe_tex_uv.s, float(tsize.y)*oe_tex_uv.t);
        vec2 dx_vtc = dFdx(cf);
        vec2 dy_vtc = dFdy(cf);
        float delta_max_sqr = max(dot(dx_vtc, dx_vtc), dot(dy_vtc, dy_vtc));
        float miplevel = max(0, 0.5 * log2(delta_max_sqr));

        color.a *= (1.0 + miplevel * oe_alpha_cutoff);
    }

    color.a *= oe_fade;

  #else // if !OE_USE_ALPHA_TO_COVERAGE

    // force alpha to 0 or 1 and threshold it.
    color.a = step(oe_alpha_cutoff, color.a * oe_fade);
    if (color.a < oe_alpha_cutoff)
        discard;

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

        // construct the TBN
        mat3 tbn = mat3(
            normalize(oe_tangent),
            normalize(cross(vp_Normal, oe_tangent)),
            vp_Normal);
        //normalize(gl_FrontFacing ? vp_Normal : -vp_Normal));

        vp_Normal = normalize(tbn * n.xyz);
    }

#endif // !OE_IS_SHADOW_CAMERA
}
