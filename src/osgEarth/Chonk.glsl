#pragma vp_function oe_chonk_default_vertex_model, vertex_model, 0.0
#pragma import_defines(OE_IS_SHADOW_CAMERA)
#pragma import_defines(OE_IS_DEPTH_CAMERA)
#pragma import_defines(OE_CHONK_MAX_LOD_FOR_NORMAL_MAPS)
#pragma import_defines(OE_CHONK_MAX_LOD_FOR_PBR_MAPS)

#ifndef OE_CHONK_MAX_LOD_FOR_NORMAL_MAPS
#define OE_CHONK_MAX_LOD_FOR_NORMAL_MAPS 99
#endif

#ifndef OE_CHONK_MAX_LOD_FOR_PBR_MAPS
#define OE_CHONK_MAX_LOD_FOR_PBR_MAPS 99
#endif

struct Instance
{
    mat4 xform;
    vec2 local_uv;
    uint lod;
    float visibility[2]; // per LOD
    float radius;
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
layout(location = 2) in uint normal_technique;
layout(location = 3) in vec4 color;
layout(location = 4) in vec2 uv;
layout(location = 5) in vec3 flex;
layout(location = 6) in int albedo_index;
layout(location = 7) in int normalmap_index;
layout(location = 8) in int pbr_index;

// stage global
mat3 xform3;
uint chonk_lod;

// outputs
out vec3 vp_Normal;
out vec4 vp_Color;
out float oe_fade;
out vec2 oe_tex_uv;
out vec3 oe_position_vec;
flat out float oe_alpha_cutoff;
flat out uint64_t oe_albedo_tex;
flat out uint64_t oe_normal_tex;
flat out uint64_t oe_pbr_tex;

void oe_chonk_default_vertex_model(inout vec4 vertex)
{
    int i = gl_BaseInstance + gl_InstanceID;

    chonk_lod = instances[i].lod;

    vertex = instances[i].xform * vec4(position, 1.0);
    vp_Color = color;
    xform3 = mat3(instances[i].xform);
    vp_Normal = xform3 * normal;
    oe_tex_uv = uv;
    oe_alpha_cutoff = instances[i].alpha_cutoff;
    oe_fade = instances[i].visibility[chonk_lod];
    oe_albedo_tex = albedo_index >= 0 ? textures[albedo_index] : 0;

#if defined(OE_IS_SHADOW_CAMERA) || defined(OE_IS_DEPTH_CAMERA)
    oe_fade = 1.0;
    return;
#endif

    // stuff we need only for a non-depth or non-shadow camera

    // Position vector scaled by the (scaled) radius of the instance
    oe_position_vec = (xform3 * position.xyz) / instances[i].radius;

    // disable/ignore normal maps as directed:
    oe_normal_tex = 0;
    if (normalmap_index >= 0 && chonk_lod <= OE_CHONK_MAX_LOD_FOR_NORMAL_MAPS)
    {
        oe_normal_tex = textures[normalmap_index];
    }

    oe_pbr_tex = 0;
    if (pbr_index >= 0 && chonk_lod <= OE_CHONK_MAX_LOD_FOR_PBR_MAPS)
    {
        oe_pbr_tex = textures[pbr_index];
    }
}


[break]
#pragma vp_function oe_chonk_default_vertex_view, vertex_view, 0.0

layout(location = 2) in uint normal_technique;
#define NT_DEFAULT 0
#define NT_ZAXIS 1
#define NT_HEMISPHERE 2 

// stage global
mat3 xform3; // set in model stage
uint chonk_lod; // set in model stage

// output
out vec3 vp_Normal;
//out vec3 oe_tangent;
out vec3 oe_position_vec;
flat out uint oe_normal_technique;
flat out uint64_t oe_normal_tex;

// amount to warp billboarded normals away from the eye [0..1]
const float oe_chonk_billboarded_normal_threshold = 0.65;

void oe_chonk_default_vertex_view(inout vec4 vertex)
{
    // propagate to frag shader:
    oe_normal_technique = normal_technique;

    // for the hemispheric normals we need a 2D position vector.
    if (oe_normal_technique == NT_HEMISPHERE)
    {
        oe_position_vec = gl_NormalMatrix * oe_position_vec;
    }

#if 0
    if (oe_normal_tex > 0)
    {
        if (oe_normal_technique == NT_DEFAULT)
        {
            vec3 ZAXIS = gl_NormalMatrix * vec3(0, 0, 1);
            if (dot(ZAXIS, vp_Normal) > 0.95)
                oe_tangent = gl_NormalMatrix * (xform3 * vec3(1, 0, 0));
            else
                oe_tangent = cross(ZAXIS, vp_Normal);
        }

        else
        {
            oe_tangent = gl_NormalMatrix * (xform3 * vec3(1, 0, 0));
        }
    }
#endif
}

[break]
#pragma vp_function oe_chonk_default_fragment, fragment
#pragma import_defines(OE_IS_SHADOW_CAMERA)
#pragma import_defines(OE_IS_DEPTH_CAMERA)
#pragma import_defines(OE_USE_ALPHA_TO_COVERAGE)
#pragma import_defines(OE_GL_RG_COMPRESSED_NORMALS)
#pragma import_defines(OE_GPUCULL_DEBUG)
#pragma import_defines(OE_CHONK_SINGLE_SIDED)

struct OE_PBR {
    float roughness;
    float ao;
    float metal;
    float brightness;
    float contrast;
} oe_pbr;

// inputs
in vec3 oe_UpVectorView;
in vec3 vp_Normal;
//in vec3 oe_tangent;
in vec3 oe_position_vec;
in vec2 oe_tex_uv;
in float oe_fade;
flat in uint64_t oe_albedo_tex;
flat in uint64_t oe_normal_tex;
flat in uint64_t oe_pbr_tex;
flat in float oe_alpha_cutoff;

flat in uint oe_normal_technique;
#define NT_DEFAULT 0
#define NT_ZAXIS 1
#define NT_HEMISPHERE 2 

uniform float oe_normal_attenuation = 0.65;

void oe_chonk_default_fragment(inout vec4 color)
{
    // When simulating normals, we invert the texture coordinates
    // for backfacing geometry
    //if (oe_normal_technique != NT_DEFAULT && !gl_FrontFacing)
    //{
        //oe_tex_uv.s = 1.0 - oe_tex_uv.s;
    //}

    // Apply the base color:
    if (oe_albedo_tex > 0)
    {
        color *= texture(sampler2D(oe_albedo_tex), oe_tex_uv);
    }

#if defined(OE_IS_SHADOW_CAMERA) || defined(OE_IS_DEPTH_CAMERA)

    // force alpha to 0 or 1 and threshold it.
    color.a = step(oe_alpha_cutoff, color.a * oe_fade);
    if (color.a < oe_alpha_cutoff)
        discard;

#else // !OE_IS_SHADOW_CAMERA && !OE_IS_DEPTH_CAMERA

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

#ifdef OE_CHONK_SINGLE_SIDED
    bool flip_backfacing_normal = false;
#else
    bool flip_backfacing_normal = true;
#endif

    vec3 face_normal = vp_Normal;
    vec3 face_up = gl_NormalMatrix * vec3(0, 0, 1);

    // for billboarded normals, adjust the normal so its coverage
    // is a hemisphere facing the viewer. Should we recalculate the TBN here?
    // Probably, but let's not if it already looks good enough.
    if (oe_normal_technique == NT_HEMISPHERE)
    {
        vec3 v3d = oe_position_vec; // do not normalize!
        vec3 v2d = vec3(v3d.x, v3d.y, 0.0);
        float size2d = length(v2d) * 1.2021; // adjust for radius, bbox diff
        size2d = mix(0.0, oe_normal_attenuation, clamp(size2d, 0.0, 1.0));
        face_normal = mix(vec3(0, 0, 1), normalize(v2d), size2d);
    }

    vec3 face_tangent = cross(face_up, face_normal);
    vec3 pixel_normal = vec3(0, 0, 1);

    if (oe_normal_tex > 0)
    {
        vec4 n = texture(sampler2D(oe_normal_tex), oe_tex_uv);

      #ifdef OE_GL_RG_COMPRESSED_NORMALS
        n.xy = n.xy*2.0 - 1.0;
        n.z = 1.0 - abs(n.x) - abs(n.y);
        float t = clamp(-n.z, 0, 1);
        n.x += (n.x > 0) ? -t : t;
        n.y += (n.y > 0) ? -t : t;
        pixel_normal = n.xyz;
      #else
        pixel_normal = normalize(n.xyz*2.0 - 1.0);
      #endif
    }

    //pixel_normal = vec3(0, 0, 1); // testing

    mat3 tbn = mat3(
        normalize(face_tangent),
        -normalize(cross(face_normal, face_tangent)),
        normalize(face_normal));

    vp_Normal = normalize(tbn * pixel_normal);

    if (flip_backfacing_normal && vp_Normal.z < 0.0)
    {
        vp_Normal.z = -vp_Normal.z;
    }

    if (oe_normal_technique == NT_ZAXIS)
    {
        vp_Normal = normalize(mix(vp_Normal, face_up, oe_normal_attenuation));
    }

    // apply PBR maps:
    if (oe_pbr_tex > 0)
    {
        vec4 texel = texture(sampler2D(oe_pbr_tex), oe_tex_uv);
        oe_pbr.metal *= texel[0];
        oe_pbr.roughness *= (1.0 - texel[1]);
        oe_pbr.ao *= texel[2];
    }

#endif // !OE_IS_SHADOW_CAMERA
}
