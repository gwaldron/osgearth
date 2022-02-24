#version 460
#extension GL_ARB_gpu_shader_int64 : enable
#pragma vp_function oe_vegetation_vs_model, vertex_model

struct Instance {
    mat4 xform;
    vec2 local_uv;
    float fade;
    float visibility[4];
    uint first_variant_cmd_index;
};
layout(binding = 0, std430) buffer Instances {
    Instance instances[];
};

// stage globals
mat3 vec3xform;

void oe_vegetation_vs_model(inout vec4 vertex)
{
    int i = gl_BaseInstance + gl_InstanceID;
    vec3xform = mat3(instances[i].xform);
};


[break]

#version 460
#extension GL_ARB_gpu_shader_int64 : enable
#pragma vp_function oe_vegetation_vs_view, vertex_view
#pragma import_defines(OE_WIND_TEX)
#pragma import_defines(OE_WIND_TEX_MATRIX)

struct Instance {
    mat4 xform;
    vec2 local_uv;
    float fade;
    float visibility[4];
    uint first_variant_cmd_index;
};
layout(binding = 0, std430) buffer Instances {
    Instance instances[];
};

layout(location = 4) in vec3 flex;

// outputs
flat out uint64_t oe_normal_tex;

// stage globals
mat3 vec3xform; // set in model function

#ifdef OE_WIND_TEX
uniform sampler3D OE_WIND_TEX;
uniform mat4 OE_WIND_TEX_MATRIX;
uniform float osg_FrameTime;
uniform sampler2D oe_veg_noise;

uniform float oe_wind_power = 1.0;

#define remap(X, LO, HI) (LO + X * (HI - LO))

void oe_apply_wind(inout vec4 vertex, in vec2 local_uv)
{
    float flexibility = length(flex);
    if (flexibility > 0.0 && oe_wind_power > 0.0)
    {
        vec4 wind = textureProj(OE_WIND_TEX, (OE_WIND_TEX_MATRIX * vertex));
        vec3 wind_dir = normalize(wind.rgb * 2 - 1); // view space
        const float rate = 0.01;
        vec4 noise_moving = textureLod(oe_veg_noise, local_uv + osg_FrameTime * rate, 0);
        float speed_var = remap(noise_moving[3], -0.2, 1.4);
        float speed = wind.a * speed_var;
        vec3 bend_vec = wind_dir * speed;
        vec3 flex_dir = normalize(gl_NormalMatrix * vec3xform * flex);
        float flex_planar = abs(dot(wind_dir, flex_dir));
        flex_planar = 1.0 - (flex_planar*flex_planar);
        vertex.xyz += bend_vec * flex_planar * flexibility * oe_wind_power;
    }
}
#endif

void oe_vegetation_vs_view(inout vec4 vertex)
{
#ifdef OE_WIND_TEX
    int i = gl_BaseInstance + gl_InstanceID;
    oe_apply_wind(vertex, instances[i].local_uv);
#endif
}


[break]

#version 430
#extension GL_ARB_gpu_shader_int64 : enable
#pragma vp_function oe_vegetation_fs, fragment
#pragma import_defines(OE_USE_ALPHA_TO_COVERAGE)
#pragma import_defines(OE_IS_SHADOW_CAMERA)
#pragma import_defines(OE_COMPRESSED_NORMAL)

in vec2 oe_tex_uv;
flat in uint64_t oe_albedo_tex;
in vec3 vp_VertexView;

void oe_vegetation_fs(inout vec4 color)
{
#ifdef OE_IS_SHADOW_CAMERA
    if (color.a < 0.15)
        discard;
#else
    // alpha-down faces that are orthogonal to the view vector.
    // this makes cross-hatch imposters look better
#if 0
    vec3 face_normal = normalize(cross(dFdx(vp_VertexView), dFdy(vp_VertexView)));
    const float edge_factor = 0.8;
    float d = clamp(edge_factor*abs(dot(face_normal, normalize(vp_VertexView))), 0.0, 1.0);
    color.a *= d;
#endif

#ifdef OE_USE_ALPHA_TO_COVERAGE
    // mitigate the screen-door effect of A2C in the distance
    // https://tinyurl.com/y7bbbpl9
    //const float threshold = 0.15;
    //float a = (color.a - threshold) / max(fwidth(color.a), 0.0001) + 0.5;
    //color.a = mix(color.a, a, unit_distance_to_vert);

    // adjust the alpha based on the calculated mipmap level:
    // better, but a bit more expensive than the above method? Benchmark?
    // https://tinyurl.com/fhu4zdxz
    if (oe_albedo_tex > 0UL)
    {
        ivec2 tsize = textureSize(sampler2D(oe_albedo_tex), 0);
        vec2 cf = vec2(float(tsize.x)*oe_tex_uv.s, float(tsize.y)*oe_tex_uv.t);
        vec2 dx_vtc = dFdx(cf);
        vec2 dy_vtc = dFdy(cf);
        float delta_max_sqr = max(dot(dx_vtc, dx_vtc), dot(dy_vtc, dy_vtc));
        float mml = max(0, 0.5 * log2(delta_max_sqr));
        color.a *= (1.0 + mml * 0.25);
    }
#else
    // force alpha to 0 or 1 and threshold it.
    const float threshold = 0.15;
    color.a = step(threshold, color.a);
    if (color.a < threshold)
        discard;
#endif // OE_USE_ALPHA_TO_COVERAGE
#endif // OE_IS_SHADOW_CAMERA
}
