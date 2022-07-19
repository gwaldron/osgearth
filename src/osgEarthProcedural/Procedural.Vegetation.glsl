#pragma vp_function oe_vegetation_vs_model, vertex_model

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

// stage globals
mat3 vec3xform;

void oe_vegetation_vs_model(inout vec4 vertex)
{
    int i = gl_BaseInstance + gl_InstanceID;
    vec3xform = mat3(instances[i].xform);
};


[break]

#pragma vp_function oe_vegetation_vs_view, vertex_view
#pragma import_defines(OE_WIND_TEX)
#pragma import_defines(OE_WIND_TEX_MATRIX)

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

layout(location = 4) in vec3 flex;

// outputs
flat out uint64_t oe_normal_tex;
flat out uint oe_lod;

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
        // sample the wind texture
        vec4 wind = textureProj(OE_WIND_TEX, (OE_WIND_TEX_MATRIX * vertex));

        // add noise into the wind speed
        const float rate = 0.01;
        vec4 noise_moving = textureLod(oe_veg_noise, local_uv + sin(osg_FrameTime * rate), 0);
        float speed_var = remap(noise_moving[3], -0.2, 1.4);
        float speed = wind.a * speed_var;

        // project the wind vector onto the flex plane
        vec3 wind_vec = normalize(wind.rgb * 2.0 - 1.0) * speed;
        vec3 flex_plane_normal = normalize(gl_NormalMatrix * vec3xform * flex);

        float dist = dot(wind_vec, flex_plane_normal);
        vec3 wind_vec_projected = wind_vec - flex_plane_normal * dist;

        // move the vertex within the flex plane
        vertex.xyz += wind_vec_projected * flexibility * oe_wind_power;
    }
}
#endif

void oe_vegetation_vs_view(inout vec4 vertex)
{
    int i = gl_BaseInstance + gl_InstanceID;
    oe_lod = instances[i].lod;

#ifdef OE_WIND_TEX
    oe_apply_wind(vertex, instances[i].local_uv);
#endif
}


[break]
#pragma vp_function oe_vegetation_fs, fragment
#pragma import_defines(OE_USE_ALPHA_TO_COVERAGE)
#pragma import_defines(OE_IS_SHADOW_CAMERA)
#pragma import_defines(OE_COMPRESSED_NORMAL)

in vec2 oe_tex_uv;
flat in uint64_t oe_albedo_tex;
flat in uint oe_lod;
in vec3 vp_VertexView;

#define OE_A2C_ADJUSTMENT 0.275

void oe_vegetation_fs(inout vec4 color)
{
#ifdef OE_IS_SHADOW_CAMERA
    if (color.a < 0.15)
        discard;
#else

    // alpha-down faces that are orthogonal to the view vector.
    // this makes cross-hatch imposters look better.
    // (only do this for lower lods)
    if (oe_lod > 0)
    {
        vec3 dx = dFdx(vp_VertexView);
        vec3 dy = dFdy(vp_VertexView);
        vec3 fn = normalize(cross(dx, dy));
        float facey = abs(dot(fn, normalize(vp_VertexView)));
        const float threshold = 0.5;
        if (facey < threshold) {
            color.a *= clamp(pow(facey / threshold, 4.0), 0, 1);
        }
    }

#ifdef OE_USE_ALPHA_TO_COVERAGE

    // Adjust the alpha based on the calculated mipmap level.
    // Looks beterr and actually helps performance a bit as well.
    // https://tinyurl.com/fhu4zdxz
    if (oe_albedo_tex > 0UL)
    {
        ivec2 tsize = textureSize(sampler2D(oe_albedo_tex), 0);
        vec2 cf = vec2(float(tsize.x)*oe_tex_uv.s, float(tsize.y)*oe_tex_uv.t);
        vec2 dx_vtc = dFdx(cf);
        vec2 dy_vtc = dFdy(cf);
        float delta_max_sqr = max(dot(dx_vtc, dx_vtc), dot(dy_vtc, dy_vtc));
        float mml = max(0, 0.5 * log2(delta_max_sqr));
        color.a *= (1.0 + mml * OE_A2C_ADJUSTMENT);
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
