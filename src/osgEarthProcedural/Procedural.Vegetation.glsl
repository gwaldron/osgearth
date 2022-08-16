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

#pragma import_defines(OE_TWEAKABLE)
#ifdef OE_TWEAKABLE
#define tweakable uniform
#else
#define tweakable const
#endif
tweakable float oe_wind_power = 1.0;

#define remap(X, LO, HI) (LO + X * (HI - LO))

void oe_apply_wind(inout vec4 vertex, in int index)
{
    // scale the vert's flexibility by the model Z scale factor
    float flexibility = length(flex) * vec3xform[2][2];

    if (flexibility > 0.0 && oe_wind_power > 0.0)
    {
        vec3 center = instances[index].xform[3].xyz;
        vec2 tile_uv = instances[index].local_uv;

        // sample the wind direction and speed:
        vec4 wind = textureProj(OE_WIND_TEX, (OE_WIND_TEX_MATRIX * vertex));
        vec3 wind_vec = normalize(wind.rgb * 2.0 - 1.0);
        float speed = wind.a * oe_wind_power;

        // sample the noise texture based on the speed, and use that
        // to permute the speed using the clumpy value:
        float time = osg_FrameTime * speed * 0.1;
        vec4 noise = textureLod(oe_veg_noise, tile_uv + time, 0);
        speed *= mix(0.75, 1.4, noise[0]);

        // integrate some ambient breeze for a swaying motion
        float ambient_seed = center.x + center.y + center.z + osg_FrameTime * 2.0;
        vec3 ambient = vec3(
            (2.0 * sin(1.0 * ambient_seed)) + 1.0,
            (1.0 * sin(2.0 * ambient_seed)) + 0.5,
            1.0);
        vec3 ambient_vec = wind_vec * ambient;

        // final wind force vector:
        vec3 wind_force =
            wind_vec * speed +
            ambient_vec * min(speed * 0.5, 0.05);

        // project the wind vector onto the flex plane
        vec3 flex_plane_normal = normalize(gl_NormalMatrix * vec3xform * flex);
        float dist = dot(wind_force, flex_plane_normal);
        vec3 wind_vec_projected = wind_force - flex_plane_normal * dist;

        // move the vertex within the flex plane
        vertex.xyz += wind_vec_projected * flexibility;
    }
}
#endif

void oe_vegetation_vs_view(inout vec4 vertex)
{
    int i = gl_BaseInstance + gl_InstanceID;
    oe_lod = instances[i].lod;

#ifdef OE_WIND_TEX
    oe_apply_wind(vertex, i);
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

#pragma import_defines(OE_TWEAKABLE)
#ifdef OE_TWEAKABLE
#define tweakable uniform
#else
#define tweakable const
#endif

tweakable float oe_veg_bbd0 = 0.5;
tweakable float oe_veg_bbd1 = 0.75;

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
        float f = abs(dot(fn, normalize(vp_VertexView)));
        color.a *= clamp(mix(0, 1, (f - oe_veg_bbd0) / (oe_veg_bbd1 - oe_veg_bbd0)), 0, 1);
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
