#pragma vp_function oe_vegetation_vs_model, vertex_model

struct Instance
{
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

struct Instance
{
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

        // Add the flutter experienced by vegetation that snaps in the wind.
        // Sample the noise texture based on the speed, and use that
        // to permute the speed.
        float time = osg_FrameTime * speed * 0.1;
        vec4 noise = textureLod(oe_veg_noise, tile_uv + time, 0);
        speed *= mix(0.75, 1.4, noise[0]);

        // final wind force vector:
        vec3 wind_force = wind_vec * speed;

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
#pragma vp_function oe_vegetation_fs, fragment, 0.9
#pragma import_defines(OE_IS_SHADOW_CAMERA)

in vec2 oe_tex_uv;
flat in uint64_t oe_albedo_tex;
flat in uint oe_lod;
in vec3 vp_VertexView;

#define OE_A2C_ADJUSTMENT 0.275

uniform float oe_veg_bbd0 = 0.5;
uniform float oe_veg_bbd1 = 0.75;
uniform float oe_veg_alphaCutoff = 0.15;

void oe_vegetation_fs(inout vec4 color)
{
#ifndef OE_IS_SHADOW_CAMERA

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

#endif // !OE_IS_SHADOW_CAMERA
}
