#extension GL_ARB_gpu_shader_int64 : enable

#pragma vp_function oe_applyDecals, fragment, last

#pragma import_defines(OE_BINDING_DECAL_PARAMS)
#ifndef OE_BINDING_DECAL_PARAMS
#define     OE_BINDING_DECAL_PARAMS 9
#endif

#pragma import_defines(OE_BINDING_DECAL_TILES)
#ifndef OE_BINDING_DECAL_TILES
#define     OE_BINDING_DECAL_TILES 11
#endif

#pragma import_defines(OE_BINDING_DECALS)
#ifndef OE_BINDING_DECALS
#define     OE_BINDING_DECALS 12
#endif

#pragma import_defines(OE_BINDING_DECAL_TEXTURES)
#ifndef OE_BINDING_DECAL_TEXTURES
#define     OE_BINDING_DECAL_TEXTURES 13
#endif

#define MAX_DECALS_PER_TILE 16

in vec3 vp_VertexView;

struct DecalTile {
    uint count;
    uint indices[MAX_DECALS_PER_TILE];
    uint padding[3];
};

struct Decal {
    mat4 mvm;
    mat4 mvmInverse;
    float hx, hy, hz;  // half extents of bbox (no vec3 in ssbo please)
    int textureIndex;  // union of decal count (element 0) and textureIndex
};

layout(std140, binding = OE_BINDING_DECAL_PARAMS) uniform Params {
    mat4 u_invProjMatrix;
    ivec4 u_viewport;
    ivec2 u_numTiles;
    uint u_pixelsPerTile;
    float u_debugTiles;
};

layout(binding = OE_BINDING_DECALS, std430) readonly buffer Decals {
    Decal oe_decals[];
};

layout(binding = OE_BINDING_DECAL_TEXTURES, std430) readonly buffer DecalTextures {
    uint64_t oe_decalTextures[];
};

// each MAX_DECALS_PER_TILE+1 ints holds the count in the first Index followed
// by the decal indices in each subsequent index.
layout(binding = OE_BINDING_DECAL_TILES, std430) readonly buffer DecalTiles {
    DecalTile oe_decalTiles[];
};

uint computeTile()
{
    int height = (u_viewport[3] - u_viewport[1]);
    ivec2 tileID = ivec2(gl_FragCoord.x - u_viewport[0], height - gl_FragCoord.y - u_viewport[1]) / int(u_pixelsPerTile);
    return tileID.y * u_numTiles.x + tileID.x;
}


#define GPU_CULLING

#ifdef GPU_CULLING

void oe_applyDecals(inout vec4 color)
{
    DecalTile tile = oe_decalTiles[computeTile()];

    for (int i = 0; i < tile.count; ++i)
    {
        Decal decal = oe_decals[tile.indices[i]];
        vec3 local = (decal.mvmInverse * vec4(vp_VertexView, 1.0)).xyz;
        vec3 bbox = vec3(decal.hx, decal.hy, decal.hz);

        if (all(lessThanEqual(abs(local), bbox)))
        {
            vec2 uv = (local.xy / bbox.xy + vec2(1.0)) * vec2(0.5);
            int ti = decal.textureIndex;
            vec4 tex = ti >= 0 ? texture(sampler2D(oe_decalTextures[ti]), uv) : vec4(1, 0, 0, 1);
            color.rgb = mix(color.rgb, tex.rgb, tex.a * (1.0-u_debugTiles));
        }
    }

    // debugging overlay to show tile density
    float ramp = clamp(float(tile.count) / 5.0, 0.0, 1.0);
    vec3 debugColor = vec3(0, ramp, ramp);
    color.rgb = mix(color.rgb, debugColor, clamp(float(tile.count), 0, 1) * u_debugTiles * 0.5);
}

#else

void oe_applyDecals(inout vec4 color)
{
    // first element exists just to hold the decal count
    int count = oe_decals[0].textureIndex;

    for (int i = 1; i < count + 1; ++i)
    {
        vec3 local = (oe_decals[i].mvmInverse * vec4(vp_VertexView, 1.0)).xyz;
        vec3 bbox = vec3(oe_decals[i].hx, oe_decals[i].hy, oe_decals[i].hz);

        if (all(lessThanEqual(abs(local), bbox)))
        {
            vec2 uv = (local.xy / bbox.xy + vec2(1.0)) * vec2(0.5);
            int ti = oe_decals[i].textureIndex;
            vec4 tex = ti >= 0 ? texture(sampler2D(oe_decalTextures[ti]), uv) : vec4(1, 0, 0, 1);
            color.rgb = mix(color.rgb, tex.rgb, 1 * tex.a);
        }
    }
}

#endif
