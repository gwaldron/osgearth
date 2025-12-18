#version 450

#pragma import_defines(OE_BINDING_DECAL_PARAMS)
#ifndef OE_BINDING_DECAL_PARAMS
#define     OE_BINDING_DECAL_PARAMS 9
#endif

#pragma import_defines(OE_BINDING_FRUSTUMS)
#ifndef OE_BINDING_FRUSTUMS
#define     OE_BINDING_FRUSTUMS 10
#endif

#pragma import_defines(OE_BINDING_DECAL_TILES)
#ifndef OE_BINDING_DECAL_TILES
#define     OE_BINDING_DECAL_TILES 11
#endif

#pragma import_defines(OE_BINDING_DECALS)
#ifndef OE_BINDING_DECALS
#define     OE_BINDING_DECALS 12
#endif

#define MAX_DECALS_PER_TILE 16
#define TILES_PER_THREAD_GROUP 16

layout(local_size_x = TILES_PER_THREAD_GROUP, local_size_y = TILES_PER_THREAD_GROUP) in;

struct Frustum {
    vec4 planes[4];
};

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

// computed in FrustumGridComputer.glsl
layout(binding = OE_BINDING_FRUSTUMS, std430) readonly buffer Frustums {
    Frustum frustums[];
};

layout(binding = OE_BINDING_DECALS, std430) readonly buffer Decals {
    Decal oe_decals[];
};

// each MAX_DECALS_PER_TILE+1 ints holds the count in the first Index followed
// by the decal indices in each subsequent index.
layout(binding = OE_BINDING_DECAL_TILES, std430) buffer DecalTiles {
    DecalTile oe_decalTiles[];
};

// test for intersection between decal and frustum
// by testing the decal's BS against all four frustum planes
bool intersects(in Decal decal, in Frustum frustum)
{
    float radius = length(vec3(decal.hx, decal.hy, decal.hz));
    vec3 posVS = (decal.mvm * vec4(0, 0, 0, 1)).xyz; // pos in view space
    for (int i = 0; i < 4; ++i)
    {
        if (dot(frustum.planes[i].xyz, posVS) - frustum.planes[i].w < -radius)
            return false;
    }
    return true;
}

void main()
{
    if (gl_GlobalInvocationID.x >= u_numTiles.x || gl_GlobalInvocationID.y >= u_numTiles.y)
        return;

    uint tile = gl_GlobalInvocationID.y * u_numTiles.x + gl_GlobalInvocationID.x;
    Frustum frustum = frustums[tile];

    // remember, first decal just holds the count.
    uint totalNumDecals = uint(oe_decals[0].textureIndex); // union
    uint numDecalsInTile = 0;

    for (uint i = 1; i < totalNumDecals+1 && numDecalsInTile < MAX_DECALS_PER_TILE; ++i)
    {
        // if decal intersects tile:
        if (intersects(oe_decals[i], frustum))
        {
            oe_decalTiles[tile].indices[numDecalsInTile++] = i;
        }
    }

    oe_decalTiles[tile].count = numDecalsInTile;
}
