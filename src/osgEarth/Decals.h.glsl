
#pragma include FrustumGrid.h.glsl

#define MAX_DECALS_PER_TILE 16


struct DecalTile
{
    uint count;
    uint indices[MAX_DECALS_PER_TILE];
    uint padding[3];
};

struct Decal
{
    mat4 mvm;
    mat4 mvmInverse;
    float hx, hy, hz;  // half extents of bbox (no vec3 in ssbo please)
    int textureIndex;  // union of decal count (element 0) and textureIndex
    float opacity;
    float padding[3];
};


layout(binding = OE_BINDING_DECALS, std430) readonly buffer Decals
{
    Decal oe_decals[];
};

// each MAX_DECALS_PER_TILE+1 ints holds the count in the first Index followed
// by the decal indices in each subsequent index.
layout(binding = OE_BINDING_DECAL_TILES, std430) buffer DecalTiles
{
    DecalTile oe_decalTiles[];
};
