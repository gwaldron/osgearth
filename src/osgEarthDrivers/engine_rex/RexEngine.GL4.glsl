#extension GL_ARB_gpu_shader_int64 : enable

#define OE_TILE_SIZE 17
#define OE_TILE_VERTS 417
#define OE_SKIRT_VERTS 128

struct oe_rex_Global {
    vec2 uvs[OE_TILE_VERTS];
    float padding[2];
    vec2 morphConstants[19]; // TODO - one per LOD
    float padding2[2];
};

struct oe_rex_Tile {
    vec4 verts[OE_TILE_VERTS];
    vec4 normals[OE_TILE_VERTS];
    vec4 tileKey;
    mat4 modelViewMatrix;
    mat4 colorMat;
    mat4 parentMat;
    mat4 elevMat;
    mat4 normalMat;
    mat4 sharedMat[8];
    int colorIndex;
    int parentIndex;
    int elevIndex;
    int normalIndex;
    int sharedIndex[8];
    int drawOrder;
    float padding[3];
};

layout(binding = 0, std430) readonly buffer TileBuffer {
    oe_rex_Tile oe_tile[];
};
layout(binding = 1, std430) readonly buffer GlobalBuffer {
    oe_rex_Global oe_global;
};
layout(binding = 5, std430) readonly buffer TextureArena {
    uint64_t oe_terrain_tex[];
};

#ifdef VP_STAGE_FRAGMENT
flat in int oe_tileID;
#else
flat out int oe_tileID;
#endif
