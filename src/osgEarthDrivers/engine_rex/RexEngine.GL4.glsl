#extension GL_ARB_gpu_shader_int64 : enable

//#if !defined(VP_STAGE_FRAGMENT)

#define OE_TILE_SIZE 17

#define MAX_TILE_VERTS 417

struct oe_rex_Global {
    vec2 uvs[MAX_TILE_VERTS];
    float padding[2];
    vec2 morphConstants[19]; // TODO - one per LOD
    float padding2[2];
};

struct oe_rex_Tile {
    vec4 verts[MAX_TILE_VERTS];
    vec4 normals[MAX_TILE_VERTS];
    vec4 tileKey;
    mat4 modelViewMatrix;
    mat4 colorMat;
    mat4 parentMat;
    mat4 elevMat;
    mat4 normalMat;
    mat4 sharedMat[4];
    int colorIndex;
    int parentIndex;
    int elevIndex;
    int normalIndex;
    int sharedIndex[4];
    int drawOrder;
    float padding[3];
};

#undef MAX_TILE_VERTS

layout(binding = 0, std430) readonly buffer TileBuffer {
    oe_rex_Tile oe_tile[];
};
layout(binding = 1, std430) readonly buffer GlobalBuffer {
    oe_rex_Global oe_global;
};
layout(binding = 5, std430) readonly buffer TextureArena {
    uint64_t oe_terrain_tex[];
};

int oe_tileID; // vertex stage global

//#endif // !VP_STAGE_FRAGMENT
