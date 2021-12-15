#extension GL_ARB_gpu_shader_int64 : enable

//#if !defined(VP_STAGE_FRAGMENT)

#define OE_TILE_SIZE 17

#define MAX_TILE_VERTS 417

struct Global {
    vec2 uvs[MAX_TILE_VERTS];
    float padding[2];
    vec2 morphConstants[19]; // TODO - one per LOD
    float padding2[2];
};

struct Tile {
    vec4 verts[MAX_TILE_VERTS];
    vec4 normals[MAX_TILE_VERTS];
    vec4 tileKey;
    mat4 modelViewMatrix;
    mat4 colorMat;
    mat4 elevMat;
    mat4 normalMat;
    mat4 parentMat;
    mat4 landcoverMat; // not used
    mat4 sharedMat;
    int colorIndex;
    int elevIndex;
    int normalIndex;
    int parentIndex;
    int landcoverIndex; // not used
    int sharedIndex;
    int drawOrder;
    float padding[1];
};

#undef MAX_TILE_VERTS

layout(binding = 0, std430) readonly buffer TileBuffer {
    Tile tile[];
};
layout(binding = 1, std430) readonly buffer GlobalBuffer {
    Global global;
};
layout(binding = 5, std430) readonly buffer TextureArena {
    uint64_t tex[];
};

int oe_tileID; // vertex stage global


//#endif // !VP_STAGE_FRAGMENT
