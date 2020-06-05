#extension GL_ARB_gpu_shader_int64 : enable

struct DrawElementsIndirectCommand
{
    uint count;
    uint instanceCount;
    uint firstIndex;
    uint baseVertex;
    uint baseInstance;
};
struct DispatchIndirectCommand
{
    uint num_groups_x;
    uint num_groups_y;
    uint num_groups_z;
};
struct InstanceHeaderData
{
    uint count;
    uint _padding[3];
};
struct InstanceData
{
    vec4 vertex;       // 16
    vec2 tilec;        // 8

    int modelSamplerIndex;
    int sideSamplerIndex;
    int topSamplerIndex;

    float width;       // 4
    float height;      // 4
    float sinrot;      // 4
    float cosrot;      // 4

    float fillEdge;    // 4
    int modelId;       // 4
    uint tileNum;      // 4
    int drawId;        // 4

    int instanceId;    // 4 // -1 = unused
    float sizeScale;   // 4
    float pixelSizeRatio;
    float padding[1];
};
struct TileData
{
    mat4 modelViewMatrix; // 4 x vec4 = 64 bytes
    //mat4 normalMatrix;    // 4 x vec4 = 64 bytes // NO mat3 allowed!
};

layout(binding=0, std430) buffer DrawCommandsBuffer
{
    DrawElementsIndirectCommand cmd[];
};
layout(binding=1, std430) buffer GenBuffer
{
    InstanceData genInstance[];
};
layout(binding=2, std430) buffer InstanceBuffer
{
    DispatchIndirectCommand di;
    float _padding[1];
    InstanceData instance[];
};
layout(binding=3, std430) buffer TileDataBuffer
{
    TileData tileData[];
};
layout(binding=4, std430) buffer RenderBuffer
{
    InstanceData render[];
};
layout(binding=5, std430) buffer TextureLUT
{
    uint64_t texHandle[];
};
