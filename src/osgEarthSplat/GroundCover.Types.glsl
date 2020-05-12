struct DrawElementsIndirectCommand
{
    uint count;
    uint instanceCount;
    uint firstIndex;
    uint baseVertex;
    uint baseInstance;
};
layout(binding=0, std430) buffer DrawCommandsBuffer
{
    DrawElementsIndirectCommand cmd[];
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
    int modelIndex;    // 4
    int sideIndex;     // 4
    int topIndex;      // 4
    float width;       // 4
    float height;      // 4
    float sinrot;      // 4
    float cosrot;      // 4
    float fillEdge;    // 4
    int modelId;       // 4
    uint tileNum;      // 4
    int drawId;        // 4
    uint instanceId;   // 4
    float sizeScale;   // 4
    float pixelSizeRatio;
};

layout(binding=1, std430) buffer InstanceBuffer
{
    InstanceHeaderData instanceHeader;
    InstanceData instance[];
};
layout(binding=4, std430) buffer RenderListBuffer
{
    InstanceHeaderData renderHeader;
    InstanceData render[];
};

struct TileData
{
    mat4 modelViewMatrix; // 4 x vec4 = 64 bytes
    mat4 normalMatrix;    // 4 x vec4 = 64 bytes // NO mat3 allowed!
};
layout(binding=2, std430) readonly buffer TileBuffer
{
    TileData tile[];
};