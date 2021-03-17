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
struct InstanceData
{
    vec4 vertex;       // 16

    vec2 tilec;        // 8
    float width;       // 4
    float height;      // 4

    float sinrot;      // 4
    float cosrot;      // 4
    float fillEdge;    // 4
    float sizeScale;   // 4

    float pixelSizeRatio; // 4
    int modelCommand;  // 4
    int billboardCommand; // 4
    int tileNum;      // 4

    uint drawMask; // 4
    float _padding[3]; // 12
};
struct TileData
{
    mat4 modelViewMatrix; // 64 //4 x vec4 = 64 bytes
    int inUse;            // 4
    float _padding[3];    // 12
};
struct RenderLeaf
{
    uint instance;
    uint drawMask; //1=bb, 2=model
};

layout(binding=0, std430) buffer DrawCommandsBuffer
{
    DrawElementsIndirectCommand cmd[];
};
layout(binding=1, std430) buffer InstanceBuffer
{
    InstanceData instance[];
};
layout(binding=2, std430) buffer CullLUT
{
    DispatchIndirectCommand di; // 12
    float _padding;           // 4
    uint cullSet[];
};
layout(binding=3, std430) buffer TileDataBuffer
{
    TileData tileData[];
};
layout(binding=4, std430) buffer RenderLUT
{
    RenderLeaf renderSet[];
};
layout(binding=5, std430) buffer TextureLUT
{
    uint64_t texArena[];
};

// transition buffer (in pixel size ratio space) b/w billboards and models
#define PSR_BUFFER 0.22