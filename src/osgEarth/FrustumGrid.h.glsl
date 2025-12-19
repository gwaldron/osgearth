#pragma import_defines(OE_BINDING_FRUSTUM_PARAMS)
#ifndef OE_BINDING_FRUSTUM_PARAMS
#define     OE_BINDING_FRUSTUM_PARAMS 9
#endif

#pragma import_defines(OE_BINDING_FRUSTUMS)
#ifndef OE_BINDING_FRUSTUMS
#define     OE_BINDING_FRUSTUMS 10
#endif

#pragma import_defines(OE_TILES_PER_THREAD_GROUP)
#ifndef OE_TILES_PER_THREAD_GROUP
#define OE_TILES_PER_THREAD_GROUP 16
#endif

struct Frustum
{
    vec4 planes[4];
};

layout(std140, binding = OE_BINDING_FRUSTUM_PARAMS) uniform FrustumParams
{
    mat4 u_invProjMatrix;
    ivec4 u_viewport;
    ivec2 u_numTiles;
    uint u_pixelsPerTile;
    float u_debugTiles;
};

layout(binding = OE_BINDING_FRUSTUMS) buffer Frustums
{
    Frustum frustums[];
};
