#version 460
#extension GL_NV_gpu_shader5 : enable

#pragma import_defines(OE_GPUCULL_DEBUG)
#pragma import_defines(OE_IS_SHADOW_CAMERA)

#pragma import_defines(OE_TWEAKABLE)
#ifdef OE_TWEAKABLE
#define tweakable uniform
#else
#define tweakable const
#endif

layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;

struct DrawElementsIndirectCommand
{
    uint count;
    uint instanceCount;
    uint firstIndex;
    uint baseVertex;
    uint baseInstance;
};

struct BindlessPtrNV
{
    uint index;
    uint reserved;
    uint64_t address;
    uint64_t length;
};

struct DrawElementsIndirectBindlessCommandNV
{
    DrawElementsIndirectCommand cmd;
    uint reserved;
    BindlessPtrNV indexBuffer;
    BindlessPtrNV vertexBuffer;
};

struct ChonkLOD
{
    vec4 bs;
    float far_pixel_scale;
    float near_pixel_scale;
    uint num_lods; // chonk-global
    uint total_num_commands; // global
};

struct Instance
{
    mat4 xform;
    vec2 local_uv;
    uint lod;
    float visibility[4];
    uint first_lod_cmd_index;
};

layout(binding = 0) buffer OutputBuffer
{
    Instance output_instances[];
};

layout(binding = 29) buffer Commands
{
    DrawElementsIndirectBindlessCommandNV commands[];
};

layout(binding = 30) buffer ChonkLODs
{
    ChonkLOD chonks[];
};

layout(binding = 31) buffer InputBuffer
{
    Instance input_instances[];
};

uniform vec3 oe_Camera;
uniform float oe_sse;
uniform vec4 oe_lod_scale;

#if OE_GPUCULL_DEBUG
//#ifdef OE_GPUCULL_DEBUG
#define REJECT(X) if (fade==1.0) { fade=(X);}
#else
#define REJECT(X) return
#endif
#define REASON_FRUSTUM 1.5
#define REASON_SSE 2.5
#define REASON_NEARCLIP 3.5

tweakable float oe_chonk_lod_transition_factor = 0.1;

void cull()
{
    const uint i = gl_GlobalInvocationID.x; // instance
    const uint lod = gl_GlobalInvocationID.y; // lod

    // initialize by clearing the visibility for this LOD:
    input_instances[i].visibility[lod] = 0.0;

    // bail if our chonk does not have this LOD
    uint v = input_instances[i].first_lod_cmd_index + lod;
    if (lod >= chonks[v].num_lods)
        return;

#ifdef OE_IS_SHADOW_CAMERA
    // only the lowest LOD for shadow-casting.
    if (lod < chonks[v].num_lods - 1)
        return;
#endif

    // intialize:
    float fade = 1.0;

    // transform the bounding sphere to a view-space bbox.
    mat4 xform = input_instances[i].xform;
    vec4 center = xform * vec4(chonks[v].bs.xyz, 1);
    vec4 center_view = gl_ModelViewMatrix * center;

    float max_scale = max(xform[0][0], max(xform[1][1], xform[2][2]));
    float r = chonks[v].bs.w * max_scale;

    // Trivially reject low-LOD instances that intersect the near clip plane:
    if ((lod > 0) && (gl_ProjectionMatrix[3][3] < 0.01)) // is perspective camera
    {
        float near = gl_ProjectionMatrix[2][3] / (gl_ProjectionMatrix[2][2] - 1.0);
        if (-(center_view.z + r) <= near)
        {
            REJECT(REASON_NEARCLIP);
        }
    }

    // find the clip-space MBR and intersect with the clip frustum:
    vec4 LL, UR, temp;
    temp = gl_ProjectionMatrix * (center_view + vec4(-r, -r, -r, 0)); temp /= temp.w;
    LL = temp; UR = temp;
    temp = gl_ProjectionMatrix * (center_view + vec4(-r, -r, +r, 0)); temp /= temp.w;
    LL = min(LL, temp); UR = max(UR, temp);
    temp = gl_ProjectionMatrix * (center_view + vec4(-r, +r, -r, 0)); temp /= temp.w;
    LL = min(LL, temp); UR = max(UR, temp);
    temp = gl_ProjectionMatrix * (center_view + vec4(-r, +r, +r, 0)); temp /= temp.w;
    LL = min(LL, temp); UR = max(UR, temp);
    temp = gl_ProjectionMatrix * (center_view + vec4(+r, -r, -r, 0)); temp /= temp.w;
    LL = min(LL, temp); UR = max(UR, temp);
    temp = gl_ProjectionMatrix * (center_view + vec4(+r, -r, +r, 0)); temp /= temp.w;
    LL = min(LL, temp); UR = max(UR, temp);
    temp = gl_ProjectionMatrix * (center_view + vec4(+r, +r, -r, 0)); temp /= temp.w;
    LL = min(LL, temp); UR = max(UR, temp);
    temp = gl_ProjectionMatrix * (center_view + vec4(+r, +r, +r, 0)); temp /= temp.w;
    LL = min(LL, temp); UR = max(UR, temp);

#if OE_GPUCULL_DEBUG
    float threshold = 0.75;
#else
    float threshold = 1.0;
#endif

    if (LL.x > threshold || LL.y > threshold)
        REJECT(REASON_FRUSTUM);

    if (UR.x < -threshold || UR.y < -threshold)
        REJECT(REASON_FRUSTUM);

#ifndef OE_IS_SHADOW_CAMERA

    // OK, it is in view - now check pixel size on screen for this LOD:
    vec2 dims = 0.5*(UR.xy - LL.xy)*oe_Camera.xy;

    float pixelSize = max(dims.x, dims.y);
    float pixelSizePad = pixelSize * oe_chonk_lod_transition_factor; // 0.1;

    float minPixelSize = oe_sse * chonks[v].far_pixel_scale * oe_lod_scale[lod];
    if (pixelSize < (minPixelSize - pixelSizePad))
        REJECT(REASON_SSE);

    float near_scale = lod > 0 ? chonks[v].near_pixel_scale * oe_lod_scale[lod - 1] : 99999.0;
    float maxPixelSize = oe_sse * near_scale;
    if (pixelSize > (maxPixelSize + pixelSizePad))
        REJECT(REASON_SSE);

    if (fade == 1.0)  // good to go, set the proper fade:
    {
        if (pixelSize > maxPixelSize)
            fade = 1.0 - (pixelSize - maxPixelSize) / pixelSizePad;
        else if (pixelSize < minPixelSize)
            fade = 1.0 - (minPixelSize - pixelSize) / pixelSizePad;
    }
#endif

    if (fade < 0.1)
        return;

    // Pass! Set the visibility for this LOD:
    input_instances[i].visibility[lod] = fade;

    // Bump all baseInstances following this one:
    const uint cmd_count = chonks[v].total_num_commands;
    for (uint i = v + 1; i < cmd_count; ++i)
    {
        atomicAdd(commands[i].cmd.baseInstance, 1);
    }
}

// Copies the visible instances to a compacted output buffer.
void compact()
{
    const uint i = gl_GlobalInvocationID.x; // instance
    const uint lod = gl_GlobalInvocationID.y; // lod

    float fade = input_instances[i].visibility[lod];
    if (fade < 0.1)
        return;

    uint v = input_instances[i].first_lod_cmd_index + lod;
    uint offset = commands[v].cmd.baseInstance;
    uint index = atomicAdd(commands[v].cmd.instanceCount, 1);

    // Lazy! Re-using the instance struct for render leaves..
    output_instances[offset + index] = input_instances[i];
    output_instances[offset + index].lod = lod;
}

// Entry point.
uniform int oe_pass;

void main()
{
    if (oe_pass == 0)
        cull();
    else
        compact();
}
