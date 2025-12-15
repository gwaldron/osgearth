#version 450
#extension GL_ARB_gpu_shader_int64 : enable
#pragma vp_function oe_applyDecals, fragment, last

#pragma import_defines(OE_DECALS_BUF_BINDING)
#pragma import_defines(OE_DECALS_TEX_BINDING)

in vec3 vp_VertexView;

struct GPUDecalInstance {
    mat4 projMatrixVS; // projector matrix, view space
    float hx, hy, hz;  // half extents of bbox (no vec3 in ssbo please)
    int textureIndex;  // union of decal count (element 0) and textureIndex
};

layout(binding=OE_DECALS_BUF_BINDING, std430) readonly buffer Decals {
    GPUDecalInstance decals[];
};
layout(binding=OE_DECALS_TEX_BINDING, std430) readonly buffer DecalTextures {
    uint64_t textures[];
};

void oe_applyDecals(inout vec4 color)
{
    // first element exists just to hold the decal count
    int count = decals[0].textureIndex;

    for (int i = 1; i < count + 1; ++i)
    {
        vec3 local = (decals[i].projMatrixVS * vec4(vp_VertexView, 1.0)).xyz;

        vec3 bbox = vec3(decals[i].hx, decals[i].hy, decals[i].hz);

        if (all(lessThanEqual(abs(local), bbox)))
        {
            vec2 uv = (local.xy / bbox.xy + vec2(1.0)) * vec2(0.5);
            int ti = decals[i].textureIndex;
            vec4 tex = ti >= 0 ? texture(sampler2D(textures[ti]), uv) : vec4(1, 0, 0, 1);
            color.rgb = mix(color.rgb, tex.rgb, 1 * tex.a);
        }
    }
}