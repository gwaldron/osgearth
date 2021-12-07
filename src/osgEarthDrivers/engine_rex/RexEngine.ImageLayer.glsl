#version 430
#extension GL_ARB_gpu_shader_int64 : enable
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name REX Engine - ImageLayer/VS
#pragma vp_function oe_rex_imageLayer_VS, vertex_view, 0.4

#pragma import_defines(OE_INDIRECT)

// Stage globals
vec4 oe_layer_tilec;
vec2 oe_layer_texc;
vec2 oe_layer_texcParent;

#ifdef OE_INDIRECT
struct Tile {
    vec4 verts[289];
    vec4 normals[289];
    mat4 modelViewMatrix;
    mat4 colorMat;
    int colorIndex;
    float padding[3];
};
layout(binding = 0, std430) readonly restrict buffer TileBuffer {
    Tile tile[];
};
layout(binding = 5, std430) readonly restrict buffer TextureArena {
    uint64_t tex[];
};
//flat out int oe_layer_colorIndex;
flat out uint64_t oe_layer_colorHandle;
#else
uniform mat4 oe_layer_texMatrix;
uniform mat4 oe_layer_texParentMatrix;
#endif

void oe_rex_imageLayer_VS(inout vec4 vertexView)
{
#ifdef OE_INDIRECT
    int i = gl_InstanceID; // gl_DrawID;
    oe_layer_texc = (tile[i].colorMat * oe_layer_tilec).st;
    //oe_layer_texcParent = (oe_layer_texParentMatrix * oe_layer_tilec).st;
    //oe_layer_colorIndex = tile[i].colorIndex;
    oe_layer_colorHandle = tex[tile[i].colorIndex];
#else
    oe_layer_texc = (oe_layer_texMatrix * oe_layer_tilec).st;
    oe_layer_texcParent = (oe_layer_texParentMatrix * oe_layer_tilec).st;
#endif
}


[break]


#version 430
#extension GL_ARB_gpu_shader_int64 : enable
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name REX Engine - Fragment
#pragma vp_function oe_rex_imageLayer_FS, fragment_coloring, 0.5

#pragma import_defines(OE_INDIRECT)
#pragma import_defines(OE_TERRAIN_RENDER_IMAGERY)
#pragma import_defines(OE_TERRAIN_MORPH_IMAGERY)
#pragma import_defines(OE_TERRAIN_BLEND_IMAGERY)
#pragma import_defines(OE_TERRAIN_CAST_SHADOWS)
#pragma import_defines(OE_IS_PICK_CAMERA)
#pragma import_defines(OE_IS_SHADOW_CAMERA)
#pragma import_defines(OE_IS_DEPTH_CAMERA)

uniform sampler2D oe_layer_tex;
uniform int       oe_layer_uid;
uniform int       oe_layer_order;

#ifdef OE_TERRAIN_MORPH_IMAGERY
uniform sampler2D oe_layer_texParent;
uniform float oe_layer_texParentExists;
in vec2 oe_layer_texcParent;
in float oe_rex_morphFactor;
#endif

in vec2 oe_layer_texc;
in vec4 oe_layer_tilec;
in float oe_layer_opacity;

// Vertex Markers:
#define VERTEX_VISIBLE  1
#define VERTEX_BOUNDARY 2
#define VERTEX_HAS_ELEVATION 4
#define VERTEX_SKIRT 8
flat in int oe_terrain_vertexMarker;

#ifdef OE_INDIRECT
flat in uint64_t oe_layer_colorHandle;
#endif

void oe_rex_imageLayer_FS(inout vec4 color)
{
    // if the provoking vertex is marked for discard, skip it:
    if ((oe_terrain_vertexMarker & VERTEX_VISIBLE) == 0)
    {
        discard;
        return;
    }

    // If this is a shadow camera and the terrain doesn't cast shadows, no render:
#if defined(OE_IS_SHADOW_CAMERA) && !defined(OE_TERRAIN_CAST_SHADOWS)
    discard;
    return;
#endif

    // If this is a depth-only camera, skip terrain skirt geometry:
#if defined(OE_IS_DEPTH_CAMERA)
    if ((oe_terrain_vertexMarker & VERTEX_SKIRT) != 0)
    {
        discard;
        return;
    }
#endif // OE_IS_DEPTH_CAMERA

    // if this is a picking camera, reset the color to all zeros:
#ifdef OE_IS_PICK_CAMERA
    color = vec4(0);
    return;
#endif

    // If imagery rendering is disabled, we're done:
#ifndef OE_TERRAIN_RENDER_IMAGERY
    return;
#endif


#ifdef OE_INDIRECT

    // whether this layer contains texel color (UID<0 means no texture)
    bool isTexelLayer = oe_layer_colorHandle > 0UL;

    // whether this is the first layer to render:
    bool isFirstLayer = oe_layer_order == 0;

    vec4 texel = color;

    if (isTexelLayer)
    {
        texel = texture(sampler2D(oe_layer_colorHandle), oe_layer_texc);
        //texel = texture(sampler2D(tex[oe_layer_colorIndex]), oe_layer_texc);

#ifdef OE_TERRAIN_MORPH_IMAGERY
        // sample the main texture:

        // sample the parent texture:
        vec4 texelParent = texture(oe_layer_texParent, oe_layer_texcParent);

        // if the parent texture does not exist, use the current texture with alpha=0 as the parent
        // so we can "fade in" an image layer that starts at LOD > 0:
        texelParent = mix(vec4(texel.rgb, 0.0), texelParent, oe_layer_texParentExists);

        // Resolve the final texel color:
        texel = mix(texel, texelParent, oe_rex_morphFactor);
#endif

        // intergrate thelayer opacity:
        texel.a = texel.a * oe_layer_opacity;
        color.a = 1.0;
    }

#ifdef OE_TERRAIN_BLEND_IMAGERY
    // If this is a first image layer, blend with the incoming terrain color.
    // Otherwise, apply directly and let GL blending do the rest.
    if (isTexelLayer && isFirstLayer)
    {
        color.rgb = texel.rgb*texel.a + color.rgb*(1.0 - texel.a);
    }
    else
    {
        color = texel;
    }
#else
    // No blending? The output is just the texel value.
    color = texel;
#endif // OE_TERRAIN_BLEND_IMAGERY


#else // OE_INDIRECT

    // whether this layer contains texel color (UID<0 means no texture)
    bool isTexelLayer = oe_layer_uid >= 0;

    // whether this is the first layer to render:
    bool isFirstLayer = oe_layer_order == 0;

    vec4 texel = color;

    if (isTexelLayer)
    {
        texel = texture(oe_layer_tex, oe_layer_texc);

#ifdef OE_TERRAIN_MORPH_IMAGERY
        // sample the main texture:

        // sample the parent texture:
        vec4 texelParent = texture(oe_layer_texParent, oe_layer_texcParent);

        // if the parent texture does not exist, use the current texture with alpha=0 as the parent
        // so we can "fade in" an image layer that starts at LOD > 0:
        texelParent = mix( vec4(texel.rgb, 0.0), texelParent, oe_layer_texParentExists );

        // Resolve the final texel color:
        texel = mix(texel, texelParent, oe_rex_morphFactor);
#endif

        // intergrate thelayer opacity:
        texel.a = texel.a * oe_layer_opacity;
        color.a = 1.0;
    }

#ifdef OE_TERRAIN_BLEND_IMAGERY
    // If this is a first image layer, blend with the incoming terrain color.
    // Otherwise, apply directly and let GL blending do the rest.
    if (isTexelLayer && isFirstLayer)
    {
        color.rgb = texel.rgb*texel.a + color.rgb*(1.0-texel.a);
    }
    else
    {
        color = texel;
    }
#else
    // No blending? The output is just the texel value.
    color = texel;
#endif // OE_TERRAIN_BLEND_IMAGERY

#endif // OE_INDIRECT
}
