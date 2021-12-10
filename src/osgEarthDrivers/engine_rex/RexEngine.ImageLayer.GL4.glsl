#version 460
#pragma include RexEngine.GL4.glsl
#pragma vp_name REX Engine - ImageLayer/VS
#pragma vp_function oe_rex_imageLayer_VS, vertex_view, 0.4

// Stage globals
vec4 oe_layer_tilec;

// outputs
out vec2 oe_color_uv;
out vec2 oe_parent_uv;
flat out uint64_t oe_color_handle;
flat out uint64_t oe_parent_handle;
flat out int oe_draw_order;

void oe_rex_imageLayer_VS(inout vec4 vertexView)
{
    oe_color_uv = (tile[oe_tileID].colorMat * oe_layer_tilec).st;
    oe_color_handle = tex[tile[oe_tileID].colorIndex];

    oe_parent_uv = (tile[oe_tileID].parentMat * oe_layer_tilec).st;
    oe_parent_handle = tex[tile[oe_tileID].parentIndex];

    oe_draw_order = tile[oe_tileID].drawOrder;
}


[break]


#version 460
#pragma include RexEngine.GL4.glsl

#pragma vp_name REX Engine - Fragment
#pragma vp_function oe_rex_imageLayer_FS, fragment_coloring, 0.5

#pragma import_defines(OE_TERRAIN_RENDER_IMAGERY)
#pragma import_defines(OE_TERRAIN_MORPH_IMAGERY)
#pragma import_defines(OE_TERRAIN_BLEND_IMAGERY)
#pragma import_defines(OE_TERRAIN_CAST_SHADOWS)
#pragma import_defines(OE_IS_PICK_CAMERA)
#pragma import_defines(OE_IS_SHADOW_CAMERA)
#pragma import_defines(OE_IS_DEPTH_CAMERA)

//uniform sampler2D oe_layer_tex;
uniform int oe_layer_uid;
uniform int oe_layer_order;

#ifdef OE_TERRAIN_MORPH_IMAGERY
in vec2 oe_parent_uv;
flat in uint64_t oe_parent_handle;
in float oe_rex_morphFactor;
#endif

// inputs
in vec2 oe_color_uv;
flat in uint64_t oe_color_handle;
in vec4 oe_layer_tilec;
in float oe_layer_opacity;
flat in int oe_terrain_vertexMarker;
flat in int oe_draw_order;

#define VERTEX_VISIBLE  1
#define VERTEX_BOUNDARY 2
#define VERTEX_HAS_ELEVATION 4
#define VERTEX_SKIRT 8

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

    // whether this layer contains texel color (UID<0 means no texture)
    bool isTexelLayer = oe_color_handle > 0UL;

    vec4 texel = color;

    if (isTexelLayer)
    {
        texel = texture(sampler2D(oe_color_handle), oe_color_uv);

#ifdef OE_TERRAIN_MORPH_IMAGERY

        if (oe_parent_handle != 0UL)
        {
            // sample the parent texture and blend for the morphing:
            vec4 texelParent = texture(sampler2D(oe_parent_handle), oe_parent_uv);
            texel = mix(texel, texelParent, oe_rex_morphFactor);
    }
#endif

        // intergrate thelayer opacity:
        texel.a = texel.a * oe_layer_opacity;
        color.a = 1.0;
    }

#ifdef OE_TERRAIN_BLEND_IMAGERY
    // If this is a first image layer, blend with the incoming terrain color.
    // Otherwise, apply directly and let GL blending do the rest.
    if (isTexelLayer && (oe_draw_order == 0))
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
}
