#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name       REX Engine - Fragment
#pragma vp_entryPoint oe_rexEngine_frag
#pragma vp_location   fragment_coloring
#pragma vp_order      0.5

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

//in float oe_layer_rangeOpacity;

// Vertex Markers:
#define VERTEX_MARKER_DISCARD  1
#define VERTEX_MARKER_GRID     2
#define VERTEX_MARKER_PATCH    4
#define VERTEX_MARKER_BOUNDARY 8
#define VERTEX_MARKER_SKIRT    16
flat in int oe_terrain_vertexMarker;

void oe_rexEngine_frag(inout vec4 color)
{
    // if the provoking vertex is marked for discard, skip it:
    if ((oe_terrain_vertexMarker & VERTEX_MARKER_DISCARD) != 0)
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
    if ((oe_terrain_vertexMarker & VERTEX_MARKER_SKIRT) != 0)
    {
        discard;
        return;
    }
#endif // OE_IS_DEPTH_CAMERA

    // if this is a picking camera, reset the color to all zeros:
#ifdef OE_IS_PICK_CAMERA
    color = vec4(0);
#else

    // If imagery rendering is disabled, we're done:
#ifndef OE_TERRAIN_RENDER_IMAGERY
    return;
#endif

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

#endif // OE_IS_PICK_CAMERA
}
