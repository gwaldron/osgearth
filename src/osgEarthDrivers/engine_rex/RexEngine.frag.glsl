#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name       REX Engine - Fragment
#pragma vp_entryPoint oe_rexEngine_frag
#pragma vp_location   fragment_coloring
#pragma vp_order      0.5

#pragma import_defines(OE_TERRAIN_RENDER_IMAGERY, OE_TERRAIN_MORPH_IMAGERY, OE_TERRAIN_BLEND_IMAGERY, OE_TERRAIN_CAST_SHADOWS, OE_IS_PICK_CAMERA, OE_IS_SHADOW_CAMERA)

uniform sampler2D oe_layer_tex;
uniform int       oe_layer_uid;
uniform int       oe_layer_order;
uniform float     oe_layer_opacity;

#ifdef OE_TERRAIN_MORPH_IMAGERY
uniform sampler2D oe_layer_texParent;
uniform float oe_layer_texParentExists;
in vec4 oe_layer_texcParent;
in float oe_rex_morphFactor;
#endif

in vec4 oe_layer_texc;
in vec4 oe_layer_tilec;

in float oe_layer_rangeOpacity;

void oe_rexEngine_frag(inout vec4 color)
{
#if defined(OE_IS_SHADOW_CAMERA) && !defined(OE_TERRAIN_CAST_SHADOWS)
    discard;
    return;
#endif

#ifdef OE_IS_PICK_CAMERA
    color = vec4(0);
#else

#ifndef OE_TERRAIN_RENDER_IMAGERY
    return;
#endif

    float isImageLayer = oe_layer_uid >= 0 ? 1.0 : 0.0;
	vec4 texelSelf = texture(oe_layer_tex, oe_layer_texc.st);

#ifdef OE_TERRAIN_MORPH_IMAGERY

    // sample the parent texture:
	vec4 texelParent = texture(oe_layer_texParent, oe_layer_texcParent.st);

    // if the parent texture does not exist, use the current texture with alpha=0 as the parent
    // so we can "fade in" an image layer that starts at LOD > 0:
    texelParent = mix( vec4(texelSelf.rgb, 0.0), texelParent, oe_layer_texParentExists );

    // Resolve the final texel color:
	vec4 texel = mix(texelSelf, texelParent, oe_rex_morphFactor);

    // Decide whether to use the texel or the incoming color:
	texel = mix(color, texel, isImageLayer);

#else

    // No morphing, just use the incoming color or texture:
    vec4 texel = mix(color, texelSelf, isImageLayer);

#endif

    // Integrate layer opacity into the texture:
    //texel.a = mix(texel.a, texel.a*oe_layer_rangeOpacity, isImageLayer);
    texel.a = mix(texel.a, texel.a*oe_layer_opacity*oe_layer_rangeOpacity, isImageLayer);
    
#ifdef OE_TERRAIN_BLEND_IMAGERY

    float isFirstImageLayer = (isImageLayer == 1.0 && oe_layer_order == 0) ? 1.0 : 0.0;

    // If this is a first image layer, blend with the incoming terrian color.
    // Otherwise, apply directly and let GL blending do the rest.
    if (isFirstImageLayer == 1.0) {
        color.rgb = texel.rgb*texel.a + color.rgb*(1.0-texel.a);
        color.a = max(color.a, texel.a);
    }
    else color = texel;

#else

    // No blending? The output is just the texel value.
    color = texel;

#endif // OE_TERRAIN_BLEND_IMAGERY

#endif // OE_IS_PICK_CAMERA
}
