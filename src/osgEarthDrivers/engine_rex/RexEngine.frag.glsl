#version 330

#pragma vp_name       REX Engine - Fragment
#pragma vp_entryPoint oe_rexEngine_frag
#pragma vp_location   fragment_coloring
#pragma vp_order      0.5
#pragma vp_define     OE_REX_GL_BLENDING
#pragma vp_define     OE_REX_MORPH_IMAGERY

uniform bool      oe_isPickCamera;
uniform sampler2D oe_layer_tex;
uniform int       oe_layer_uid;
uniform int       oe_layer_order;
uniform float     oe_layer_opacity;

#ifdef OE_REX_MORPH_IMAGERY
uniform sampler2D oe_layer_texParent;
uniform float oe_layer_texParentExists;
in vec4 oe_layer_texcParent;
in float oe_rex_morphFactor;
#endif

in vec4 oe_layer_texc;

in float oe_layer_rangeOpacity;

void oe_rexEngine_frag(inout vec4 color)
{
    float applyImagery = oe_layer_uid >= 0 ? 1.0 : 0.0;
    
	vec4 texelSelf = texture(oe_layer_tex, oe_layer_texc.st);

#ifdef OE_REX_MORPH_IMAGERY

    // sample the parent texture:
	vec4 texelParent = texture(oe_layer_texParent, oe_layer_texcParent.st);

    // if the parent texture does not exist, use the current texture with alpha=0 as the parent
    // so we can "fade in" an image layer that starts at LOD > 0:
    texelParent = mix( vec4(texelSelf.rgb, 0.0), texelParent, oe_layer_texParentExists );

    // Resolve the final texel color:
	vec4 texel = mix(texelSelf, texelParent, oe_rex_morphFactor);

    // Decide whether to use the texel or the incoming color:
	texel = mix(color, texel, applyImagery);

#else

    // No morphing, just use the incoming color or texture:
    vec4 texel = mix(color, texelSelf, applyImagery);

#endif

    // Integrate layer opacity into the texture:
    texel.a = mix(texel.a, texel.a*oe_layer_opacity*oe_layer_rangeOpacity, applyImagery);

    float firstLayer = (applyImagery == 1.0 && oe_layer_order == 0) ? 1.0 : 0.0;

#ifdef OE_REX_GL_BLENDING
    
    // Blend RGB with the incoming color:
    //color.rgb = texel.rgb*texel.a + color.rgb*(1.0-texel.a);

    // If this is a first image layer, use the max alpha; otherwise just leave it
    // to GL blending
    if (firstLayer == 1.0) {
        color.rgb = texel.rgb*texel.a + color.rgb*(1.0-texel.a);
        color.a = max(color.a, texel.a);
    }
    else color = texel;

#else

    // No blending? The output is just the texel value.
    color = texel;

#endif

    // disable primary coloring for pick cameras. Necessary to support picking of
    // draped geometry.
    float pick = oe_isPickCamera ? 1.0 : 0.0;
    color = mix(color, vec4(0), pick);
}
