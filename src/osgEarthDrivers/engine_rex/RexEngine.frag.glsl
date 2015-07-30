#version 330

#pragma vp_name       "REX Engine - Fragment"
#pragma vp_entryPoint "oe_rexEngine_frag"
#pragma vp_location   "fragment_coloring"
#pragma vp_order      "0.5"
#pragma vp_define     "OE_REX_USE_BLENDING"

uniform bool      oe_isPickCamera;
uniform sampler2D oe_layer_tex;
uniform sampler2D oe_layer_tex_parent;
uniform int       oe_layer_uid;
uniform int       oe_layer_order;
uniform float     oe_layer_opacity;
uniform vec4	  oe_tile_key;

in vec4 oe_layer_texc;
in vec4 oe_layer_texcParent;
in vec4 flerp;

void oe_rexEngine_frag(inout vec4 color)
{
    float applyImagery = oe_layer_uid >= 0 ? 1.0 : 0.0;

	vec4 colorSelf   = texture2D(oe_layer_tex		  , oe_layer_texc.st);
	vec4 colorParent = texture2D(oe_layer_tex_parent, oe_layer_texcParent.st);
	vec4 colorTex = mix(colorSelf, colorParent, flerp.y);

	vec4 texel = mix(color, colorTex, applyImagery);
    texel.a = mix(texel.a, texel.a*oe_layer_opacity, applyImagery);

    float firstLayer = oe_layer_order == 0 ? 1.0 : 0.0;

#ifdef OE_REX_USE_BLENDING
    color = mix(texel, texel*texel.a + color*(1.0-texel.a), firstLayer);
#else
    color = texel;
#endif

    // disable primary coloring for pick cameras. Necessary to support picking of
    // draped geometry.
    float pick = oe_isPickCamera ? 1.0 : 0.0;
    color = mix(color, vec4(0), pick);
}
