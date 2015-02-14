#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint "oe_mp_apply_coloring"
#pragma vp_location   "fragment_coloring"
#pragma vp_order      "0"

#define MP_USE_BLENDING

uniform vec4 oe_terrain_color;
uniform sampler2D oe_layer_tex;
uniform int oe_layer_uid;
uniform int oe_layer_order;
uniform float oe_layer_opacity;

varying vec4 oe_layer_texc;
varying float oe_terrain_rangeOpacity;

uniform float m;

void oe_mp_apply_coloring(inout vec4 color)
{
    color = oe_terrain_color.a >= 0.0 ? oe_terrain_color : color;

    float applyImagery = oe_layer_uid >= 0 ? 1.0 : 0.0;
    vec4 texel = mix(color, texture2D(oe_layer_tex, oe_layer_texc.st), applyImagery);
    texel.a = mix(texel.a, texel.a*oe_layer_opacity*oe_terrain_rangeOpacity, applyImagery);

#ifdef MP_USE_BLENDING
    float firstLayer = oe_layer_order == 0 ? 1.0 : 0.0;
    color = mix(texel, texel*texel.a + color*(1.0-texel.a), firstLayer);
#else
    color = texel;
#endif
}
