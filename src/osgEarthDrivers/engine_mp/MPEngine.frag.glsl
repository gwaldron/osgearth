#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#define MP_USE_TERRAIN_COLOR
#define MP_USE_BLENDING

uniform sampler2D oe_layer_tex;
uniform int oe_layer_uid;
uniform int oe_layer_order;
uniform float oe_layer_opacity;

varying vec4 oe_layer_texc;

#ifdef MP_USE_TERRAIN_COLOR
uniform vec4 oe_terrain_color;
#endif

void oe_mp_apply_coloring(inout vec4 color)
{
#ifdef MP_USE_TERRAIN_COLOR
    color = oe_terrain_color;
#endif

    float applyImagery = oe_layer_uid >= 0 ? 1.0 : 0.0;
    vec4 texel = mix(color, texture2D(oe_layer_tex, oe_layer_texc.st), applyImagery);
    texel.a = mix(texel.a, texel.a*oe_layer_opacity, applyImagery);

#ifdef MP_USE_BLENDING
    float firstLayer = oe_layer_order == 0 ? 1.0 : 0.0;
    color = mix(texel, texel*texel.a + color*(1.0-texel.a), firstLayer);
#else
    color = texel;
#endif
}
