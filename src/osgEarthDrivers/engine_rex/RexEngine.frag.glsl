#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#define REX_USE_TERRAIN_COLOR
#define REX_USE_BLENDING
#define REX_LOD_BLENDING

uniform sampler2D oe_layer_tex;
uniform int oe_layer_uid;
uniform int oe_layer_order;
uniform float oe_layer_opacity;
uniform sampler2D oe_layer_parentTex;

varying vec4 oe_layer_texc;
varying vec4 oe_layer_texcparent;
varying float oe_rexlod_r;

#ifdef REX_USE_TERRAIN_COLOR
uniform vec4 oe_terrain_color;
#endif

void oe_rexEngine_frag(inout vec4 color)
{
#ifdef REX_USE_TERRAIN_COLOR
    color = oe_terrain_color;
#endif

    float applyImagery = oe_layer_uid >= 0 ? 1.0 : 0.0;
    vec4 texel = mix(color, texture2D(oe_layer_tex, oe_layer_texc.st), applyImagery);
    texel.a = mix(texel.a, texel.a*oe_layer_opacity, applyImagery);
    
#ifdef REX_LOD_BLENDING
    vec4 parentTexel = texture2D(oe_layer_parentTex, oe_layer_texcparent.st);
    float enable = step(0.09, parentTexel.a);                  // did we get a parent texel?
    parentTexel.rgb = mix(texel.rgb, parentTexel.rgb, enable); // if not, use the incoming color for the blend
    parentTexel.a = mix(0.0, texel.a, enable);                 // ...and blend from alpha=0 for a fade-in effect.
    texel = mix(texel, parentTexel, oe_rexlod_r);
#endif

#ifdef REX_USE_BLENDING
    float firstLayer = oe_layer_order == 0 ? 1.0 : 0.0;
    color = mix(texel, texel*texel.a + color*(1.0-texel.a), firstLayer);
#else
    color = texel;
#endif
}
