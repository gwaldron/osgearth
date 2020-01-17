#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_mp_apply_coloring
#pragma vp_location   fragment_coloring
#pragma vp_order      0.5
#pragma vp_define     MP_USE_BLENDING

#pragma import_defines(OE_IS_PICK_CAMERA)
#pragma import_defines(OE_IS_DEPTH_CAMERA)
#pragma import_defines(OE_IS_SHADOW_CAMERA)
#pragma import_defines(OE_TERRAIN_CAST_SHADOWS)

uniform vec4 oe_terrain_color;
uniform sampler2D oe_layer_tex;
uniform int oe_layer_uid;
uniform int oe_layer_order;
uniform float oe_layer_opacity;

in vec4 oe_layer_texc;
in float oe_layer_rangeOpacity;

void oe_mp_apply_coloring(inout vec4 color)
{
#if defined(OE_IS_DEPTH_CAMERA)
    #if defined(OE_IS_SHADOW_CAMERA) && !defined(OE_TERRAIN_CAST_SHADOWS)
        discard;
    #endif
    return;
#endif

#ifdef OE_IS_PICK_CAMERA
    color = vec4(0);
#else

    color = oe_terrain_color.a >= 0.0 ? oe_terrain_color : color;

    float applyImagery = oe_layer_uid >= 0 ? 1.0 : 0.0;
    vec4 texel = mix(color, texture(oe_layer_tex, oe_layer_texc.st), applyImagery);
    texel.a = mix(texel.a, texel.a*oe_layer_opacity*oe_layer_rangeOpacity, applyImagery);

#ifdef MP_USE_BLENDING
    float firstLayer = oe_layer_order == 0 ? 1.0 : 0.0;
    color = mix(texel, texel*texel.a + color*(1.0-texel.a), firstLayer);    
#else
    color = texel;
#endif

#endif // OE_IS_PICK_CAMERA
}
