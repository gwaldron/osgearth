#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_overlay_fragment
#pragma vp_location   fragment_coloring
#pragma vp_order      0.6

#pragma import_defines(OE_IS_PICK_CAMERA)

uniform sampler2D oe_overlay_tex;
in vec4 oe_overlay_texcoord;

void oe_overlay_fragment(inout vec4 color)
{
    vec4 texel = textureProj(oe_overlay_tex, oe_overlay_texcoord);

#ifdef OE_IS_PICK_CAMERA
    color = texel;
#else
    color = vec4( mix( color.rgb, texel.rgb, texel.a ), color.a);
#endif
}
