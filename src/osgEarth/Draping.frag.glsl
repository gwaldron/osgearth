#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_overlay_fragment
#pragma vp_location   fragment_coloring
#pragma vp_order      0.6

uniform bool oe_isPickCamera;
uniform sampler2D oe_overlay_tex;
in vec4 oe_overlay_texcoord;

void oe_overlay_fragment(inout vec4 color)
{
    vec4 texel = texture2DProj(oe_overlay_tex, oe_overlay_texcoord);
    vec4 blendedTexel = vec4( mix( color.rgb, texel.rgb, texel.a ), color.a);

    float pick = oe_isPickCamera? 1.0 : 0.0;
    color = mix(blendedTexel, texel, pick);
}
