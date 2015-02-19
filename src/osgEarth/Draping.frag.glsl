#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint "oe_overlay_fragment"
#pragma vp_location   "fragment_coloring"

uniform sampler2D oe_overlay_tex;
varying vec4      oe_overlay_texcoord;

void oe_overlay_fragment( inout vec4 color )
{
    vec4 texel = texture2DProj(oe_overlay_tex, oe_overlay_texcoord);
    color = vec4( mix( color.rgb, texel.rgb, texel.a ), color.a);
}