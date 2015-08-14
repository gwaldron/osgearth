#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint "oe_contour_fragment"
#pragma vp_location   "fragment_coloring"
#pragma vp_order      "0.2"

uniform sampler1D oe_contour_xfer;
uniform float oe_contour_opacity;
varying float oe_contour_lookup;

void oe_contour_fragment( inout vec4 color )
{
    vec4 texel = texture1D( oe_contour_xfer, oe_contour_lookup );
    color.rgb = mix(color.rgb, texel.rgb, texel.a * oe_contour_opacity);
}