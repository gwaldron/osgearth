#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_contour_fragment
#pragma vp_location   fragment_coloring
#pragma vp_order      0.2

varying vec4 oe_layer_tilec;
uniform sampler1D oe_contour_xfer;
uniform float oe_contour_opacity;
uniform float oe_contour_min;
uniform float oe_contour_range;

float oe_terrain_getElevation(in vec2 uv);

void oe_contour_fragment( inout vec4 color )
{
    float height = oe_terrain_getElevation(oe_layer_tilec.st);
    float height_normalized = (height-oe_contour_min)/oe_contour_range;
    float lookup = clamp( height_normalized, 0.0, 1.0 );
    vec4 texel = texture1D( oe_contour_xfer, lookup );
    color.rgb = mix(color.rgb, texel.rgb, texel.a * oe_contour_opacity);
}