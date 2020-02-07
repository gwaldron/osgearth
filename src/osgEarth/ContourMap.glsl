#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

    // NOTE: This vertex shader is no longer used.

#pragma vp_entryPoint oe_contour_vertex
#pragma vp_location   vertex_model
#pragma vp_order      0.5

out vec4 oe_layer_tilec;
out float oe_contour_lookup;
uniform float oe_contour_min;
uniform float oe_contour_range;

float oe_terrain_getElevation(in vec2 uv);

void oe_contour_vertex(inout vec4 VertexModel)
{
    float height = oe_terrain_getElevation(oe_layer_tilec.st);
    float height_normalized = (height-oe_contour_min)/oe_contour_range;
    oe_contour_lookup = clamp( height_normalized, 0.0, 1.0 );
}


[break]

#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_contour_fragment
#pragma vp_location   fragment_coloring
#pragma vp_order      0.5

in vec4 oe_layer_tilec;
uniform sampler1D oe_contour_xfer;
uniform float oe_contour_min;
uniform float oe_contour_range;

float oe_terrain_getElevation(in vec2 uv);

void oe_contour_fragment( inout vec4 color )
{
    float height = oe_terrain_getElevation(oe_layer_tilec.st);
    float height_normalized = (height-oe_contour_min)/oe_contour_range;
    float lookup = clamp( height_normalized, 0.0, 1.0 );
    vec4 texel = texture( oe_contour_xfer, lookup );
    color.rgb = mix(color.rgb, texel.rgb, texel.a);
}
