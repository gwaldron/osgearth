#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

    // NOTE: This vertex shader is no longer used.

#pragma vp_entryPoint oe_contour_vertex
#pragma vp_location   vertex_model
#pragma vp_order      0.5

varying vec4 oe_layer_tilec;
uniform float oe_contour_min;
uniform float oe_contour_range;
varying float oe_contour_lookup;

float oe_terrain_getElevation(in vec2 uv);

void oe_contour_vertex(inout vec4 VertexModel)
{
    float height = oe_terrain_getElevation(oe_layer_tilec.st);
    float height_normalized = (height-oe_contour_min)/oe_contour_range;
    oe_contour_lookup = clamp( height_normalized, 0.0, 1.0 );
}