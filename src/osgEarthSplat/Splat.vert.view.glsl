#version 330

#pragma vp_entryPoint "oe_splat_vertex_view"
#pragma vp_location   "vertex_view"
#pragma vp_order      "0.5"

#pragma include "Splat.types.glsl"

out vec4 oe_layer_tilec;
out float oe_splat_range;
out vec2 oe_splat_covtc;

uniform mat4 $COVERAGE_TEXMAT_UNIFORM;   // assigned at runtime


void oe_splat_vertex_view(inout vec4 VertexVIEW)
{
    // range from camera to vertex
    oe_splat_range = -VertexVIEW.z;

    // calculate the coverage sampling coordinates. The texture matrix accounts
    // for any super-sampling that might be in effect for the current LOD.
    oe_splat_covtc = ($COVERAGE_TEXMAT_UNIFORM * oe_layer_tilec).st;
}
