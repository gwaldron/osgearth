#version 120

#pragma vp_entryPoint "oe_splat_vertex_view"
#pragma vp_location   "vertex_view"
#pragma vp_order      "0.5"

// from the terrain engine
in vec4 oe_layer_tilec;

out float oe_splat_range;
out vec2 oe_splat_covtc;
out float oe_splat_scaleOffsetInt;

uniform mat4 $COVERAGE_TEXMAT_UNIFORM;   // assigned at runtime
uniform float oe_splat_scaleOffset;

void oe_splat_vertex_view(inout vec4 VertexVIEW)
{
    // range from camera to vertex
    oe_splat_range = -VertexVIEW.z/VertexVIEW.w;

    // calculate the coverage sampling coordinates. The texture matrix accounts
    // for any super-sampling that might be in effect for the current LOD.
    oe_splat_covtc =($COVERAGE_TEXMAT_UNIFORM * oe_layer_tilec).st;

    // quantize the scale offset so we take the hit in the FS
    oe_splat_scaleOffsetInt = oe_splat_scaleOffset >= 0.0 ? ceil(oe_splat_scaleOffset) : floor(oe_splat_scaleOffset);
}
