#version $GLSL_VERSION_STR

#pragma vp_entryPoint oe_splat_vertex_model
#pragma vp_location   vertex_model
#pragma vp_order      0.5

vec3 vp_Normal; // stage global
out float oe_splat_slope;

void oe_splat_vertex_model(inout vec4 VertexMODEL)
{
    // calculate slope from the Z component of the current normal
    // since the terrain is in LTP space. This is only used when normal maps
    // are not available, which is hopefully never :/
    oe_splat_slope = 1.0-vp_Normal.z;
}
