#version 330
#pragma vp_entryPoint "oe_splat_vertex_model"
#pragma vp_location   "vertex_model"
#pragma vp_order      "0.5"

out vec3 vp_Normal;
out float oe_splat_slope;

void oe_splat_vertex_model(inout vec4 VertexMODEL)
{
    // calculate slope from the Z component of the current normal
    // since the terrain is in LTP space.
    oe_splat_slope = 1.0-vp_Normal.z;
}
