#version 120

varying vec3 oe_Normal;
varying float oe_splat_slope;

void oe_splat_vertex_model(inout vec4 VertexMODEL)
{
    // calculate slope from the Z component of the current normal
    // since the terrain is in LTP space.
    oe_splat_slope = 1.0-oe_Normal.z;
}
