#version 330 compatibility

#pragma vp_entryPoint "oe_nmap_vertex"
#pragma vp_location   "vertex_model"
#pragma vp_order      "0.5"

uniform mat4 oe_tile_normalTexMatrix;

// stage globals
vec3 vp_Normal;
vec4 oe_layer_tilec;

out vec4 oe_nmap_normalCoords;

flat out vec3 oe_nmap_T;
flat out vec3 oe_nmap_N;

void oe_nmap_vertex(inout vec4 VertexMODEL)
{
    // calculate the sampling coordinates for the normal texture
    oe_nmap_normalCoords = oe_tile_normalTexMatrix * oe_layer_tilec;

    // form the matrix that will transform a normal vector from
    // tangent space to model space in the fragment shader.
    // We expect the vertex normal to be a simple UP vector.
#if 0
    vec3 B = vec3(0,1,0);
    vec3 N = vp_Normal;
    vec3 T = normalize(cross(B,N));
    oe_nmap_TBN = gl_NormalMatrix * mat3(T, B, N);
#endif

    const vec3 B = vec3(0,1,0);
    oe_nmap_N = vp_Normal;
    oe_nmap_T = normalize(cross(B, vp_Normal));
}
