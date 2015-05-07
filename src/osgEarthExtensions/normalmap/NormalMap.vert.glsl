#version 110
#pragma vp_entryPoint "oe_nmap_vertex"
#pragma vp_location   "vertex_model"
#pragma vp_order      "0.5"

varying vec3 oe_Normal;
varying vec4 oe_layer_tilec;

uniform mat4 oe_nmap_normalTexMatrix;
varying vec4 oe_nmap_normalCoords;
varying mat3 oe_nmap_TBN;

void oe_nmap_vertex(inout vec4 VertexMODEL)
{
    // calculate the sampling coordinates for the normal texture
    oe_nmap_normalCoords = oe_nmap_normalTexMatrix * oe_layer_tilec;

    // form the matrix that will transform a normal vector from
    // tangent space to model space in the fragment shader.
    // We expect the vertex normal to be a simple UP vector.
    vec3 B = vec3(0,1,0);
    vec3 N = oe_Normal;
    vec3 T = normalize(cross(B,N));
    oe_nmap_TBN = gl_NormalMatrix * mat3(T, B, N);
}