#version 330 compatibility

#pragma vp_entryPoint "oe_mp_NormalMap_vertex"
#pragma vp_location   "vertex_view"
#pragma vp_order      "0.5"

uniform mat4 oe_tile_normalTexMatrix;

// stage globals
vec3 vp_Normal;
vec4 oe_layer_tilec;

out vec2 oe_mp_NormalMap_coords;
flat out mat3 oe_mp_NormalMap_TBN;

void oe_mp_NormalMap_vertex(inout vec4 unused)
{
    // calculate the sampling coordinates for the normal texture
    oe_mp_NormalMap_coords = (oe_tile_normalTexMatrix * oe_layer_tilec).st;

    // form the matrix that will transform a normal vector from
    // tangent space to model space in the fragment shader.
    // We expect the vertex normal to be a simple UP vector.
    vec3 B = gl_NormalMatrix * vec3(0,1,0);
    vec3 T = normalize(cross(B, vp_Normal));
    oe_mp_NormalMap_TBN = mat3(T, B, vp_Normal);
}
