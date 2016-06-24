#version $GLSL_VERSION_STR

#pragma vp_entryPoint oe_mp_NormalMap_vertex
#pragma vp_location   vertex_view
#pragma vp_order      0.5

uniform mat4 oe_tile_normalTexMatrix;

// stage globals
vec3 vp_Normal;
vec4 oe_layer_tilec;

out vec2 oe_normalMapCoords;
out vec3 oe_normalMapBinormal;

void oe_mp_NormalMap_vertex(inout vec4 unused)
{
    // calculate the sampling coordinates for the normal texture
    oe_normalMapCoords = (oe_tile_normalTexMatrix * oe_layer_tilec).st;

    // send the bi-normal vector to the fragment shader.
    oe_normalMapBinormal = gl_NormalMatrix * vec3(0,1,0);
}
