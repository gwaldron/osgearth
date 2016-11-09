#version $GLSL_VERSION_STR

#pragma vp_entryPoint oe_graticule_vertex
#pragma vp_location   vertex_view
#pragma vp_order      0.5

uniform vec4 oe_tile_key;
out vec4 oe_layer_tilec;
out vec2 oe_graticule_coord;


void oe_graticule_vertex(inout vec4 vertex)
{
    // calculate long and lat from [0..1] across the profile:
    vec2 r = (oe_tile_key.xy + oe_layer_tilec.xy)/exp2(oe_tile_key.z);
    oe_graticule_coord = vec2(0.5*r.x, r.y);
}
