#version $GLSL_VERSION_STR

#pragma vp_entryPoint oe_GeodeticGraticule_vertex
#pragma vp_location   vertex_view
#pragma vp_order      0.5

#pragma import_defines(OE_DISABLE_GRATICULE)

uniform vec4 oe_tile_key;
out vec4 oe_layer_tilec;
out vec2 oe_GeodeticGraticule_coord;


void oe_GeodeticGraticule_vertex(inout vec4 vertex)
{
#ifndef OE_DISABLE_GRATICULE
    // calculate long and lat from [0..1] across the profile:
    vec2 r = (oe_tile_key.xy + oe_layer_tilec.xy)/exp2(oe_tile_key.z);
    oe_GeodeticGraticule_coord = vec2(0.5*r.x, r.y);
#endif
}
