#version 110
#pragma vp_entryPoint "oe_graticule_fragment"
#pragma vp_location   "fragment_coloring"

uniform float oe_graticule_lineWidth;
uniform float oe_graticule_resolution;
uniform vec4  oe_graticule_color;

varying vec2 oe_graticule_coord;

void oe_graticule_fragment(inout vec4 color)
{
    // double the effective res for longitude since it has twice the span
    vec2 gr = vec2(0.5*oe_graticule_resolution, oe_graticule_resolution);
    vec2 distanceToLine = mod(oe_graticule_coord, gr);
    vec2 dx = abs(dFdx(oe_graticule_coord));
    vec2 dy = abs(dFdy(oe_graticule_coord));
    vec2 dF = vec2(max(dx.s, dy.s), max(dx.t, dy.t)) * oe_graticule_lineWidth;
        
    if ( any(lessThan(distanceToLine, dF)) )
    {
        color.rgb = mix(color.rgb, oe_graticule_color.rgb, oe_graticule_color.a);
    }
}