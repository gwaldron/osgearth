#version $GLSL_VERSION_STR

#pragma vp_entryPoint oe_GeodeticGraticule_vertex
#pragma vp_location   vertex_view
#pragma vp_order      0.5

#pragma import_defines(OE_DISABLE_GRATICULE)

out vec4 oe_layer_tilec;
out vec2 oe_GeodeticGraticule_coord;

vec4 oe_tile_key;

void oe_GeodeticGraticule_vertex(inout vec4 vertex)
{
#ifndef OE_DISABLE_GRATICULE
    // calculate long and lat from [0..1] across the profile:
    vec2 r = (oe_tile_key.xy + oe_layer_tilec.xy)/exp2(oe_tile_key.z);
    oe_GeodeticGraticule_coord = vec2(0.5*r.x, r.y);
#endif
}


[break]

#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_GeodeticGraticule_fragment
#pragma vp_location   fragment_lighting
#pragma vp_order      1.1

#pragma import_defines(OE_DISABLE_GRATICULE)

uniform float oe_GeodeticGraticule_lineWidth;
uniform float oe_GeodeticGraticule_resolution;
uniform vec4  oe_GeodeticGraticule_color;
uniform mat4 osg_ViewMatrixInverse;

in vec2 oe_GeodeticGraticule_coord;

void oe_GeodeticGraticule_fragment(inout vec4 color)
{
#ifndef OE_DISABLE_GRATICULE
    // double the effective res for longitude since it has twice the span
    vec2 gr = vec2(0.5*oe_GeodeticGraticule_resolution, oe_GeodeticGraticule_resolution);
    vec2 distanceToLine = mod(oe_GeodeticGraticule_coord, gr);
    vec2 dx = abs(dFdx(oe_GeodeticGraticule_coord));
    vec2 dy = abs(dFdy(oe_GeodeticGraticule_coord));
    vec2 dF = vec2(max(dx.s, dy.s), max(dx.t, dy.t)) * oe_GeodeticGraticule_lineWidth;

    if ( any(lessThan(distanceToLine, dF)))
    {
        // calculate some anti-aliasing
        vec2 f = distanceToLine/dF;
        float antialias = 1.0 - 2.0*abs(0.5 - min(f.x,f.y));

        // Fade out the lines as you get closer to the ground.
        vec3 eye = osg_ViewMatrixInverse[3].xyz;
        float hae = length(eye) - 6378137.0;
        float maxHAE = 2000.0;
        float alpha = clamp(hae / maxHAE, 0.0, 1.0) * antialias;
        if (antialias < 0.)
          alpha = 0.;
        color.rgb = mix(color.rgb, oe_GeodeticGraticule_color.rgb, oe_GeodeticGraticule_color.a * alpha);
    }
#endif
}
