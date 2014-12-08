#version 110

vec3 oe_global_Normal;

uniform sampler2D oe_nmap_normalTex;
varying vec2 oe_nmap_normalCoords;

void oe_nmap_fragment(inout vec4 color)
{
    vec4 decoded = texture2D(oe_nmap_normalTex, oe_nmap_normalCoords)*2.0-1.0;
    oe_global_Normal = gl_NormalMatrix * normalize(decoded.xyz);

    // visualize curvature:
    //color.rgb = vec3(0,0,0);
    //if(decoded.a >= 0.4) color.r = 1.0;
    //if(decoded.a <= -0.4) color.b = 1.0;
}
