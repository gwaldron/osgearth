#version 110

vec3 oe_global_Normal;

uniform sampler2D oe_nmap_normalTex;
varying vec2 oe_nmap_normalCoords;

void oe_nmap_fragment(inout vec4 color)
{
    vec3 encodedNormal = texture2D(oe_nmap_normalTex, oe_nmap_normalCoords).xyz;
    oe_global_Normal = gl_NormalMatrix * normalize(encodedNormal*2.0 - 1.0);

    // TEST:
    //color.rgb = encodedNormal;
}
