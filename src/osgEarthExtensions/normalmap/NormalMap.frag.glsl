#version 110
#pragma vp_entryPoint "oe_nmap_fragment"
#pragma vp_location   "fragment_coloring"
#pragma vp_order      "0.2"

// stage global:
vec3 oe_global_Normal;

uniform vec4 oe_tile_key;
uniform sampler2D oe_nmap_normalTex;
varying vec4 oe_nmap_normalCoords;
varying mat3 oe_nmap_TBN;

void oe_nmap_fragment(inout vec4 color)
{
    vec4 encodedNormal = texture2D(oe_nmap_normalTex, oe_nmap_normalCoords.st);
    vec3 normalTangent = normalize(encodedNormal.xyz*2.0-1.0);
    oe_global_Normal = normalize(oe_nmap_TBN * normalTangent);

    // visualize curvature:
    //color.rgb = vec3(0,0,0);
    //if(decoded.a >= 0.4) color.r = 1.0;
    //if(decoded.a <= -0.4) color.b = 1.0;

    // visualize normals:
    //color.rgb = encodedNormal.xyz;
}
