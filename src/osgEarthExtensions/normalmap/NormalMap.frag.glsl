#version 330 compatibility

#pragma vp_entryPoint "oe_nmap_fragment"
#pragma vp_location   "fragment_coloring"
#pragma vp_order      "0.2"

in vec3 vp_Normal;

uniform vec4 oe_tile_key;
uniform sampler2D oe_tile_normalTex;

in vec4 oe_nmap_normalCoords;
flat in vec3 oe_nmap_T;
flat in vec3 oe_nmap_N;

void oe_nmap_fragment(inout vec4 color)
{
    const vec3 B = vec3(0,1,0);

    vec4 encodedNormal = texture2D(oe_tile_normalTex, oe_nmap_normalCoords.st);
    vec3 normalTangent = normalize(encodedNormal.xyz*2.0-1.0);
    
    mat3 TBM = gl_NormalMatrix * mat3(oe_nmap_T, B, oe_nmap_N);

    vp_Normal = normalize(TBM * normalTangent);

    // visualize curvature gradient:
    //color.rgb = vec3(0,0,0);
    //color.r = (encodedNormal.a+1.0)/2.0;
    //color.b = 1.0-color.r;

    // visualize curvature quantized:
    //if(encodedNormal.a >= 0.4) color.r = 1.0;
    //if(encodedNormal.a <= -0.4) color.b = 1.0;

    // visualize normals:
    //color.rgb = encodedNormal.xyz;
}
