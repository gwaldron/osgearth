#version 330 compatibility

#pragma vp_entryPoint "oe_mp_NormalMap_fragment"
#pragma vp_location   "fragment_coloring"
#pragma vp_order      "0.2"

in vec3 vp_Normal;

uniform sampler2D oe_tile_normalTex;

in vec2 oe_mp_NormalMap_coords;
flat in mat3 oe_mp_NormalMap_TBN;

void oe_mp_NormalMap_fragment(inout vec4 color)
{
    //const vec3 B = vec3(0,1,0);

    vec4 encodedNormal = texture2D(oe_tile_normalTex, oe_mp_NormalMap_coords);
    vec3 normalTangent = normalize(encodedNormal.xyz*2.0-1.0);

    vp_Normal = normalize(oe_mp_NormalMap_TBN * normalTangent);

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
