#version $GLSL_VERSION_STR

#pragma vp_entryPoint oe_mp_NormalMap_fragment
#pragma vp_location   fragment_coloring
#pragma vp_order      0.2

uniform sampler2D oe_tile_normalTex;

in vec3 vp_Normal;
in vec2 oe_normalMapCoords;
in vec3 oe_normalMapBinormal;

void oe_mp_NormalMap_fragment(inout vec4 color)
{
    //const vec3 B = vec3(0,1,0);

    vec4 encodedNormal = texture2D(oe_tile_normalTex, oe_normalMapCoords);
    vec3 normal        = normalize(encodedNormal.xyz*2.0-1.0);

    //vp_Normal = normalize(oe_mp_NormalMap_TBN * normalTangent);

    vec3 tangent = normalize(cross(oe_normalMapBinormal, vp_Normal));

    vp_Normal = normalize(mat3(tangent, oe_normalMapBinormal, vp_Normal) * normal);

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
