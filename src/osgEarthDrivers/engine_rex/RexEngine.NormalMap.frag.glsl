#version $GLSL_VERSION_STR

#pragma vp_entryPoint oe_normalMapFragment
#pragma vp_location   fragment_coloring
#pragma vp_order      0.2

// import terrain SDK
vec4 oe_terrain_getNormalAndCurvature(in vec2);

uniform sampler2D oe_tile_normalTex;

in vec3 vp_Normal;
in vec3 oe_UpVectorView;
in vec2 oe_normalMapCoords;
in vec3 oe_normalMapBinormal;

void oe_normalMapFragment(inout vec4 color)
{
    vec4 encodedNormal = oe_terrain_getNormalAndCurvature(oe_normalMapCoords);
    //vec4 encodedNormal = texture2D(oe_tile_normalTex, oe_normalMapCoords);
    vec3 normal = normalize(encodedNormal.xyz*2.0-1.0);

    vec3 tangent = normalize(cross(oe_normalMapBinormal, oe_UpVectorView));
    vp_Normal = normalize( mat3(tangent, oe_normalMapBinormal, oe_UpVectorView) * normal );

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
