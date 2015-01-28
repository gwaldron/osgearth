#version 110

vec3 oe_global_Normal;

uniform vec4 oe_tile_key;
uniform sampler2D oe_nmap_normalTex;
varying vec2 oe_nmap_normalCoords;
varying mat3 oe_nmap_TBN;

void oe_nmap_fragment(inout vec4 color)
{
    //if ( oe_tile_key.z > 5.0 )
    {
        vec4 decoded = texture2D(oe_nmap_normalTex, oe_nmap_normalCoords)*2.0-1.0;
        vec3 normalTangent = normalize(decoded.xyz);
        oe_global_Normal = normalize(oe_nmap_TBN * normalTangent);
    }

    // visualize curvature:
    //color.rgb = vec3(0,0,0);
    //if(decoded.a >= 0.4) color.r = 1.0;
    //if(decoded.a <= -0.4) color.b = 1.0;

    // visualize normals:
    //color.rgb = (decoded.xyz+1.0)*0.5;
}
