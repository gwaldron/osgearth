// begin: Splat.frag.common.glsl

#pragma vp_define "OE_USE_NORMAL_MAP"
#ifdef OE_USE_NORMAL_MAP

// normal map version:
uniform sampler2D oe_nmap_normalTex;
in vec4 oe_nmap_normalCoords;

float oe_splat_getSlope()
{
    vec4 encodedNormal = texture2D(oe_nmap_normalTex, oe_nmap_normalCoords.st);
    vec3 normalTangent = normalize(encodedNormal.xyz*2.0-1.0);
    return clamp((1.0-normalTangent.z)/0.8, 0.0, 1.0);
}

#else // !OE_USE_NORMAL_MAP

// non- normal map version:
in float oe_splat_slope;

float oe_splat_getSlope()
{
    return oe_splat_slope;
}

#endif // OE_USE_NORMAL_MAP

// end: Splat.frag.common.glsl