// begin: Splat.frag.common.glsl

// TODO:
// Encapsulate the "use normal map" define logic in the terrain SDK itself.

#pragma vp_define OE_USE_NORMAL_MAP

#ifdef OE_USE_NORMAL_MAP

// import SDK
vec4 oe_terrain_getNormalAndCurvature(in vec2);

// normal map version:
in vec2 oe_normalMapCoords;

float oe_splat_getSlope()
{
    vec4 encodedNormal = oe_terrain_getNormalAndCurvature( oe_normalMapCoords );
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