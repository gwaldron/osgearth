#pragma vp_name Rex Terrain SDK

#pragma import_defines(OE_MESA_23_WORKAROUND)
#ifdef OE_MESA_23_WORKAROUND
#define TILE_COORDS gl_MultiTexCoord0
#else
#define TILE_COORDS oe_layer_tilec
#endif

uniform sampler2D oe_tile_elevationTex;
uniform mat4 oe_tile_elevationTexMatrix;
uniform sampler2D oe_tile_normalTex;
uniform mat4 oe_tile_normalTexMatrix;

// SDK functions for the Rex engine.
// Declare and call these from any shader that runs on the terrain.

// uniforms from terrain engine
uniform vec2 oe_tile_elevTexelCoeff;

// Stage global
vec4 oe_layer_tilec;
vec4 oe_tile_key;

// Sample the elevation data at a UV tile coordinate.
float oe_terrain_getElevation(in vec2 uv)
{
    // Texel-level scale and bias allow us to sample the elevation texture
    // on texel center instead of edge.
    vec2 uv_scaledBiased = uv
        * oe_tile_elevTexelCoeff.x * oe_tile_elevationTexMatrix[0][0]     // scale
        + oe_tile_elevTexelCoeff.x * oe_tile_elevationTexMatrix[3].st     // bias
        + oe_tile_elevTexelCoeff.y;

    return texture(oe_tile_elevationTex, uv_scaledBiased).r;
}

// Read the elevation at the build-in tile coordinates (convenience)
float oe_terrain_getElevation()
{
    return oe_terrain_getElevation(TILE_COORDS.st);
}

// Read the normal vector and curvature at resolved UV tile coordinates.
vec4 oe_terrain_getNormalAndCurvature(in vec2 uv_scaledBiased)
{
    vec4 n = texture(oe_tile_normalTex, uv_scaledBiased);
    n.xyz = n.xyz*2.0-1.0;
    float curv = n.z;
    n.z = 1.0 - abs(n.x) - abs(n.y);
    // unnecessary since Z is never < 0:
    //float t = clamp(-n.z, 0, 1);
    //n.x += (n.x > 0)? -t : t;
    //n.y += (n.y > 0)? -t : t;
    return vec4(normalize(n.xyz), curv);
}

vec4 oe_terrain_getNormalAndCurvature()
{
    vec2 uv_scaledBiased = TILE_COORDS.st
        * oe_tile_elevTexelCoeff.x * oe_tile_normalTexMatrix[0][0]
        + oe_tile_elevTexelCoeff.x * oe_tile_normalTexMatrix[3].st
        + oe_tile_elevTexelCoeff.y;

    return oe_terrain_getNormalAndCurvature(uv_scaledBiased);
}

/**
 * Scales repeating texture coordinate such that they are [0..1]
 * at a specific reference tile LOD. 
 */
vec2 oe_terrain_scaleCoordsToRefLOD(in vec2 tc, in float refLOD)
{
    float dL = oe_tile_key.z - refLOD;
    float factor = exp2(dL);
    float invFactor = 1.0/factor;
    vec2 result = tc * vec2(invFactor);

    vec2 a = floor(oe_tile_key.xy * invFactor);
    vec2 b = a * factor;
    vec2 c = b + factor;

    float m = floor(clamp(factor,0.0,1.0)); // if factor>=1.0
    result += m*(oe_tile_key.xy-b)/(c-b);

    return result;
}

/**
 * Scales repeating texture coordinate such that they are [0..1]
 * at a specific reference tile LOD.
 */
vec4 oe_terrain_scaleCoordsAndTileKeyToRefLOD(in vec2 tc, in float refLOD)
{
    float dL = oe_tile_key.z - refLOD;
    float factor = exp2(dL);
    float invFactor = 1.0 / factor;
    vec2 result = tc * vec2(invFactor);

    vec2 a = floor(oe_tile_key.xy * invFactor);
    vec2 b = a * factor;
    vec2 c = b + factor;

    float m = floor(clamp(factor, 0.0, 1.0)); // if factor>=1.0
    result += m * (oe_tile_key.xy - b) / (c - b);

    return vec4(result, a);
}

