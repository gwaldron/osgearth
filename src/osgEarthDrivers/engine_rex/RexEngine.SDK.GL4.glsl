#version 460
#pragma include RexEngine.GL4.glsl
#pragma vp_name Rex Terrain SDK
/**
 * SDK functions for the Rex engine.
 * Declare and call these from any shader that runs on the terrain.
 */

// Stage global
vec4 oe_layer_tilec;
vec4 oe_tile_key;

#if !defined(VP_STAGE_FRAGMENT)

uniform vec2 oe_tile_elevTexelCoeff;

// Sample the elevation data at a UV tile coordinate.
float oe_terrain_getElevation(in vec2 uv)
{
    // Texel-level scale and bias allow us to sample the elevation texture
    // on texel center instead of edge.
    vec2 uv_scaledBiased = uv
        * oe_tile_elevTexelCoeff.x * tile[oe_tileID].elevMat[0][0]     // scale
        + oe_tile_elevTexelCoeff.x * tile[oe_tileID].elevMat[3].st     // bias
        + oe_tile_elevTexelCoeff.y;

    return texture(sampler2D(tex[tile[oe_tileID].elevIndex]), uv_scaledBiased).r;
}

// Read the elevation at the build-in tile coordinates (convenience)
float oe_terrain_getElevation()
{
    return oe_terrain_getElevation(oe_layer_tilec.st);
}

// Read the normal vector and curvature at resolved UV tile coordinates.
vec4 oe_terrain_getNormalAndCurvature(in vec2 uv_scaledBiased)
{
    vec4 n = texture(sampler2D(tex[tile[oe_tileID].normalIndex]), uv_scaledBiased);
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
    vec2 uv_scaledBiased = oe_layer_tilec.st
        * oe_tile_elevTexelCoeff.x * tile[oe_tileID].normalMat[0][0]
        + oe_tile_elevTexelCoeff.x * tile[oe_tileID].normalMat[3].st
        + oe_tile_elevTexelCoeff.y;

    return oe_terrain_getNormalAndCurvature(uv_scaledBiased);
}

uint64_t oe_terrain_getNormalHandle()
{
    return tex[tile[oe_tileID].normalIndex];
}

vec2 oe_terrain_getNormalCoords()
{
    return oe_layer_tilec.st
        * oe_tile_elevTexelCoeff.x * tile[oe_tileID].normalMat[0][0]
        + oe_tile_elevTexelCoeff.x * tile[oe_tileID].normalMat[3].st
        + oe_tile_elevTexelCoeff.y;
}

#endif // !VP_STAGE_FRAGMENT

vec4 oe_terrain_getNormalAndCurvature(in uint64_t handle, in vec2 uv)
{
    vec4 n = texture(sampler2D(handle), uv);
    n.xyz = n.xyz*2.0 - 1.0;
    float curv = n.z;
    n.z = 1.0 - abs(n.x) - abs(n.y);
    // unnecessary since Z is never < 0:
    //float t = clamp(-n.z, 0, 1);
    //n.x += (n.x > 0)? -t : t;
    //n.y += (n.y > 0)? -t : t;
    return vec4(normalize(n.xyz), curv);
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
