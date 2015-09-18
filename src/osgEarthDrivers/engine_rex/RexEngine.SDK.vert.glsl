#version 330
#pragma vp_name "Rex Terrain SDK"

/**
 * SDK functions for the Rex engine.
 * Declare and call these from any shader that runs on the terrain.
 */

// uniforms from terrain engine
uniform sampler2D oe_tile_elevationTex;
uniform mat4 oe_tile_elevationTexMatrix;
uniform float oe_tile_elevationSize;
uniform vec4 oe_tile_key;

// Stage global
vec4 oe_layer_tilec;


/**
 * Sample the elevation data at a UV tile coordinate.
 */
float oe_terrain_getElevation(in vec2 uv)
{
    // Texel-level scale and bias allow us to sample the elevation texture
    // on texel center instead of edge.
    float texelScale = (oe_tile_elevationSize-1.0)/oe_tile_elevationSize;
    float texelBias  = 0.5/oe_tile_elevationSize;

    // Apply the scale and bias.
    vec2 elevc = uv
        * texelScale * oe_tile_elevationTexMatrix[0][0]     // scale
        + texelScale * oe_tile_elevationTexMatrix[3].st     // bias
        + texelBias;

    return texture(oe_tile_elevationTex, elevc).r;
}


/**
 * Read the elevation at the build-in tile coordinates (convenience)
 */
float oe_terrain_getElevation()
{
    return oe_terrain_getElevation(oe_layer_tilec.st);
}


/**
 * Scales repeating texture coordinate such that they are [0..1]
 * at a specific reference tile LOD. 
 */
vec2 oe_terrain_scaleCoordsToRefLOD(in vec2 tc, in float refLOD)
{
    float dL = oe_tile_key.z - floor(refLOD);
    float factor = exp2(dL);
    float invFactor = 1.0/factor;
    vec2 scale = vec2(invFactor);
    vec2 result = tc * scale;
    if ( factor >= 1.0 ) {
        vec2 a = floor(oe_tile_key.xy * invFactor);
        vec2 b = a * factor;
        vec2 c = (a+1.0) * factor;
        vec2 offset = (oe_tile_key.xy-b)/(c-b);
        result += offset;
    }
    return result;
}
