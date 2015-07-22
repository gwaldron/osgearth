#version 120
#pragma vp_location "fragment_coloring"

uniform vec4 oe_tile_key;  // osgEarth TileKey


// Mapping of view ranges to splat texture levels of detail.
#define RANGE_COUNT 11
const float oe_SplatRanges[RANGE_COUNT] = float[](  50.0, 125.0, 250.0, 500.0, 1000.0, 4000.0, 30000.0, 150000.0, 300000.0, 1000000.0, 5000000.0 );
const float oe_SplatLevels[RANGE_COUNT] = float[](  20.0,  19.0,  18.0,  17.0,   16.0,   14.0,    12.0,     10.0,      8.0,       6.0,       4.0 );

/**
 * Given a camera distance, return the two LODs it falls between and
 * the blend factor [0..1] between then.
 * in  range   = camera distace to fragment
 * in  baseLOD = LOD at which texture scale is 1.0
 * out LOD0    = near LOD
 * out LOD1    = far LOD
 * out blend   = Blend factor between LOD0 and LOD1 [0..1]
 */
void
oe_splat_getLodBlend(in float range, in float baseLOD, out float out_LOD0, out float out_LOD1, out float out_blend)
{
    float clampedRange = clamp(range, oe_SplatRanges[0], oe_SplatRanges[RANGE_COUNT-1]);

    out_blend = -1.0;
    for(int i=0; i<RANGE_COUNT-1 && out_blend < 0; ++i)
    {
        if ( clampedRange >= oe_SplatRanges[i] && clampedRange <= oe_SplatRanges[i+1] )
        {
            out_LOD0 = oe_SplatLevels[i]   + baseLOD;
            out_LOD1 = oe_SplatLevels[i+1] + baseLOD;
            out_blend = clamp((clampedRange-oe_SplatRanges[i])/(oe_SplatRanges[i+1]-oe_SplatRanges[i]), 0.0, 1.0);
        }
    }
}

/**
 * Scales the incoming tile splat coordinates to match the requested
 * LOD level. We offset the level from the current tile key's LOD (.z)
 * because otherwise you run into single-precision jitter at high LODs.
 */
vec2 
oe_splat_getSplatCoords(in vec2 tc, float lod)
{
    float dL = oe_tile_key.z - lod;
    float factor = exp2(dL);
    float invFactor = 1.0/factor;
    vec2 scale = vec2(invFactor); 
    vec2 result = tc * scale;

    // For upsampling we need to calculate an offset as well
    if ( factor >= 1.0 )
    {
        vec2 a = floor(oe_tile_key.xy * invFactor);
        vec2 b = a * factor;
        vec2 c = (a+1.0) * factor;
        vec2 offset = (oe_tile_key.xy-b)/(c-b);
        result += offset;
    }

    return result;
}

