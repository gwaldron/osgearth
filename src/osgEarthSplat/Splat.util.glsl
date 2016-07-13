#version 120
#pragma vp_location fragment_coloring

uniform vec4 oe_tile_key;  // osgEarth TileKey

// Number of LOD range. Do not increase this past 25; doing so will result in precision errors
// and rendering artifacts when the camera is very close to the ground.
#define LOD_COUNT 25

const float oe_SplatRanges[LOD_COUNT] = float[](
       100000000.0, // 0
        75000000.0, // 1
        50000000.0, // 2
        10000000.0, // 3
         7500000.0, // 4
         5000000.0, // 5
         2500000.0, // 6
         1000000.0, // 7
          500000.0, // 8
          225000.0, // 9
          150000.0, // 10
           80000.0, // 11
           30000.0, // 12
           14000.0, // 13
            4000.0, // 14
            2500.0, // 15
            1000.0, // 16
             500.0, // 17
             250.0, // 18
             125.0, // 19
              50.0, // 20
              25.0, // 21
              12.0, // 22
               6.0, // 23
               3.0  // 24
    );

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
oe_splat_getLodBlend(in float range, out float out_LOD0, out float out_rangeOuter, out float out_rangeInner, out float out_clampedRange)
{
    out_clampedRange = clamp(range, oe_SplatRanges[LOD_COUNT-1], oe_SplatRanges[0]);

    out_LOD0 = 0;

    for(int i=0; i<LOD_COUNT-1; ++i)
    {
        if ( out_clampedRange < oe_SplatRanges[i] && out_clampedRange >= oe_SplatRanges[i+1] )
        {
            out_LOD0 = float(i); //   + baseLOD;
            break;
        }
    }

    out_rangeOuter = oe_SplatRanges[int(out_LOD0)];
    out_rangeInner = oe_SplatRanges[int(out_LOD0)+1];
}

float
oe_splat_getRangeForLod(in float lod)
{
    return oe_SplatRanges[int(lod)];
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
    vec2 a = floor(oe_tile_key.xy * invFactor);
    vec2 b = a * factor;
    vec2 c = (a+1.0) * factor;
    vec2 offset = (oe_tile_key.xy-b)/(c-b);

    // only apply offset if factor >= 1.0
    float m = floor(clamp(factor,0.0,1.0));
    result += (m*offset);

    return result;
}

