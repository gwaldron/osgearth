#version $GLSL_VERSION_STR

#pragma vp_location fragment_coloring

// Number of LOD range. Do not increase this past 25; doing so will result in precision errors
// and rendering artifacts when the camera is very close to the ground.
#define LOD_COUNT 26

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
               3.0, // 24
               1.0  // 25
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
