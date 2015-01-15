#version 110

$include "Splat.types.glsl"

uniform sampler2D oe_splat_coverage_tex;

// Samples the coverage data and returns main and detail indices.
oe_SplatRenderInfo oe_splat_getRenderInfo(in vec2 tc, in oe_SplatEnv env)
{
    float primary = -1.0;   // primary texture index
    float detail = -1.0;    // detail texture index
    float saturation = 0.0; // default noise function saturation factor
    float threshold = 0.0;  // default noise function threshold
    float minSlope = 0.0;   // default minimum slope
    float value = 255.0 * texture2D(oe_splat_coverage_tex, tc).r;
$CODE_INJECTION_POINT
    return oe_SplatRenderInfo(primary, detail, saturation, threshold, minSlope);
}
