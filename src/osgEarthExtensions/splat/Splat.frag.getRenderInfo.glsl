#version 330

#pragma include "Splat.types.glsl"

// Samples the coverage data and returns main and detail indices.
oe_SplatRenderInfo oe_splat_getRenderInfo(in float value, in oe_SplatEnv env)
{
    float primary = -1.0;   // primary texture index
    float detail = -1.0;    // detail texture index
    float brightness = 1.0; // default noise function brightness factor
    float contrast = 1.0;   // default noise function contrast factor
    float threshold = 0.0;  // default noise function threshold
    float slope = 0.0;      // default minimum slope

    $CODE_INJECTION_POINT

    return oe_SplatRenderInfo(primary, detail, brightness, contrast, threshold, slope);
}
