#version 110
uniform sampler2D oe_splat_coverage_tex;

// Environment structure passed around locally.
struct oe_SplatEnv {
    float range;
    float elevation;
    vec4 noise;
};

// Rendering parameters for splat texture and noise-based detail texture.
struct oe_SplatRenderInfo {
    float primaryIndex;
    float detailIndex;
    float saturation;
    float threshold;
    float minSlope;
};

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
