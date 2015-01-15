// begin: Splat.types.glsl

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

// end: Splat.types.glsl