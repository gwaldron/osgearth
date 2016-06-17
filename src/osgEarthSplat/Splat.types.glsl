// begin: Splat.types.glsl

// Environment structure passed around locally.
struct oe_SplatEnv {
    float lod;
    float range;
    float elevation;
    float slope;
    vec4 noise;
};

// Rendering parameters for splat texture and noise-based detail texture.
struct oe_SplatRenderInfo {
    float primaryIndex;
    float detailIndex;
    float brightness;
    float contrast;
    float threshold;
    float minSlope;
};

// end: Splat.types.glsl