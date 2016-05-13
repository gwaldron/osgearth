// begin: Splat.types.glsl

// Environment structure passed around locally.
struct oe_SplatEnv {
    float lod;
    float range;
    float lod0;
    float lod1;
    float lodLo;
    float lodHi;
    float rangeHi;
    float rangeLo;
    float elevation;
    float slope;
    float side;       //0=lo, 1=hi
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