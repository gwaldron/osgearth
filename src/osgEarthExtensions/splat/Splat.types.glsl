// begin: Splat.types.glsl

// Environment structure passed around locally.
struct oe_SplatEnv {
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


// Mapping of view ranges to splat texture levels of detail.
#define RANGE_COUNT 9
const float oe_SplatRanges[RANGE_COUNT] = float[](  250.0, 500.0, 1000.0, 4000.0, 30000.0, 150000.0, 300000.0, 1000000.0, 5000000.0 );
const float oe_SplatLevels[RANGE_COUNT] = float[](   18.0,  17.0,   16.0,   14.0,    12.0,     10.0,      8.0,       6.0,       4.0 );


// end: Splat.types.glsl