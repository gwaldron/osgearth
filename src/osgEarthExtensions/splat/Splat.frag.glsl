#version 130
#extension GL_EXT_texture_array : enable

$SPLAT_EDIT     // replaced at runtime

// from the terrain engine:
varying vec4 oe_layer_tilec;
uniform vec4 oe_tile_key;

// from the vertex shader:
varying float oe_splat_slope;
varying vec2 oe_splat_covtc;
varying float oe_splat_range;
varying float oe_splat_scaleOffsetInt;

// from SplatTerrainEffect:
uniform float oe_splat_intensity;
uniform float oe_splat_warp;
uniform float oe_splat_blur;
uniform float oe_splat_covlod;
uniform sampler2D oe_splat_coverage_tex;
uniform sampler2DArray oe_splat_tex;

uniform sampler2D oe_terrain_tex;
uniform mat4 oe_terrain_tex_mat;
uniform float oe_splat_snow;
uniform float oe_splat_detail_range;

// noise controllers.
// we will probably replace the first 4 with a texture at some point.
uniform float oe_splat_freq;
uniform float oe_splat_pers;
uniform float oe_splat_lac;
uniform float oe_splat_octaves;
uniform float oe_splat_thresh;
uniform float oe_splat_slopeFactor;
uniform float oe_splat_saturate;

// See NoiseShaders
float oe_splat_noise2(in vec2);
float oe_noise_fractal_2d(in vec2 seed, in float freq, in float pers, in float lac, in int octaves);

// Environment structure passed around locally.
// (reminder: struct defs cannot include newlines for GLES)
struct oe_SplatEnv {
    float range;
    float elevation;
    float noise;
};

// Rendering parameters for splat texture and noise-based detail texture.
struct oe_SplatRenderInfo {
    float primaryIndex;
    float detailIndex;
    float saturation;
    float threshold;
    float slope;
};
        
// permutation vectors for the warper.
// highly unlikely that we will use more than 2 or 3 of these in practice.
#define OE_SPLAT_MAX_SAMPLES 15
const vec2 oe_splat_warpVecs[OE_SPLAT_MAX_SAMPLES] = vec2[]( vec2( -0.942016, -0.399062 ), vec2( 0.845586, -0.768907 ), vec2( 0.344959, 0.293878 ), vec2( -0.915886, 0.457714 ), vec2( -0.815442, -0.879125 ), vec2( -0.382775, 0.276768 ), vec2( 0.974844, 0.756484 ), vec2( 0.443233, -0.975116 ), vec2( 0.53743, -0.473734 ), vec2( -0.264969, -0.41893 ), vec2( 0.791975, 0.190909 ), vec2( -0.241888, 0.997065 ), vec2( -0.8141, 0.914376 ), vec2( 0.199841, 0.786414 ), vec2( 0.143832, -0.141008 ));

// Warps the coverage sampling coordinates to mitigate blockiness.
vec2 oe_splat_warpCoverageCoords(int sample, in vec2 splat_tc)
{
    vec2 v = oe_splat_warpVecs[sample];
    vec2 seed = oe_splat_covtc + v;
    float n1 = oe_splat_noise2(seed*100.0);
    vec2 tc = oe_splat_covtc + n1*v*oe_splat_warp;
    return clamp(tc, 0.0, 1.0);
}

vec4 oe_splat_getTexel(in float index, in vec2 tc)
{
    return texture2DArray(oe_splat_tex, vec3(tc, index));
}

// Samples a detail texel using its render info parameters.
// Returns the weighting factor in the alpha channel.
vec4 oe_splat_getDetailTexel(in oe_SplatRenderInfo ri, in vec2 tc, in oe_SplatEnv env)
{
    // TODO: pow() function is slow. Consider something faster
    float hasDetail = ri.detailIndex >= 0.0 ? 1.0 : 0.0;
#ifdef SPLAT_EDIT
    float noiseADJ = pow(env.noise, 1.0-oe_splat_saturate);
    float threshADJ = clamp( oe_splat_thresh + oe_splat_slopeFactor*(1.0-oe_splat_slope), 0.0, 1.0);
#else
    float noiseADJ = pow(env.noise, 1.0-ri.saturation);
    float threshADJ = clamp( ri.threshold + ri.slope*(1.0-oe_splat_slope), 0.0, 1.0);
#endif
    vec4 result = oe_splat_getTexel( max(ri.detailIndex,0), tc);
    result.a = hasDetail * clamp((noiseADJ-threshADJ)/(1.0-threshADJ),0.0,1.0);
    return result;
}

// Samples the coverage data and returns main and detail indices.
oe_SplatRenderInfo oe_splat_getRenderInfo(in vec2 tc, in oe_SplatEnv env)
{
    float primary = -1.0;   // primary texture index
    float detail = -1.0;    // detail texture index
    float saturation = 0.0; // default noise function saturation factor
    float threshold = 0.0;  // default noise function threshold
    float slope = 0.0;      // default slope integration factor
    float value = 255.0 * texture2D(oe_splat_coverage_tex, tc).r;
$COVERAGE_BUILD_RENDER_INFO
    return oe_SplatRenderInfo(primary, detail, saturation, threshold, slope);
}

// Generates a texel using nearest-neighbor coverage sampling.
vec4 oe_splat_nearest(in vec2 splat_tc, in oe_SplatEnv env)
{
    vec2 warped_tc = oe_splat_warpCoverageCoords(0, splat_tc);
    oe_SplatRenderInfo ri = oe_splat_getRenderInfo(warped_tc, env);
    return oe_splat_getTexel(ri.primaryIndex, splat_tc);
}

// Generates a texel using bilinear filtering on the coverage data.
vec4 oe_splat_bilinear(in vec2 splat_tc, in oe_SplatEnv env)
{
    vec4 texel = vec4(0,0,0,1);

    //TODO: coverage warping is slow due to the noise function. Consider removing/reworking.
    vec2 warped_tc = oe_splat_warpCoverageCoords(0, splat_tc);

    float a = oe_splat_blur;
    float pixelWidth = a/256.0; // 256 = hard-coded cov tex size //TODO 
    float halfPixelWidth = 0.5*pixelWidth;
    float pixelWidth2 = pixelWidth*pixelWidth;

    // Find the four quantized coverage coordinates that form a box around the actual
    // coverage coordinates, where each quantized coord is at the center of a coverage texel.
    vec2 rem = mod(warped_tc, pixelWidth);
    vec2 sw = vec2(0);
    sw.x = warped_tc.x - rem.x + (rem.x >= halfPixelWidth ? halfPixelWidth : -halfPixelWidth);
    sw.y = warped_tc.y - rem.y + (rem.y >= halfPixelWidth ? halfPixelWidth : -halfPixelWidth);
    vec2 ne = sw + pixelWidth;
    vec2 nw = vec2(sw.x, ne.y);
    vec2 se = vec2(ne.x, sw.y);

    // Calculate the weighting for each corner.
    vec2 dsw = warped_tc-sw;
    vec2 dse = warped_tc-se;
    vec2 dne = warped_tc-ne;
    vec2 dnw = warped_tc-nw;

    float sw_weight = max(pixelWidth2-dot(dsw,dsw),0.0);
    float se_weight = max(pixelWidth2-dot(dse,dse),0.0);
    float ne_weight = max(pixelWidth2-dot(dne,dne),0.0);
    float nw_weight = max(pixelWidth2-dot(dnw,dnw),0.0);

    // normalize the weights so they total 1.0
    float invTotalWeight = 1.0/(sw_weight+se_weight+ne_weight+nw_weight);
    sw_weight *= invTotalWeight;
    se_weight *= invTotalWeight;
    ne_weight *= invTotalWeight;
    nw_weight *= invTotalWeight;

    // Sample coverage values using quantized corner coords:
    oe_SplatRenderInfo ri_sw = oe_splat_getRenderInfo( clamp(sw, 0.0, 1.0), env );
    oe_SplatRenderInfo ri_se = oe_splat_getRenderInfo( clamp(se, 0.0, 1.0), env );
    oe_SplatRenderInfo ri_ne = oe_splat_getRenderInfo( clamp(ne, 0.0, 1.0), env );
    oe_SplatRenderInfo ri_nw = oe_splat_getRenderInfo( clamp(nw, 0.0, 1.0), env );

    // Primary splat:
    vec3 sw_primary = oe_splat_getTexel(ri_sw.primaryIndex, splat_tc).rgb;
    vec3 se_primary = oe_splat_getTexel(ri_se.primaryIndex, splat_tc).rgb;
    vec3 ne_primary = oe_splat_getTexel(ri_ne.primaryIndex, splat_tc).rgb;
    vec3 nw_primary = oe_splat_getTexel(ri_nw.primaryIndex, splat_tc).rgb;

    // Detail splat - weighting is in the alpha channel
    // TODO: Pointless to have a detail range? -gw
    float detailToggle = env.range < oe_splat_detail_range ? 1.0 : 0.0;
    vec4 sw_detail = detailToggle * oe_splat_getDetailTexel(ri_sw, splat_tc, env);
    vec4 se_detail = detailToggle * oe_splat_getDetailTexel(ri_se, splat_tc, env);
    vec4 ne_detail = detailToggle * oe_splat_getDetailTexel(ri_ne, splat_tc, env);
    vec4 nw_detail = detailToggle * oe_splat_getDetailTexel(ri_nw, splat_tc, env);

    // Combine everything based on weighting:
    texel.rgb =
        sw_weight * mix(sw_primary, sw_detail.rgb, sw_detail.a) +
        se_weight * mix(se_primary, se_detail.rgb, se_detail.a) +
        ne_weight * mix(ne_primary, ne_detail.rgb, ne_detail.a) +
        nw_weight * mix(nw_primary, nw_detail.rgb, nw_detail.a);

    return texel;
}


// Snow splatter. This will whiten the texel based on elevation.
void oe_splat_winter(in vec2 splat_tc, in oe_SplatEnv env, inout vec4 texel)
{
    float snowToggle = env.elevation > oe_splat_snow ? 1.0 : 0.0;
    {
        float snowiness = clamp(max(0.0, env.elevation-oe_splat_snow)/oe_splat_snow, 0.0, 0.8);
        vec4 snow = vec4(1,1,1,1);
        texel.rgb = mix(texel.rgb, snow.rgb, snowiness*snowToggle);
    }
}

// Gets the noise value at the given coordinates.
// TODO: change this to use a texture versus the noise function. Noise is EXPENSIVE
float oe_splat_getNoise(in vec2 tc)
{
    float n = oe_noise_fractal_2d(tc, oe_splat_freq, oe_splat_pers, oe_splat_lac, int(oe_splat_octaves));
    return n;
}

// Scales the incoming tile splat coordinates to match the requested
// LOD level. We offset the level from the current tile key's LOD (.z)
// because otherwise you run into single-precision jitter at high LODs.
vec2 oe_splat_getSplatCoords(float lod)
{
    float dL = oe_tile_key.z - lod;
    float factor = exp2(dL);
    float invFactor = 1.0/factor;
    vec2 scale = vec2(invFactor); 
    vec2 result = oe_layer_tilec.st * scale;

    // For upsampling we need to calculate an offset as well
    float upSampleToggle = factor >= 1.0 ? 1.0 : 0.0;
    {
        vec2 a = floor(oe_tile_key.xy * invFactor);
        vec2 b = a * factor;
        vec2 c = (a+1.0) * factor;
        vec2 offset = (oe_tile_key.xy-b)/(c-b);
        result += upSampleToggle * offset;
    }

    return result;
}

// Main entry point for fragment shader.
void oe_splat_fragment(inout vec4 color)
{
    // Populate the environment:
    float noise = oe_splat_getNoise(oe_splat_covtc); //TODO: use a texture instead; VERY EXPENSIVE
    float elevation = texture2D(oe_terrain_tex, oe_layer_tilec.st).r; //TODO: eliminate if unused
    oe_SplatEnv env = oe_SplatEnv(oe_splat_range, elevation, noise);

    // Mapping of view ranges to splat texture levels of detail.
#define RANGE_COUNT 9
    const float ranges[RANGE_COUNT] = float[](  250.0, 500.0, 1000.0, 4000.0, 30000.0, 150000.0, 300000.0, 1000000.0, 5000000.0 );
    const float lods  [RANGE_COUNT] = float[](  18.0,  17.0,   16.0,   14.0,    12.0,     10.0,      8.0,       6.0,       4.0 );

    // Choose the best range based on distance to camera.
    float d = clamp(oe_splat_range, ranges[0], ranges[RANGE_COUNT-1]);

    vec4 texel;

    // Find the 2 ranges bookending the camera range, and blend between then.
    // TODO: replace bilinear with nearest for far ranges?
    for(int i=0; i<RANGE_COUNT-1; ++i)
    {
        if ( d >= ranges[i] && d <= ranges[i+1] )
        {
            float lod0 = lods[i] + oe_splat_scaleOffsetInt;
            vec2 splat_tc0 = oe_splat_getSplatCoords(lod0);
            vec4 texel0 = oe_splat_bilinear(splat_tc0, env);

            float lod1 = lods[i+1] + oe_splat_scaleOffsetInt;
            vec2 splat_tc1 = oe_splat_getSplatCoords(lod1);
            vec4 texel1 = oe_splat_bilinear(splat_tc1, env);

            float r = (d-ranges[i])/(ranges[i+1]-ranges[i]);
            texel = mix(texel0, texel1, r);

            oe_splat_winter(splat_tc1, env, texel);

            break;
        }
    }

    color = mix(color, texel, oe_splat_intensity*texel.a);

    // uncomment to visualize slope.
    //color.rgba = vec4(oe_splat_slope,0,0,1);
}
