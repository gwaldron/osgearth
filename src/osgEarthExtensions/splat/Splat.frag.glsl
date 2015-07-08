#version 130
#extension GL_EXT_texture_array : enable

#pragma vp_entryPoint "oe_splat_complex"
#pragma vp_location   "fragment_coloring"
#pragma vp_order      "0.4"

// define to activate 'edit' mode in which uniforms control
// the splatting parameters.
#pragma vp_define "SPLAT_EDIT"

// define to activate GPU-generated noise instead of a noise texture.
#pragma vp_define "SPLAT_GPU_NOISE"

// define to activate color image layer mixing.
#pragma vp_define "SPLAT_USE_COLOR_IMAGE"

// include files
#pragma include "Splat.types.glsl"
#pragma include "Splat.frag.common.glsl"

// ref: Splat.getRenderInfo.frag.glsl
oe_SplatRenderInfo oe_splat_getRenderInfo(in float value, in oe_SplatEnv env);


// from the terrain engine:
varying vec4 oe_layer_tilec;
uniform vec4 oe_tile_key;

// from the vertex shader:
varying vec2 oe_splat_covtc;
varying float oe_splat_range;
varying float oe_splat_scaleOffsetInt;

// from SplatTerrainEffect:
uniform float oe_splat_warp;
uniform float oe_splat_blur;
uniform sampler2D oe_splat_coverageTex;
uniform sampler2DArray oe_splatTex;

uniform sampler2D oe_terrain_tex;
uniform mat4 oe_terrain_tex_matrix;
uniform float oe_splat_detailRange;
uniform float oe_splat_noiseScale;
uniform float oe_splat_useBilinear; // 1=true, -1=false

#ifdef SPLAT_USE_COLOR_IMAGE
uniform float oe_splat_color_start_dist;
uniform float oe_splat_color_ratio;
uniform sampler2D oe_color_tex;
uniform mat4 oe_color_tex_mat;

#endif

#ifdef SPLAT_EDIT
uniform float oe_splat_brightness;
uniform float oe_splat_contrast;
uniform float oe_splat_threshold;
uniform float oe_splat_minSlope;
#endif

// Warps the coverage sampling coordinates to mitigate blockiness.
vec2 oe_splat_warpCoverageCoords(in vec2 splat_tc, in oe_SplatEnv env)
{
    vec2 seed = oe_splat_covtc;
    float n1 = 2.0*env.noise.y-1.0;
    vec2 tc = oe_splat_covtc + n1*oe_splat_warp;
    return clamp(tc, 0.0, 1.0);
}

vec4 oe_splat_getTexel(in float index, in vec2 tc)
{
    return texture2DArray(oe_splatTex, vec3(tc, index));
}

// Samples a detail texel using its render info parameters.
// Returns the weighting factor in the alpha channel.
vec4 oe_splat_getDetailTexel(in oe_SplatRenderInfo ri, in vec2 tc, in oe_SplatEnv env)
{
    float hasDetail = ri.detailIndex >= 0.0 ? 1.0 : 0.0;

#ifdef SPLAT_EDIT
    float brightness = oe_splat_brightness;
    float contrast = oe_splat_contrast;
    float threshold = oe_splat_threshold;
    float minSlope = oe_splat_minSlope;
#else
    float brightness = ri.brightness;
    float contrast = ri.contrast;
    float threshold = ri.threshold;
    float minSlope = ri.minSlope;
#endif

    // start with the noise value
    float n = env.noise.x;
	
    // apply slope limiter, then reclamp and threshold:
    float s;
    if ( env.slope >= minSlope )
        s = 1.0;
    else if ( env.slope < 0.1*minSlope )
        s = 0.0;
    else
        s = (env.slope-0.1*minSlope)/(minSlope-0.1*minSlope);

    brightness *= s;

    // apply brightness and contrast, then reclamp
    n = clamp(((n-0.5)*contrast + 0.5) * brightness, 0.0, 1.0);
    
    // apply final threshold:
	n = n < threshold ? 0.0 : n;

    // sample the texel and return it.
    vec4 result = oe_splat_getTexel( max(ri.detailIndex,0), tc);
    return vec4(result.rgb, hasDetail*n);
}

// Generates a texel using nearest-neighbor coverage sampling.
vec4 oe_splat_nearest(in vec2 splat_tc, in oe_SplatEnv env)
{
    vec2 warped_tc = oe_splat_warpCoverageCoords(splat_tc, env);
    float coverageValue = texture2D(oe_splat_coverageTex, warped_tc).r;
    //float coverageValue = 255.0 * texture2D(oe_splat_coverageTex, warped_tc).r;
    oe_SplatRenderInfo ri = oe_splat_getRenderInfo(coverageValue, env);
    vec4 primary = oe_splat_getTexel(ri.primaryIndex, splat_tc);
    float detailToggle = ri.detailIndex >= 0 ? 1.0 : 0.0;
    vec4 detail  = oe_splat_getDetailTexel(ri, splat_tc, env) * detailToggle;    
    return vec4( mix(primary.rgb, detail.rgb, detail.a), 1.0 );
}

// Generates a texel using bilinear filtering on the coverage data.
vec4 oe_splat_bilinear(in vec2 splat_tc, in oe_SplatEnv env)
{
    vec4 texel = vec4(0,0,0,1);

    //TODO: coverage warping is slow due to the noise function. Consider removing/reworking.
    vec2 warped_tc = oe_splat_warpCoverageCoords(splat_tc, env);

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
    float value_sw = texture2D(oe_splat_coverageTex, clamp(sw, 0.0, 1.0)).r;
    float value_se = texture2D(oe_splat_coverageTex, clamp(se, 0.0, 1.0)).r;
    float value_ne = texture2D(oe_splat_coverageTex, clamp(ne, 0.0, 1.0)).r;
    float value_nw = texture2D(oe_splat_coverageTex, clamp(nw, 0.0, 1.0)).r;

    // Build the render info data for each corner:
    oe_SplatRenderInfo ri_sw = oe_splat_getRenderInfo(value_sw, env);
    oe_SplatRenderInfo ri_se = oe_splat_getRenderInfo(value_se, env);
    oe_SplatRenderInfo ri_ne = oe_splat_getRenderInfo(value_ne, env);
    oe_SplatRenderInfo ri_nw = oe_splat_getRenderInfo(value_nw, env);

    // Primary splat:
    vec3 sw_primary = oe_splat_getTexel(ri_sw.primaryIndex, splat_tc).rgb;
    vec3 se_primary = oe_splat_getTexel(ri_se.primaryIndex, splat_tc).rgb;
    vec3 ne_primary = oe_splat_getTexel(ri_ne.primaryIndex, splat_tc).rgb;
    vec3 nw_primary = oe_splat_getTexel(ri_nw.primaryIndex, splat_tc).rgb;

    // Detail splat - weighting is in the alpha channel
    // TODO: Pointless to have a detail range? -gw
    // TODO: If noise is a texture, just try to single-sample it instead
    float detailToggle = env.range < oe_splat_detailRange ? 1.0 : 0.0;
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

#ifdef SPLAT_GPU_NOISE

uniform float oe_splat_freq;
uniform float oe_splat_pers;
uniform float oe_splat_lac;
uniform float oe_splat_octaves;

// see: Splat.Noise.glsl
float oe_noise_fractal4D(in vec2 seed, in float frequency, in float persistence, in float lacunarity, in int octaves);

vec4 oe_splat_getNoise(in vec2 tc)
{
    return vec4(oe_noise_fractal4D(tc, oe_splat_freq, oe_splat_pers, oe_splat_lac, int(oe_splat_octaves)));
}

#else // !SPLAT_GPU_NOISE

uniform sampler2D oe_splat_noiseTex;
vec4 oe_splat_getNoise(in vec2 tc)
{
    return texture2D(oe_splat_noiseTex, tc.st);
}

#endif // SPLAT_GPU_NOISE


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
    if ( factor >= 1.0 )
    {
        vec2 a = floor(oe_tile_key.xy * invFactor);
        vec2 b = a * factor;
        vec2 c = (a+1.0) * factor;
        vec2 offset = (oe_tile_key.xy-b)/(c-b);
        result += offset;
    }

    return result;
}

// Simplified entry point with does no filtering or range blending. (much faster.)
void oe_splat_simple(inout vec4 color)
{
    float noiseLOD = floor(oe_splat_noiseScale);
    vec2 noiseCoords = oe_splat_getSplatCoords(noiseLOD);

    oe_SplatEnv env;
    env.range = oe_splat_range;
    env.slope = oe_splat_getSlope();
    env.noise = oe_splat_getNoise(noiseCoords);
    env.elevation = 0.0;

    float lod = 12.0;
    vec2 splatCoords = oe_splat_getSplatCoords(lod);
    color = oe_splat_nearest(splatCoords, env);
}


// Main entry point for fragment shader.
void oe_splat_complex(inout vec4 color)
{
    // Noise coords.
    float noiseLOD = floor(oe_splat_noiseScale);
    vec2 noiseCoords = oe_splat_getSplatCoords(noiseLOD); //12

    oe_SplatEnv env;
    env.range = oe_splat_range;
    env.slope = oe_splat_getSlope();
    env.noise = oe_splat_getNoise(noiseCoords);
    env.elevation = 0.0; // usused atm. //texture2D(oe_terrain_tex, (oe_terrain_tex_matrix*oe_layer_tilec).st).r;

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
            vec4 texel0 = oe_splat_useBilinear > 0.0?
                oe_splat_bilinear(splat_tc0, env) :
                oe_splat_nearest(splat_tc0, env);
            

            float lod1 = lods[i+1] + oe_splat_scaleOffsetInt;
            vec2 splat_tc1 = oe_splat_getSplatCoords(lod1);
            vec4 texel1 = oe_splat_useBilinear > 0.0?
                oe_splat_bilinear(splat_tc1, env) :
                oe_splat_nearest(splat_tc1, env);

            float r = (d-ranges[i])/(ranges[i+1]-ranges[i]);
            texel = mix(texel0, texel1, r);

            break;
        }
    }

    color = mix(color, texel, texel.a);

#ifdef SPLAT_USE_COLOR_IMAGE
	vec3 groundColor = texture(oe_color_tex, (oe_color_tex_mat*oe_layer_tilec).st).rgb;
	float fade_dist = oe_splat_color_start_dist/2.0 + 0.1;
	float fade = clamp(oe_splat_range, oe_splat_color_start_dist, oe_splat_color_start_dist + fade_dist);
	fade = (fade-oe_splat_color_start_dist)/fade_dist;
	color.rgb = mix(color.rgb, color.rgb*(2.0*groundColor), oe_splat_color_ratio*fade);
#endif

    // uncomment to visualize slope.
    //color.rgba = vec4(env.slope,0,0,1);
}
