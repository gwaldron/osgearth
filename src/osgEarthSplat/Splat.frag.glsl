#version $GLSL_VERSION_STR

#if(__VERSION__ < 400)
#extension GL_ARB_gpu_shader5 : enable      // textureGather
#endif

#pragma vp_entryPoint oe_splat_complex
#pragma vp_location   fragment_coloring
#pragma vp_order      0.4

// define to activate 'edit' mode in which uniforms control
// the splatting parameters.
#pragma vp_define SPLAT_EDIT

// define to activate GPU-generated noise instead of a noise texture.
#pragma vp_define SPLAT_GPU_NOISE

// include files
#pragma include Splat.types.glsl
#pragma include Splat.frag.common.glsl

// from: Splat.util.glsl
void oe_splat_getLodBlend(in float range, out float lod0, out float rangeOuter, out float rangeInner, out float clampedRange);

// from terrain SDK:
vec2 oe_terrain_scaleCoordsToRefLOD(in vec2 tc, in float refLOD);

// from the terrain engine:
in vec4 oe_layer_tilec;                     // unit tile coords

// from the vertex shader:
in vec2 oe_splat_covtc;                     // coverage texture coords
in float oe_splat_range;                    // distance from camera to vertex
flat in float oe_splat_coverageTexSize;     // size of coverage texture

// from SplatTerrainEffect:
uniform sampler2D oe_splat_coverageTex;
uniform sampler2DArray oe_splatTex;
uniform int oe_splat_scaleOffsetInt;

uniform float oe_splat_detailRange;
uniform float oe_splat_noiseScale;

#ifdef SPLAT_EDIT
uniform float oe_splat_brightness;
uniform float oe_splat_contrast;
uniform float oe_splat_threshold;
uniform float oe_splat_minSlope;
#endif

// lookup table containing the coverage value => texture index mappings
uniform samplerBuffer oe_splat_coverageLUT;


// reads the encoded splatting render information for a coverage value.
// this data was encoded in Surface::createLUTBUffer().
void oe_splat_getRenderInfo(in float value, in oe_SplatEnv env, out oe_SplatRenderInfo ri)
{
    const int num_lods = 26;
    const float inv255 = 0.00392156862;

    int index = int(value)*num_lods + int(env.lod);

    // fetch the splatting parameters:
    vec4 t = texelFetch(oe_splat_coverageLUT, index);

    ri.primaryIndex = t[0];
    ri.detailIndex  = t[1];

    // brightness and contrast are packed into one float:
    ri.brightness   = trunc(t[2])/100.0;
    ri.contrast     = fract(t[2])*100.0;

    // threshold and slope are packed into one float:
    ri.threshold    = trunc(t[3])/100.0;
    ri.minSlope     = fract(t[3])*100.0;
}

vec4 oe_splat_getTexel(in float index, in vec2 tc)
{
    return texture(oe_splatTex, vec3(tc, index));
}

// Samples a detail texel using its render info parameters.
// Returns the weighting factor in the alpha channel.
vec4 oe_splat_getDetailTexel(in oe_SplatRenderInfo ri, in vec2 tc, in oe_SplatEnv env)
{
    float hasDetail = clamp(ri.detailIndex+1.0, 0.0, 1.0);

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
    vec4 result = oe_splat_getTexel(ri.detailIndex, tc);
    return vec4(result.rgb, hasDetail*n);
}

// Generates a texel using nearest-neighbor coverage sampling.
vec4 oe_splat_nearest(in vec2 splat_tc, inout oe_SplatEnv env)
{
    float coverageValue = texture(oe_splat_coverageTex, oe_splat_covtc).r;
    oe_SplatRenderInfo ri;
    oe_splat_getRenderInfo(coverageValue, env, ri);
    vec4 primary = oe_splat_getTexel(ri.primaryIndex, splat_tc);
    float detailToggle = ri.detailIndex >= 0 ? 1.0 : 0.0;
    vec4 detail  = oe_splat_getDetailTexel(ri, splat_tc, env) * detailToggle;    
    return vec4( mix(primary.rgb, detail.rgb, detail.a), 1.0 );
}

// Generates a texel using bilinear filtering on the coverage data.
vec4 oe_splat_bilinear(in vec2 splat_tc, inout oe_SplatEnv env)
{
    vec4 texel = vec4(0,0,0,1);

    float size = oe_splat_coverageTexSize;

    vec4 value = textureGather(oe_splat_coverageTex, oe_splat_covtc, 0);
    float value_sw = value.w;
    float value_se = value.z;
    float value_ne = value.y;
    float value_nw = value.x;

    // Build the render info data for each corner:
    oe_SplatRenderInfo ri_sw; oe_splat_getRenderInfo(value_sw, env, ri_sw);
    oe_SplatRenderInfo ri_se; oe_splat_getRenderInfo(value_se, env, ri_se);
    oe_SplatRenderInfo ri_ne; oe_splat_getRenderInfo(value_ne, env, ri_ne);
    oe_SplatRenderInfo ri_nw; oe_splat_getRenderInfo(value_nw, env, ri_nw);

    // Primary splat:
    vec3 sw_primary = oe_splat_getTexel(ri_sw.primaryIndex, splat_tc).rgb;
    vec3 se_primary = oe_splat_getTexel(ri_se.primaryIndex, splat_tc).rgb;
    vec3 ne_primary = oe_splat_getTexel(ri_ne.primaryIndex, splat_tc).rgb;
    vec3 nw_primary = oe_splat_getTexel(ri_nw.primaryIndex, splat_tc).rgb;

    // Detail splat - weighting is in the alpha channel
    // TODO: Pointless to have a detail range? -gw
    // TODO: If noise is a texture, just try to single-sample it instead
    float detailToggle =env.range < oe_splat_detailRange ? 1.0 : 0.0;
    vec4 sw_detail = detailToggle * oe_splat_getDetailTexel(ri_sw, splat_tc, env);
    vec4 se_detail = detailToggle * oe_splat_getDetailTexel(ri_se, splat_tc, env);
    vec4 ne_detail = detailToggle * oe_splat_getDetailTexel(ri_ne, splat_tc, env);
    vec4 nw_detail = detailToggle * oe_splat_getDetailTexel(ri_nw, splat_tc, env);   

#if 0
    // Combine everything based on weighting:
    texel.rgb =
        sw_weight * mix(sw_primary, sw_detail.rgb, sw_detail.a) +
        se_weight * mix(se_primary, se_detail.rgb, se_detail.a) +
        ne_weight * mix(ne_primary, ne_detail.rgb, ne_detail.a) +
        nw_weight * mix(nw_primary, nw_detail.rgb, nw_detail.a);

#else

    vec3 nw_mix = mix(nw_primary, nw_detail.rgb, nw_detail.a);
    vec3 ne_mix = mix(ne_primary, ne_detail.rgb, ne_detail.a);
    vec3 sw_mix = mix(sw_primary, sw_detail.rgb, sw_detail.a);
    vec3 se_mix = mix(se_primary, se_detail.rgb, se_detail.a);

    //float cellSize = 1.0/size;
    //vec2 g1 = fract(oe_splat_covtc*size); //(size-1.0));
    //vec2 g2 = fract(g1-0.5+pixelWidth);
    vec2 weight = fract( oe_splat_covtc*size - 0.5+(1.0/size) ); //cellSize);

    vec3 temp0 = mix(nw_mix, ne_mix, weight.x);
    vec3 temp1 = mix(sw_mix, se_mix, weight.x);

    texel.rgb = mix(temp1, temp0, weight.y);

#endif

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
    return texture(oe_splat_noiseTex, tc.st);
}

#endif // SPLAT_GPU_NOISE

// Simplified entry point with does no filtering or range blending. (much faster.)
void oe_splat_simple(inout vec4 color)
{
    float noiseLOD = floor(oe_splat_noiseScale);
    vec2 noiseCoords = oe_terrain_scaleCoordsToRefLOD(oe_layer_tilec.st, noiseLOD);

    oe_SplatEnv env;
    env.range = oe_splat_range;
    env.slope = oe_splat_getSlope();
    env.noise = oe_splat_getNoise(noiseCoords);
    env.elevation = 0.0;
    
    float lod0;
    float rangeOuter, rangeInner;
    oe_splat_getLodBlend(oe_splat_range, lod0, rangeOuter, rangeInner, env.range);
    vec2 tc = oe_terrain_scaleCoordsToRefLOD(oe_layer_tilec.st, lod0 + float(oe_splat_scaleOffsetInt));

    color = oe_splat_bilinear(tc, env);

    //color = mix(color, vec4(tc.s, tc.t, 0.0, 1.0), 0.5);
}

// Main entry point for fragment shader.
void oe_splat_complex(inout vec4 color)
{
    // Noise coords.
    float noiseLOD = floor(oe_splat_noiseScale);
    vec2 noiseCoords = oe_terrain_scaleCoordsToRefLOD(oe_layer_tilec.st, noiseLOD); //TODO: move to VS for slight speedup

    oe_SplatEnv env;
    env.range = oe_splat_range;
    env.slope = oe_splat_getSlope();
    env.noise = oe_splat_getNoise(noiseCoords);
    env.elevation = 0.0;

    // quantize the scale offset so we take the hit in the FS
    float scaleOffset = float(oe_splat_scaleOffsetInt);
        
    // Calculate the 2 LODs we need to blend. We have to do this in the FS because 
    // it's quite possible for a single triangle to span more than 2 LODs.
    float lod0, lod1;
    float rangeOuter, rangeInner;
    oe_splat_getLodBlend(oe_splat_range, lod0, rangeOuter, rangeInner, env.range);
    
    // Sample the two LODs:
    vec2 tc0 = oe_terrain_scaleCoordsToRefLOD(oe_layer_tilec.st, lod0 + scaleOffset);
    env.lod = lod0;
    vec4 texel0 = oe_splat_bilinear(tc0, env);
    
    vec2 tc1 = oe_terrain_scaleCoordsToRefLOD(oe_layer_tilec.st, lod0 + 1.0 + scaleOffset);
    env.lod = lod0+1.0;
    vec4 texel1 = oe_splat_bilinear(tc1, env);

    // recalcluate blending ratio
    float lodBlend = clamp((rangeOuter - env.range) / (rangeOuter - rangeInner), 0, 1);
       
    // Blend:
    vec4 texel = mix(texel0, texel1, lodBlend);

    color = mix(color, texel, texel.a);
    color.a = 1.0;

    // uncomment to visualize slope.
    //color.rgba = vec4(env.slope,0,0,1);
}
