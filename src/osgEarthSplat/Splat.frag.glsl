#if(__VERSION__ < 400)
    #extension GL_ARB_gpu_shader5 : enable  // textureGather
#endif

#pragma vp_entryPoint oe_splat_sampleCoverage
#pragma vp_location   fragment_coloring
#pragma vp_order      0.2

#pragma import_defines(OE_LANDCOVER_TEX);

uniform sampler2D OE_LANDCOVER_TEX;

//uniform sampler2D oe_splat_coverageTex;
in vec2 oe_splat_covtc;
flat in float oe_splat_coverageTexSize;

// stage global: set this and read it in the splatter.
vec4 oe_LandCover_coverage;

// read the comment below regarding textureGather
//#define USE_TEXTURE_GATHER

void oe_splat_sampleCoverage(inout vec4 unused)
{
#ifdef USE_TEXTURE_GATHER
    // A wee bit faster, but causes a rendering anomaly
    oe_splat_coverage = textureGather(oe_tile_landCoverTex, oe_splat_covtc, 0).wzxy;
#else

    float pixelWidth = 1.0/oe_splat_coverageTexSize;
    float halfPixelWidth = pixelWidth * 0.5;

    // Find the four quantized coverage coordinates that form a box around the actual
    // coverage coordinates, where each quantized coord is at the center of a coverage texel.
    vec2 rem = mod(oe_splat_covtc, pixelWidth);
    vec2 sw;
    sw.x = oe_splat_covtc.x - rem.x + (rem.x >= halfPixelWidth ? halfPixelWidth : -halfPixelWidth);
    sw.y = oe_splat_covtc.y - rem.y + (rem.y >= halfPixelWidth ? halfPixelWidth : -halfPixelWidth); 
    vec2 ne = sw + pixelWidth;
    vec2 nw = vec2(sw.x, ne.y);
    vec2 se = vec2(ne.x, sw.y);

    oe_LandCover_coverage = vec4(
        texture(OE_LANDCOVER_TEX, sw).r,
        texture(OE_LANDCOVER_TEX, se).r,
        texture(OE_LANDCOVER_TEX, nw).r,
        texture(OE_LANDCOVER_TEX, ne).r );

    //return vec4(
    //    texture(oe_splat_coverageTex, clamp(sw, 0.0, 1.0)).r,
    //    texture(oe_splat_coverageTex, clamp(se, 0.0, 1.0)).r,
    //    texture(oe_splat_coverageTex, clamp(nw, 0.0, 1.0)).r,
    //    texture(oe_splat_coverageTex, clamp(ne, 0.0, 1.0)).r );
#endif
}



[break]

// order of 1.1 allows user the opportunity to override oe_splat_coverage stage global
#pragma vp_entryPoint oe_splat_complex
#pragma vp_location   fragment_coloring
#pragma vp_order      1.1

// include files
#pragma include Splat.types.glsl

// statset defines
#pragma import_defines(OE_SPLAT_NOISE_SAMPLER)
#pragma import_defines(OE_SPLAT_EDIT_MODE)
#pragma import_defines(OE_SPLAT_GPU_NOISE)
#pragma import_defines(OE_TERRAIN_RENDER_NORMAL_MAP)
#pragma import_defines(OE_TERRAIN_BLEND_IMAGERY)
#pragma import_defines(OE_SPLAT_USE_MATERIALS)

// from: Splat.util.glsl
void oe_splat_getLodBlend(in float range, out float lod0, out float rangeOuter, out float rangeInner, out float clampedRange);

// from the terrain engine:
in vec4 oe_layer_tilec;                     // unit tile coords

// from the vertex shader:
in vec2 oe_splat_covtc;                     // coverage texture coords
in float oe_splat_range;                    // distance from camera to vertex
flat in float oe_splat_coverageTexSize;     // size of coverage texture

in float oe_layer_opacity;

// stage global: coverage quad-value set in oe_splat_sampleCoverage
vec4 oe_LandCover_coverage;

// from SplatLayerFactory:
uniform sampler2DArray oe_splatTex;
uniform int oe_splat_scaleOffsetInt;

uniform float oe_splat_detailRange;
uniform float oe_splat_noiseScale;

uniform vec4 oe_tile_key_u;

#ifdef OE_SPLAT_EDIT_MODE
uniform float oe_splat_brightness;
uniform float oe_splat_contrast;
uniform float oe_splat_threshold;
uniform float oe_splat_minSlope;
#endif

// lookup table containing the coverage value => texture index mappings
uniform samplerBuffer oe_splat_coverageLUT;

uniform int oe_layer_order;

//............................................................................
// Get the slope of the terrain

#ifdef OE_TERRAIN_RENDER_NORMAL_MAP
// import SDK
vec4 oe_terrain_getNormalAndCurvature(in vec2);

// normal map version:
in vec2 oe_normalMapCoords;

float oe_splat_getSlope()
{
    vec4 encodedNormal = oe_terrain_getNormalAndCurvature( oe_normalMapCoords );
    vec3 normalTangent = normalize(encodedNormal.xyz*2.0-1.0);
    return clamp((1.0-normalTangent.z)/0.8, 0.0, 1.0);
}

#else // !OE_TERRAIN_RENDER_NORMAL_MAP

// non- normal map version:
in float oe_splat_slope;

float oe_splat_getSlope()
{
    return oe_splat_slope;
}

#endif // OE_TERRAIN_RENDER_NORMAL_MAP


//............................................................................
// reads the encoded splatting render information for a coverage value.
// this data was encoded in Surface::createLUTBUffer().

void oe_splat_getRenderInfo(in float value, in oe_SplatEnv env, out oe_SplatRenderInfo ri)
{
    const int num_lods = 26;

    int lutIndex = int(value)*num_lods + int(env.lod);

    // fetch the splatting parameters:
    vec4 t = texelFetch(oe_splat_coverageLUT, lutIndex);

    ri.primaryIndex = float((int(t[0])>>8)-1);
    ri.detailIndex  = float((int(t[0])&0xFF)-1);

#ifdef OE_SPLAT_USE_MATERIALS
    ri.materialIndex = float(int(t[1])-1);
#endif

    // brightness and contrast are packed into one float:
    ri.brightness   = trunc(t[2])/100.0;
    ri.contrast     = fract(t[2])*10.0;

    // threshold and slope are packed into one float:
    ri.threshold    = trunc(t[3])/100.0;
    ri.minSlope     = fract(t[3])*10.0;
}


//............................................................................
// Sample a texel from the splatting texture catalog

void oe_splat_getTexel(in float index, in vec2 tc, out vec4 texel)
{
    texel = index >= 0.0 ? texture(oe_splatTex, vec3(tc, index)) : vec4(1,0,0,0);
}

#ifdef OE_SPLAT_USE_MATERIALS
void oe_splat_getMaterial(in float index, in vec2 tc, out vec4 material)
{
    material = index >= 0.0 ? texture(oe_splatTex, vec3(tc, index)) : vec4(.5,.5,1,0);
}
#endif

//............................................................................
// Samples a detail texel using its render info parameters.
// Returns the weighting factor in the alpha channel.

vec4 oe_splat_getDetailTexel(in oe_SplatRenderInfo ri, in vec2 tc, in oe_SplatEnv env)
{
    float hasDetail = clamp(ri.detailIndex+1.0, 0.0, 1.0);

#ifdef OE_SPLAT_EDIT_MODE
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
    vec4 result;
    oe_splat_getTexel(ri.detailIndex, tc, result);
    return vec4(result.rgb, hasDetail*n);
}

//............................................................................
// Generates a texel using nearest-neighbor coverage sampling.

vec4 oe_splat_nearest(in vec2 splat_tc, in oe_SplatEnv env)
{
    float coverageValue = oe_LandCover_coverage[0];
    oe_SplatRenderInfo ri;
    oe_splat_getRenderInfo(coverageValue, env, ri);
    vec4 primary;
    oe_splat_getTexel(ri.primaryIndex, splat_tc, primary);
    float detailToggle = ri.detailIndex >= 0 ? 1.0 : 0.0;
    vec4 detail  = oe_splat_getDetailTexel(ri, splat_tc, env) * detailToggle;    
    return vec4( mix(primary.rgb, detail.rgb, detail.a), primary.a );
}

//............................................................................
// Generates a texel using bilinear filtering on the coverage data.

void oe_splat_bilinear(in vec2 splat_tc, in oe_SplatEnv env, out vec4 color, out vec4 material)
{
    // Build the render info data for each corner:
    oe_SplatRenderInfo ri[4]; //sw,se,nw,ne
    oe_splat_getRenderInfo(oe_LandCover_coverage[0], env, ri[0]);
    oe_splat_getRenderInfo(oe_LandCover_coverage[1], env, ri[1]);
    oe_splat_getRenderInfo(oe_LandCover_coverage[2], env, ri[2]);
    oe_splat_getRenderInfo(oe_LandCover_coverage[3], env, ri[3]);

    // Primary splat:
    vec4 p[4];
    oe_splat_getTexel(ri[0].primaryIndex, splat_tc, p[0]);
    oe_splat_getTexel(ri[1].primaryIndex, splat_tc, p[1]);
    oe_splat_getTexel(ri[2].primaryIndex, splat_tc, p[2]);
    oe_splat_getTexel(ri[3].primaryIndex, splat_tc, p[3]);

#ifdef OE_SPLAT_USE_MATERIALS
    // Material splat:
    vec4 m[4];
    oe_splat_getTexel(ri[0].materialIndex, splat_tc, m[0]);
    oe_splat_getTexel(ri[1].materialIndex, splat_tc, m[1]);
    oe_splat_getTexel(ri[2].materialIndex, splat_tc, m[2]);
    oe_splat_getTexel(ri[3].materialIndex, splat_tc, m[3]);
#endif

    // Detail splat - weighting is in the alpha channel
    // TODO: Pointless to have a detail range? -gw
    // TODO: If noise is a texture, just try to single-sample it instead
    float dTog = env.range < oe_splat_detailRange ? 1.0 : 0.0;
    vec4 d[4];
    d[0] = dTog * oe_splat_getDetailTexel(ri[0], splat_tc, env);
    d[1] = dTog * oe_splat_getDetailTexel(ri[1], splat_tc, env);
    d[2] = dTog * oe_splat_getDetailTexel(ri[2], splat_tc, env);
    d[3] = dTog * oe_splat_getDetailTexel(ri[3], splat_tc, env); 

    vec4 b[4];
    b[0] = vec4(mix(p[0].rgb, d[0].rgb, d[0].a), p[0].a);
    b[1] = vec4(mix(p[1].rgb, d[1].rgb, d[1].a), p[1].a);
    b[2] = vec4(mix(p[2].rgb, d[2].rgb, d[2].a), p[2].a);
    b[3] = vec4(mix(p[3].rgb, d[3].rgb, d[3].a), p[3].a);

    vec2 weight = fract(oe_splat_covtc*oe_splat_coverageTexSize - 0.5);

    vec4 south = mix(b[0], b[1], weight.x);
    vec4 north = mix(b[2], b[3], weight.x);

    color = mix(south, north, weight.y);

#ifdef OE_SPLAT_USE_MATERIALS
    south = mix(m[0], m[1], weight.x);
    north = mix(m[2], m[3], weight.x);
    material = mix(south, north, weight.y);
#endif
}

//............................................................................

#ifdef OE_SPLAT_GPU_NOISE

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

#ifdef OE_SPLAT_NOISE_SAMPLER
uniform sampler2D OE_SPLAT_NOISE_SAMPLER;
vec4 oe_splat_getNoise(in vec2 tc)
{
    return texture(OE_SPLAT_NOISE_SAMPLER, tc.st);
}
#else
vec4 oe_splat_getNoise(in vec2 tc)
{
    return vec4(0.0);
}
#endif

#endif // SPLAT_GPU_NOISE


// cannot use the SDK version because it is VS only
vec2 oe_scaleToRefLOD(in vec2 tc, in float refLOD)
{
    float dL = oe_tile_key_u.z - refLOD;
    float factor = exp2(dL);
    float invFactor = 1.0 / factor;
    vec2 result = tc * vec2(invFactor);

    vec2 a = floor(oe_tile_key_u.xy * invFactor);
    vec2 b = a * factor;
    vec2 c = b + factor;

    float m = floor(clamp(factor, 0.0, 1.0)); // if factor>=1.0
    result += m * (oe_tile_key_u.xy - b) / (c - b);

    return result;
}

//............................................................................
// Simplified entry point with does no filtering or range blending. (much faster.)

void oe_splat_simple(inout vec4 color)
{
    float noiseLOD = floor(oe_splat_noiseScale);
    vec2 noiseCoords = oe_scaleToRefLOD(oe_layer_tilec.st, noiseLOD);

    oe_SplatEnv env;
    env.range = oe_splat_range;
    env.slope = oe_splat_getSlope();
    env.noise = oe_splat_getNoise(noiseCoords);
    env.elevation = 0.0;
    
    float lod0;
    float rangeOuter, rangeInner;
    oe_splat_getLodBlend(oe_splat_range, lod0, rangeOuter, rangeInner, env.range);
    vec2 tc = oe_scaleToRefLOD(oe_layer_tilec.st, lod0 + float(oe_splat_scaleOffsetInt));

#ifdef OE_SPLAT_USE_MATERIALS
    vec4 material;
    oe_splat_bilinear(tc, env, color, material);
#endif

    color.a *= oe_layer_opacity;
}

//............................................................................
// Main entry point for fragment shader.

// stage inputs
in vec3 vp_Normal;

// stage global
mat3 oe_normalMapTBN;


void oe_splat_complex(inout vec4 color)
{
    // Noise coords.
    float noiseLOD = floor(oe_splat_noiseScale);
    vec2 noiseCoords = oe_scaleToRefLOD(oe_layer_tilec.st, noiseLOD);

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
    vec4 texel0, texel1;
    vec4 material0, material1;

    vec2 tc0 = oe_scaleToRefLOD(oe_layer_tilec.st, lod0 + scaleOffset);
    env.lod = lod0;
    oe_splat_bilinear(tc0, env, texel0, material0);
    
    vec2 tc1 = oe_scaleToRefLOD(oe_layer_tilec.st, lod0 + 1.0 + scaleOffset);
    env.lod = lod0+1.0;
    oe_splat_bilinear(tc1, env, texel1, material1);

    // recalcluate blending ratio
    float lodBlend = clamp((rangeOuter - env.range) / (rangeOuter - rangeInner), 0, 1);
       
    // Blend the two samples based on LOD factor:
    vec4 texel = mix(texel0, texel1, lodBlend);

#ifdef OE_SPLAT_USE_MATERIALS
    // Blend together the material samples:
    vec4 material = mix(material0, material1, lodBlend);
    vec3 n = oe_normalMapTBN * (material.xyz*2.0-1.0);
    vp_Normal = normalize(vp_Normal + n);
#endif

    // incorporate the layer's opacity:
    texel.a *= oe_layer_opacity;

#ifdef OE_TERRAIN_BLEND_IMAGERY
    // If this is a first image layer, blend with the incoming terrain color.
    // Otherwise, apply directly and let GL blending do the rest.
    if (oe_layer_order == 0)
    {
        color.rgb = texel.rgb*texel.a + color.rgb*(1.0-texel.a);
    }
    else
    {
        color = texel;
    }
#else
    // No blending? The output is just the texel value.
    color = texel;
#endif // OE_TERRAIN_BLEND_IMAGERY
}
