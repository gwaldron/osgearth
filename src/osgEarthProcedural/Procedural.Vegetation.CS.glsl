#version 430
layout(local_size_x=1, local_size_y=1, local_size_z=1) in;

#pragma include Procedural.Vegetation.Types.glsl

uniform sampler2D oe_veg_noiseTex;
#define NOISE_SMOOTH   0
#define NOISE_RANDOM   1
#define NOISE_RANDOM_2 2
#define NOISE_CLUMPY   3

// (LLx, LLy, URx, URy, tileNum)
uniform float oe_tile[5];

uniform vec2 oe_tile_elevTexelCoeff;
uniform sampler2D oe_tile_elevationTex;
uniform mat4 oe_tile_elevationTexMatrix;
uniform float oe_veg_colorMinSaturation;

#pragma import_defines(OE_LANDCOVER_TEX)
#pragma import_defines(OE_LANDCOVER_TEX_MATRIX)
uniform sampler2D OE_LANDCOVER_TEX;
uniform mat4 OE_LANDCOVER_TEX_MATRIX;

#pragma import_defines(OE_GROUNDCOVER_COLOR_SAMPLER)
#pragma import_defines(OE_GROUNDCOVER_COLOR_MATRIX)
#ifdef OE_GROUNDCOVER_COLOR_SAMPLER
uniform sampler2D OE_GROUNDCOVER_COLOR_SAMPLER ;
uniform mat4 OE_GROUNDCOVER_COLOR_MATRIX ;
#endif

#pragma import_defines(OE_GROUNDCOVER_PICK_NOISE_TYPE)
#ifdef OE_GROUNDCOVER_PICK_NOISE_TYPE
int pickNoiseType = OE_GROUNDCOVER_PICK_NOISE_TYPE ;
#else
//int pickNoiseType = NOISE_RANDOM;
int pickNoiseType = NOISE_CLUMPY;
#endif

#pragma import_defines(OE_LIFEMAP_SAMPLER)
#pragma import_defines(OE_LIFEMAP_MATRIX)
#ifdef OE_LIFEMAP_SAMPLER
uniform sampler2D OE_LIFEMAP_SAMPLER ;
uniform mat4 OE_LIFEMAP_MATRIX ;
#endif

#pragma import_defines(OE_BIOME_SAMPLER)
#pragma import_defines(OE_BIOME_MATRIX)
#ifdef OE_BIOME_SAMPLER
uniform sampler2D OE_BIOME_SAMPLER;
uniform mat4 OE_BIOME_MATRIX;
#endif

#ifdef OE_GROUNDCOVER_COLOR_SAMPLER
// https://stackoverflow.com/a/17897228/4218920
vec3 rgb2hsv(vec3 c)
{
    const vec4 K = vec4(0.0, -1.0 / 3.0, 2.0 / 3.0, -1.0);
    vec4 p = mix(vec4(c.bg, K.wz), vec4(c.gb, K.xy), step(c.b, c.g));
    vec4 q = mix(vec4(p.xyw, c.r), vec4(c.r, p.yzx), step(p.x, c.r));
    float d = q.x - min(q.w, q.y);
    const float e = 1.0e-10;
    return vec3(abs(q.z + (q.w - q.y) / (6.0 * d + e)), d / (q.x + e), q.x);
}

bool isLegalColor(in vec2 tilec)
{
    vec4 c = texture(OE_GROUNDCOVER_COLOR_SAMPLER, (OE_GROUNDCOVER_COLOR_MATRIX*vec4(tilec,0,1)).st);
    vec3 hsv = rgb2hsv(c.rgb);
    return hsv[1] > oe_GroundCover_colorMinSaturation;
}
#endif // OE_GROUNDCOVER_COLOR_SAMPLER

float getElevation(in vec2 tilec) {
    vec2 elevc = tilec
        * oe_tile_elevTexelCoeff.x * oe_tile_elevationTexMatrix[0][0] // scale
        + oe_tile_elevTexelCoeff.x * oe_tile_elevationTexMatrix[3].st // bias
        + oe_tile_elevTexelCoeff.y;
    return texture(oe_tile_elevationTex, elevc).r;
}

uniform float dense_power = 1.0;
uniform float lush_power = 1.0;
uniform float rugged_power = 1.0;

#define RUGGED 0
#define DENSE 1
#define LUSH 2

void generate()
{
    const uint x = gl_GlobalInvocationID.x;
    const uint y = gl_GlobalInvocationID.y;

    int tileNum = int(oe_tile[4]);

    uint local_i = 
        y * gl_NumWorkGroups.x + x;

    uint i = 
        tileNum * (gl_NumWorkGroups.x * gl_NumWorkGroups.y)
        + local_i;

    // tileNum -1 = slot unoccupied
    instance[i].tileNum = -1;

    vec2 offset = vec2(float(x), float(y));
    vec2 halfSpacing = 0.5 / vec2(gl_NumWorkGroups.xy);
    vec2 tilec = halfSpacing + offset / vec2(gl_NumWorkGroups.xy);

    vec4 noise = textureLod(oe_veg_noiseTex, tilec, 0);

    vec2 shift = vec2(fract(noise[1]*1.5), fract(noise[2]*1.5))*2.0-1.0;
    tilec += shift * halfSpacing;

    vec4 tilec4 = vec4(tilec, 0, 1);

#ifdef OE_GROUNDCOVER_COLOR_SAMPLER
    if (!isLegalColor(tilec))
        return;
#endif

    float fill;
    float lush;

    vec2 biome_uv = (OE_BIOME_MATRIX*tilec4).st;
    ivec2 biome_xy = ivec2(biome_uv * 255.0);
    int biome_index = int(texelFetch(OE_BIOME_SAMPLER, biome_xy, 0).r); // *255.0);
    Biome biome = biomes[biome_index];

    if (biome.offset < 0) // undefined biome
        return;

    vec2 lifemap_uv = (OE_LIFEMAP_MATRIX*tilec4).st;
    vec3 lifemap = texture(OE_LIFEMAP_SAMPLER, lifemap_uv).xyz;
    fill = lifemap[DENSE] * dense_power;
    lush = lifemap[LUSH] * lush_power;
    lush = noise[pickNoiseType] * lush;

    // select an asset at random
    int pickIndex = clamp(int(floor(lush * float(biome.count))), 0, biome.count - 1);
    int assetIndex = biome.offset + pickIndex;

    // Recover the asset we randomly picked:
    Asset asset = assets[assetIndex];

    fill *= asset.fill;

    // discard instances based on noise value threshold (coverage). If it passes,
    // scale the noise value back up to [0..1]
    if (noise[NOISE_SMOOTH] > fill)
        return;
    else
        noise[NOISE_SMOOTH] /= fill;

    // It's a keeper - record it to the instance buffer.
    instance[i].tileNum = tileNum; // need this for merge

    vec2 LL = vec2(oe_tile[0], oe_tile[1]);
    vec2 UR = vec2(oe_tile[2], oe_tile[3]);

    vec4 vertex_model = vec4(mix(LL, UR, tilec), getElevation(tilec), 1.0);

    instance[i].vertex = vertex_model;
    instance[i].tilec = tilec;

    instance[i].fillEdge = 1.0;
    const float xx = 0.5;
    if (noise[NOISE_SMOOTH] > xx)
        instance[i].fillEdge = 1.0-((noise[NOISE_SMOOTH]-xx)/(1.0-xx));

    instance[i].modelCommand = asset.modelCommand;
    instance[i].billboardCommand = asset.billboardCommand;

    //a pseudo-random scale factor to the width and height of a billboard
    instance[i].sizeScale = 1.0 + asset.sizeVariation * (noise[NOISE_RANDOM_2]*2.0-1.0);
    instance[i].width = asset.width * instance[i].sizeScale;
    instance[i].height = asset.height * instance[i].sizeScale;

    float rotation = 6.283185 * fract(noise[NOISE_RANDOM_2]*5.5);
    instance[i].sinrot = sin(rotation);
    instance[i].cosrot = cos(rotation);
}


// Consolidates all the instances made by generate()
void merge()
{
    uint i = gl_GlobalInvocationID.x;

    // only if instance is set and tile is active:
    int tileNum = instance[i].tileNum;
    if (tileNum >= 0 && tile[tileNum].inUse == 1)
    {
        uint k = atomicAdd(di.num_groups_x, 1);
        cullSet[k] = i;
    }
}


uniform vec3 oe_Camera;
uniform int oe_ic_numCommands; // total number of draw commands
uniform float oe_veg_sse = 100.0; // pixels
uniform float oe_veg_maxRange;

#define MASK_BILLBOARD 1
#define MASK_MODEL 2

void cull()
{
    uint i = cullSet[ gl_GlobalInvocationID.x ];

    // initialize the drawMask to 0, meaning no render leaf for this instance (yet)
    instance[i].drawMask = 0;

    // Which tile did this come from?
    int tileNum = instance[i].tileNum;

    // Bring into view space:
    vec4 vertex_view = tile[tileNum].modelViewMatrix * instance[i].vertex;

    float range = -vertex_view.z;
    float maxRange = oe_veg_maxRange / oe_Camera.z;

    // distance culling:
    if (range >= maxRange)
        return;

    // frustum culling:
    vec4 clipLL = gl_ProjectionMatrix * (vertex_view + vec4(-instance[i].width, 0, 0, 0));
    clipLL.xy /= clipLL.w;
    if (clipLL.x > 1.0 || clipLL.y > 1.0)
        return;

    vec4 clipUR = gl_ProjectionMatrix * (vertex_view + vec4(instance[i].width, instance[i].height, 0, 0));
    clipUR.xy /= clipUR.w;
    if (clipUR.x < -1.0 || clipUR.y < -1.0)
        return;

    // Model versus billboard selection:
    bool bbExists = instance[i].billboardCommand >= 0;
    bool modelExists = instance[i].modelCommand >= 0;

    // If model, make sure we're within the SSE limit:
    vec2 pixelSizeRatio = vec2(1);
    if (modelExists)
    {
        vec2 pixelSize = 0.5*(clipUR.xy-clipLL.xy) * oe_Camera.xy;
        pixelSizeRatio = pixelSize / vec2(oe_veg_sse);
    }

    float psr = min(pixelSizeRatio.x, pixelSizeRatio.y);
    instance[i].pixelSizeRatio = psr;

    bool drawBB = bbExists && (psr < 1.0 + PSR_BUFFER || !modelExists);
    bool drawModel = modelExists && (psr > 1.0 - PSR_BUFFER || bbExists == false);

    if (drawBB)
    {
        instance[i].drawMask |= MASK_BILLBOARD;
        int first_index = instance[i].billboardCommand;
        for (uint k = first_index+1; k < oe_ic_numCommands; ++k)
            atomicAdd(cmd[k].baseInstance, 1);
    }

    if (drawModel)
    {
        instance[i].drawMask |= MASK_MODEL;
        int first_index = instance[i].modelCommand;
        for (uint k = first_index+1; k < oe_ic_numCommands; ++k)
            atomicAdd(cmd[k].baseInstance, 1);
    }
}


void sort()
{
    uint i = cullSet[gl_GlobalInvocationID.x];

    if ((instance[i].drawMask & MASK_BILLBOARD) != 0)
    {
        int cmd_index = instance[i].billboardCommand;

        // find the index of the first instance for this bin:
        uint cmdStartIndex = cmd[cmd_index].baseInstance;

        // bump the instance count for this command; the new instanceCount
        // is also the index within the command:
        uint instanceIndex = atomicAdd(cmd[cmd_index].instanceCount, 1);

        // copy to the right place in the render list
        uint index = cmdStartIndex + instanceIndex;
        renderSet[index].instance = i;
        renderSet[index].drawMask = MASK_BILLBOARD;
    }

    if ((instance[i].drawMask & MASK_MODEL) != 0)
    {
        int cmd_index = instance[i].modelCommand;

        // find the index of the first instance for this bin:
        uint cmdStartIndex = cmd[cmd_index].baseInstance;

        // bump the instance count for this command; the new instanceCount
        // is also the index within the command:
        uint instanceIndex = atomicAdd(cmd[cmd_index].instanceCount, 1);

        // insert in the render leaf list
        uint index = cmdStartIndex + instanceIndex;
        renderSet[index].instance = i;
        renderSet[index].drawMask = MASK_MODEL;
    }
}

uniform int oe_pass;

#define PASS_GENERATE 0
#define PASS_MERGE 1
#define PASS_CULL 2
#define PASS_SORT 3

void main()
{
    if (oe_pass == PASS_GENERATE)
        generate();
    else if (oe_pass == PASS_MERGE)
        merge();
    else if (oe_pass == PASS_CULL)
        cull();
    else // if (oe_pass == PASS_SORT)
        sort();
}