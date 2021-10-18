#version 430
#pragma vp_name Texture Splatter VV
#pragma vp_function oe_splat_View, vertex_view
#extension GL_ARB_gpu_shader_int64 : enable

#pragma import_defines(OE_SPLAT_TWEAKS)
#pragma import_defines(OE_SPLAT_NUM_LEVELS)

const int levels[2] = int[](14, 19);
flat out vec2 splat_tilexy[2];
out vec2 splat_uv[2];

// from REX SDK:
vec4 oe_terrain_getNormalAndCurvature();
vec4 oe_terrain_scaleCoordsAndTileKeyToRefLOD(in vec2 tc, in float refLOD);

out vec4 oe_layer_tilec;
out float splatLevelBlend;
out float oe_elev;

float oe_terrain_getElevation();

#ifdef OE_SPLAT_TWEAKS
#define tweakable uniform
#else
#define tweakable const
#endif

tweakable float oe_splat_blend_start = 2500.0;
tweakable float oe_splat_blend_end = 500.0;

uniform vec4 oe_tile_key;

#define MAP_TO_01(VAL,LO,HI) clamp((VAL-LO) / (HI-LO), 0.0, 1.0)

void oe_splat_View(inout vec4 vertex_view)
{
    // texture coordinates
    for (int i = 0; i < OE_SPLAT_NUM_LEVELS; ++i)
    {
        vec4 uvxy = oe_terrain_scaleCoordsAndTileKeyToRefLOD(oe_layer_tilec.st, levels[i]);
        splat_uv[i] = uvxy.xy; // oe_terrain_scaleCoordsToRefLOD(oe_layer_tilec.st, levels[i]);
        splat_tilexy[i] = uvxy.zw; // floor(oe_tile_key.xy / exp2(oe_tile_key.z - float(levels[i])));
    }
    splatLevelBlend = MAP_TO_01(-vertex_view.z, oe_splat_blend_start, oe_splat_blend_end);

    oe_elev = oe_terrain_getElevation();
}


[break]
#version 430
#pragma vp_name Texture Splatter FS
#pragma vp_function oe_splat_Frag, fragment, 0.8
#extension GL_ARB_gpu_shader_int64 : enable

#pragma import_defines(OE_LIFEMAP_TEX)
#pragma import_defines(OE_LIFEMAP_MAT)
uniform sampler2D OE_LIFEMAP_TEX;
uniform mat4 OE_LIFEMAP_MAT;

#pragma import_defines(OE_COLOR_LAYER_TEX)
#pragma import_defines(OE_COLOR_LAYER_MAT)
#ifdef OE_COLOR_LAYER_TEX
uniform sampler2D OE_COLOR_LAYER_TEX;
uniform mat4 OE_COLOR_LAYER_MAT;
#endif

#pragma import_defines(OE_TEX_DIM_X)
#pragma import_defines(OE_TEX_DIM_Y)

#pragma import_defines(OE_SPLAT_TWEAKS)
#pragma import_defines(OE_LIFEMAP_DIRECT)
#pragma import_defines(OE_SPLAT_USE_MTL_GLS_AO)

layout(binding = 5, std430) buffer TextureLUT {
    uint64_t texHandle[];
};
layout(binding = 6, std430) buffer RenderParamsLUT {
    vec2 texScale[];
};

#define RUGGED 0
#define DENSE 1
#define LUSH 2
#define SPECIAL 3

in vec3 vp_Normal;
in vec3 oe_UpVectorView;
in float splatLevelBlend;
in vec4 oe_layer_tilec;

#pragma import_defines(OE_SPLAT_NUM_LEVELS)
flat in vec2 splat_tilexy[2];
in vec2 splat_uv[2];

flat in int maxLevel;
in float oe_elev;

#ifdef OE_SPLAT_TWEAKS
    #define tweakable uniform
#else
    #define tweakable const
#endif

tweakable float dense_power = 1.0;
tweakable float lush_power = 1.0;
tweakable float rugged_power = 1.0;
tweakable float normal_power = 1.0;
tweakable float ao_power = 1.0;
tweakable float oe_depth = 0.02;
tweakable float oe_snow = 0.0;
tweakable float oe_snow_min_elev = 1000.0;
tweakable float oe_snow_max_elev = 3500.0;
tweakable float oe_splat_blend_rgbh_mix = 0.8;
tweakable float oe_splat_blend_normal_mix = 0.85;
tweakable float brightness = 1.0;
tweakable float contrast = 1.0;
tweakable float dense_contrast = 0.35;
tweakable float dense_brightness = -0.5;

in float oe_layer_opacity;

mat3 oe_normalMapTBN;

#define MAP_TO_01(VAL,LO,HI) clamp((VAL-LO) / (HI-LO), 0.0, 1.0)
#define SOFTEN(V) ((V)*(V))
#define DECEL(V,P) (1.0-pow(1.0-(V),(P)))

#if defined(OE_LIFEMAP_DIRECT) && OE_LIFEMAP_DIRECT
#define MODIFY(V,M) clamp((M), 0.0, 1.0)
#else
#define MODIFY(V,M) clamp((V)*(M), 0.0, 1.0)
#endif

vec3 unpackNormal(in vec4 p)
{
    vec3 n;
    n.xy = p.xy*2.0 - 1.0;
    n.z = 1.0 - abs(n.x) - abs(n.y);
    // unnecessary since Z is never < 0:
    //float t = clamp(-n.z, 0, 1);
    //n.x += (n.x > 0)? -t : t;
    //n.y += (n.y > 0)? -t : t;
    return normalize(n);
}

struct Pixel {
    vec4 rgbh;
    vec3 normal;
    vec3 material;
};

#define ROUGHNESS 0
#define AO 1
#define METAL 2

// fragment stage global PBR parameters.
struct OE_PBR {
    float roughness;
    float ao;
    float metal;
    float brightness;
    float contrast;
} oe_pbr;

// testing code for scale
uniform float tex_size_scale = 1.0;

// compute the splatting texture coordinate by combining the macro (tile_xy)
// and micro (local xy) components. Cannot do this in the VS because it will
// overflow the interpolator and pause pixel jitter
void get_coord(out vec2 coord, in int index, in int level)
{
    vec2 scale = texScale[index] * tex_size_scale;
    vec2 a = fract(splat_tilexy[level] * scale);
    vec2 b = splat_uv[level] * scale;
    coord = a + b;
}

void get_pixel(out Pixel res, in int index, in vec2 coord)
{
    res.rgbh = texture(sampler2D(texHandle[index * 2]), coord);
    vec4 temp = texture(sampler2D(texHandle[index * 2 + 1]), coord);
    res.normal = unpackNormal(temp);
    res.material = vec3(temp[2], temp[3], 0.0); // roughness, ao, metal
}

float heightAndEffectMix(in float h1, in float a1, in float h2, in float a2, in float roughness)
{
    float d = mix(1.0, oe_depth, DECEL(roughness, 2.3));
    // https://tinyurl.com/y5nkw2l9
    //float depth = 0.02;
    float ma = max(h1 + a1, h2 + a2) - d;
    float b1 = max(h1 + a1 - ma, 0.0);
    float b2 = max(h2 + a2 - ma, 0.0);
    return b2 / (b1 + b2);
}

void pixmix(out Pixel res, in Pixel p1, in Pixel p2, float m)
{
    res.rgbh = mix(p1.rgbh, p2.rgbh, m);
    res.normal = mix(p1.normal, p2.normal, m);
    res.material = mix(p1.material, p2.material, m);
}

void resolveColumn(out Pixel pixel, int level, int x, float yvar)
{
    Pixel p1, p2;
    vec2 coord;

    // calulate row mixture
    float yf = yvar * (float(OE_TEX_DIM_Y) - 1.0);
    float yf_floor = floor(yf);
    int y = int(yf_floor);
    float y_mix = yf - yf_floor;

    // calc texture index and read two rows
    int i = (y*OE_TEX_DIM_X + x);
    get_coord(coord, i, level);
    get_pixel(p1, i, coord);

    if (y < OE_TEX_DIM_Y - 1)
    {
        i += (OE_TEX_DIM_X); // advance to next row
        get_coord(coord, i, level);
    }
    get_pixel(p2, i, coord);

    // blend with working image using both heightmap and effect:
    float r = max(p1.material[ROUGHNESS], p2.material[ROUGHNESS]);
    float m = heightAndEffectMix(p1.rgbh.a, 1.0 - y_mix, p2.rgbh.a, y_mix, r);

    pixmix(pixel, p1, p2, m);
}

void resolveLevel(out Pixel pixel, int level, float xvar, float yvar)
{
    Pixel c1, c2; // adjacent columns

    // calulate col mixture
    float xf = xvar * (float(OE_TEX_DIM_X) - 1.0);
    float xf_floor = floor(xf);
    int x = int(xf_floor);
    float x_mix = xf - xf_floor;

    resolveColumn(c1, level, x, yvar);

    resolveColumn(c2, level, clamp(x + 1, 0, OE_TEX_DIM_X - 1), yvar);

    // blend with working image using both heightmap and effect:
    float rr = max(c1.material[ROUGHNESS], c2.material[ROUGHNESS]);
    float m = heightAndEffectMix(c1.rgbh.a, 1.0 - x_mix, c2.rgbh.a, x_mix, rr);

    if (level == 0)
    {
        pixmix(pixel, c1, c2, m);
    }
    else
    {
        Pixel temp;
        pixmix(temp, c1, c2, m);

        float mat_mix = min(splatLevelBlend, oe_splat_blend_rgbh_mix);
        pixel.rgbh = mix(pixel.rgbh, temp.rgbh, mat_mix);
        pixel.normal = mix(pixel.normal, temp.normal, min(splatLevelBlend, oe_splat_blend_normal_mix));
        pixel.material = mix(pixel.material, temp.material, mat_mix);
    }
}

void oe_splat_Frag(inout vec4 quad)
{
    vec4 life = texture(OE_LIFEMAP_TEX, (OE_LIFEMAP_MAT * oe_layer_tilec).st);

    float dense = MODIFY(life[DENSE], dense_power);
    float lush = MODIFY(life[LUSH], lush_power);
    float rugged = MODIFY(life[RUGGED], rugged_power);

    Pixel pixel;
    for (int i = 0; i < OE_SPLAT_NUM_LEVELS; ++i)
    {
        resolveLevel(pixel, i, rugged, lush);
    }

    oe_pbr.roughness *= pixel.material[ROUGHNESS];
    oe_pbr.ao *= pow(pixel.material[AO], ao_power);
    oe_pbr.metal = pixel.material[METAL];

    vec3 color;

    float f_c = contrast + (dense * dense_contrast);
    float f_b = brightness + (dense * dense_brightness);

    pixel.rgbh.rgb = clamp(((pixel.rgbh.rgb - 0.5)*f_c + 0.5) * f_b, 0, 1);

#if 1
    // perma-snow caps:
    float coldness = MAP_TO_01(oe_elev, oe_snow_min_elev, oe_snow_max_elev);
    float min_snow_cos_angle = 1.0 - SOFTEN(oe_snow*coldness);
    const float snow_buf = 0.01;
    float b = min(min_snow_cos_angle + snow_buf, 1.0);
    float cos_angle = dot(vp_Normal, oe_UpVectorView);
    float snowiness = step(min_snow_cos_angle, cos_angle);
    color = mix(pixel.rgbh.rgb, vec3(1), snowiness);
    oe_pbr.roughness = mix(oe_pbr.roughness, 0.1, snowiness);
#else
    color = pixel.rgbh.rgb;
#endif

    // WATER
    float water = life.a;
    const vec3 water_color = vec3(0.03, 0.07, 0.12); // vec3(0.1, 0.2, 0.4);
    color = mix(color, water_color, water);
    oe_pbr.roughness = mix(oe_pbr.roughness, 0.3, water);
    oe_pbr.ao = mix(oe_pbr.ao, 1.0, water);

    // NORMAL
    float np = mix(normal_power, normal_power * 0.5, water);
    pixel.normal.xy = vec2(
        DECEL(pixel.normal.x, np),
        DECEL(pixel.normal.y, np));
    vp_Normal = normalize(vp_Normal + oe_normalMapTBN * pixel.normal);

#ifdef OE_COLOR_LAYER_TEX
    vec3 cltexel = texture(OE_COLOR_LAYER_TEX, (OE_COLOR_LAYER_MAT*oe_layer_tilec).st).rgb;
    vec3 clcolor = clamp(2.0 * cltexel * color, 0.0, 1.0);
    color = mix(color, clcolor, smoothstep(0.0, 0.5, 1.0 - 0.5*(dense + lush)));
#endif

    // final color output:
    quad = vec4(color, oe_layer_opacity);
}
