#pragma vp_name Texture Splatter VV
#pragma vp_function oe_splat_View, vertex_view

#pragma import_defines(OE_SPLAT_TWEAKS)
#pragma import_defines(OE_SPLAT_NUM_LEVELS)
#pragma import_defines(OE_SNOW)

const int levels[2] = int[](14, 19);
flat out vec2 splat_tilexy[2];
out vec2 splat_uv[2];

// from REX SDK:
vec4 oe_terrain_scaleCoordsAndTileKeyToRefLOD(in vec2 tc, in float refLOD);

out vec4 oe_layer_tilec;
out float oe_splat_levelblend;

#ifdef OE_SNOW
out float oe_elev;
float oe_terrain_getElevation();
#endif

#ifdef OE_SPLAT_TWEAKS
#define tweakable uniform
#else
#define tweakable const
#endif

tweakable float oe_splat_blend_start = 2500.0;
tweakable float oe_splat_blend_end = 500.0;

#define MAP_TO_01(VAL,LO,HI) clamp((VAL-LO) / (HI-LO), 0.0, 1.0)

void oe_splat_View(inout vec4 vertex_view)
{
    // texture coordinates
    for (int i = 0; i < OE_SPLAT_NUM_LEVELS; ++i)
    {
        vec4 uvxy = oe_terrain_scaleCoordsAndTileKeyToRefLOD(oe_layer_tilec.st, levels[i]);
        splat_uv[i] = uvxy.xy;
        splat_tilexy[i] = uvxy.zw;
    }
    oe_splat_levelblend = MAP_TO_01(-vertex_view.z, oe_splat_blend_start, oe_splat_blend_end);

#ifdef OE_SNOW
    oe_elev = oe_terrain_getElevation();
#endif
}


[break]
#pragma vp_name Texture Splatter FS
#pragma vp_function oe_splat_Frag, fragment, 0.8

#pragma oe_use_shared_layer(OE_LIFEMAP_TEX, OE_LIFEMAP_MAT)
//#pragma oe_use_shared_layer(OE_COLOR_LAYER_TEX, OE_COLOR_LAYER_MAT)

#pragma import_defines(OE_TEX_DIM_X)
#pragma import_defines(OE_TEX_DIM_Y)

#pragma import_defines(OE_SPLAT_TWEAKS)
#pragma import_defines(OE_LIFEMAP_DIRECT)
#pragma import_defines(OE_SPLAT_USE_MTL_GLS_AO)
#pragma import_defines(OE_SNOW)

layout(binding = 5, std430) buffer SplatTextureArena {
    uint64_t texHandle[];
};

uniform float oe_texScale[OE_TEX_DIM_X*OE_TEX_DIM_Y];

#define RUGGED 0
#define DENSE 1
#define LUSH 2
#define SPECIAL 3

in vec3 vp_Normal;
in vec3 vp_VertexView;
in vec3 oe_UpVectorView;
in float oe_splat_levelblend;
in vec4 oe_layer_tilec;

#pragma import_defines(OE_SPLAT_NUM_LEVELS)
flat in vec2 splat_tilexy[2];
in vec2 splat_uv[2];

#ifdef OE_SNOW
in float oe_elev;
#endif

#ifdef OE_LIFEMAP_DIRECT
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
tweakable float oe_splat_brightness = 1.0;
tweakable float oe_splat_contrast = 1.0;
tweakable float oe_dense_contrast = 0.35;
tweakable float oe_dense_brightness = -0.5;

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

// compute the splatting texture coordinate by combining the macro (tile_xy)
// and micro (local xy) components. Cannot do this in the VS because it will
// overflow the interpolator and pause pixel jitter
void get_coord(out vec2 coord, in int index, in int level)
{
    vec2 scale = vec2(oe_texScale[index]);
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
    //float d = mix(oe_depth, 1.0, clamp((-vp_VertexView.z-500.0) / 500.0, 0.0, 1.0));
    float d = oe_depth; // mix(1.0, oe_depth, DECEL(roughness, 2.3));
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

void resolveRow(out Pixel result, int level, int row, float xvar)
{
    Pixel p1, p2;
    vec2 coord;

    // calulate first column index and mix factor
    float xf = xvar * (float(OE_TEX_DIM_X) - 1.0);
    float xf_floor = floor(xf);
    int x = int(xf_floor);
    float x_mix = xf - xf_floor;

    // texture index:
    int i = row * OE_TEX_DIM_X + x;

    // read both columns:
    get_coord(coord, i, level);
    get_pixel(p1, i, coord);
    i = (i%OE_TEX_DIM_X < OE_TEX_DIM_X) ? i + 1 : i;
    get_coord(coord, i, level);
    get_pixel(p2, i, coord);

    // blend them using both heightmap and roughness:
    float r = max(p1.material[ROUGHNESS], p2.material[ROUGHNESS]);
    float m = heightAndEffectMix(p1.rgbh.a, 1.0 - x_mix, p2.rgbh.a, x_mix, r);
    pixmix(result, p1, p2, m);
}

void resolveLevel(out Pixel result, int level, float rugged, float lush, float dense)
{
    // resolve the substrate (dirt and rocks)
    Pixel substrate;
    resolveRow(substrate, level, 0, rugged);

    // resolve the surface texture (greenery and debris)
    Pixel surface;
    resolveRow(surface, level, 1, lush);

    // use density to modulate the depth blend between the two.
    float m = heightAndEffectMix(
        substrate.rgbh[3], 1.0-dense,
        surface.rgbh[3], dense,
        1.0);

    if (level == 0)
    {
        pixmix(result, substrate, surface, m);
    }
    else
    {
        Pixel temp;
        pixmix(temp, substrate, surface, m);

        float mat_mix = min(oe_splat_levelblend, oe_splat_blend_rgbh_mix);
        result.rgbh = mix(result.rgbh, temp.rgbh, mat_mix);
        result.normal = mix(result.normal, temp.normal, min(oe_splat_levelblend, oe_splat_blend_normal_mix));
        result.material = mix(result.material, temp.material, mat_mix);
    }
}

void oe_splat_Frag(inout vec4 quad)
{
    // sample the life map and extract the compenents:
    vec4 life = texture(OE_LIFEMAP_TEX, (OE_LIFEMAP_MAT * oe_layer_tilec).st);
    float rugged = MODIFY(life[RUGGED], rugged_power);
    float lush = MODIFY(life[LUSH], lush_power);
    float dense = MODIFY(life[DENSE], dense_power);
    
    // compute the pixel color:
    Pixel pixel;
    for (int level = 0; level < OE_SPLAT_NUM_LEVELS; ++level)
    {
        resolveLevel(pixel, level, rugged, lush, dense);
    }

    // apply PBR
    oe_pbr.roughness *= pixel.material[ROUGHNESS];
    oe_pbr.ao *= pow(pixel.material[AO], ao_power);
    oe_pbr.metal = pixel.material[METAL];

    float f_c = oe_splat_contrast + (dense * oe_dense_contrast);
    float f_b = oe_splat_brightness + (dense * oe_dense_brightness);
    pixel.rgbh.rgb = clamp(((pixel.rgbh.rgb - 0.5)*f_c + 0.5) * f_b, 0, 1);

    vec3 color = pixel.rgbh.rgb;

    // WATER
    float water = life.a;
    const vec3 water_color = vec3(0.02, 0.05, 0.1);
    color = mix(color, water_color, water);
    oe_pbr.roughness = mix(oe_pbr.roughness, 0.3, water);
    oe_pbr.ao = mix(oe_pbr.ao, 1.0, water);

    // NORMAL
    pixel.normal.xy = vec2(
        DECEL(pixel.normal.x, normal_power),
        DECEL(pixel.normal.y, normal_power));
    vp_Normal = mix(normalize(vp_Normal + oe_normalMapTBN * pixel.normal), oe_UpVectorView, water);

#ifdef OE_SNOW
    // SNOW
    float coldness = MAP_TO_01(oe_elev, oe_snow_min_elev, oe_snow_max_elev);
    float cos_angle = max(0, dot(vp_Normal, oe_UpVectorView));
    float snowiness = heightAndEffectMix(pixel.rgbh.a, 1.0, oe_snow, cos_angle, 0.0) * (1.0 - water);
    color = mix(color, vec3(1), snowiness);
    oe_pbr.roughness = mix(oe_pbr.roughness, 0.1, snowiness);
#endif

#ifdef OE_COLOR_LAYER_TEX
    vec3 cltexel = texture(OE_COLOR_LAYER_TEX, (OE_COLOR_LAYER_MAT*oe_layer_tilec).st).rgb;
    vec3 clcolor = clamp(2.0 * cltexel * color, 0.0, 1.0);
    color = mix(color, clcolor, smoothstep(0.0, 0.5, 1.0 - 0.5*(dense + lush)));
#endif

    // final color output:
    quad = vec4(color, oe_layer_opacity);
}
