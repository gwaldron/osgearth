#version 430
#pragma vp_name Texture Splatter VV
#pragma vp_function oe_splat_View, vertex_view
#extension GL_ARB_gpu_shader_int64 : enable

#pragma import_defines(OE_SPLAT_TWEAKS)

#define NUM_LEVELS 1
const int levels[2] = int[](14, 19);
flat out vec2 splat_tilexy[2];
out vec2 splat_uv[2];

// from REX SDK:
vec4 oe_terrain_getNormalAndCurvature();
vec2 oe_terrain_scaleCoordsToRefLOD(in vec2 tc, in float refLOD);

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

float mapTo01(in float value, in float lo, in float hi)
{
    return clamp((value - lo) / (hi - lo), 0.0, 1.0);
}

void oe_splat_View(inout vec4 vertex_view)
{
    // texture coordinates
    for (int i = 0; i < NUM_LEVELS; ++i)
    {
        vec2 uv = ((i & 1) == 0) ? oe_layer_tilec.st : oe_layer_tilec.ts;
        
        splat_uv[i] = oe_terrain_scaleCoordsToRefLOD(uv, levels[i]);

        splat_tilexy[i] = floor(oe_tile_key.xy / exp2(oe_tile_key.z - levels[i]));
    }
    splatLevelBlend = mapTo01(-vertex_view.z, oe_splat_blend_start, oe_splat_blend_end);

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

float mapTo01(in float value, in float lo, in float hi)
{
    return clamp((value - lo) / (hi - lo), 0.0, 1.0);
}

in vec3 vp_Normal;
in vec3 oe_UpVectorView;
float oe_roughness;
float oe_ao;
in float splatLevelBlend;
in vec4 oe_layer_tilec;

#define NUM_LEVELS 1
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
tweakable float oe_splat_blend_normal_mix = 0.8;
tweakable float brightness = 1.0;
tweakable float contrast = 1.0;

in float oe_layer_opacity;


mat3 oe_normalMapTBN;


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


// testing code for scale
uniform float tex_size_scale = 1.0;

// compute the splatting texture coordinate by combining the macro (tile_xy)
// and micro (local xy) components. Cannot do this in the VS because it will
// overflow the interpolator and pause pixel jitter
vec2 get_coord(in int index, in int level)
{
    vec2 scale = texScale[index*(level+1)] * tex_size_scale;
    vec2 a = fract(splat_tilexy[level] * scale);
    vec2 b = splat_uv[level] * scale;
    return a + b;
}

vec4 get_rgbh(in int index, in vec2 coord)
{
    return texture(sampler2D(texHandle[index*2]), coord);
}

vec4 get_material(in int index, in vec2 coord)
{
    return texture(sampler2D(texHandle[index*2 + 1]), coord);
}

float contrastify(in float v, in float c)
{
    return clamp((v - 0.5)*c + 0.5, 0.0, 1.0);
}

float modify(in float val, in float modifier)
{
#if defined(OE_LIFEMAP_DIRECT) && OE_LIFEMAP_DIRECT
    return clamp(modifier, 0.0, 1.0);
#else
    return clamp(val * modifier, 0.0, 1.0);
#endif
}

float soften(in float x)
{
    return x * x;
}

float harden(in float x)
{
    return 1.0 - (1.0 - x)*(1.0 - x);
}

float decel(in float v, in float p)
{
    return 1.0 - pow(1.0 - v, p);
}

vec2 decel(in vec2 v, in float p)
{
    return vec2(
        1.0 - pow(1.0 - v.x, p),
        1.0 - pow(1.0 - v.y, p));
}

float heightAndEffectMix(in float h1, in float a1, in float h2, in float a2, in float roughness)
{
    float d = mix(1.0, oe_depth, decel(roughness, 2.3));
    // https://tinyurl.com/y5nkw2l9
    //float depth = 0.02;
    float ma = max(h1 + a1, h2 + a2) - d;
    float b1 = max(h1 + a1 - ma, 0.0);
    float b2 = max(h2 + a2 - ma, 0.0);
    return b2 / (b1 + b2);
}

struct Pixel {
    vec4 rgbh;
    vec3 normal;
    float roughness;
    float ao;
};

float dense, lush, rugged;

void resolveColumn(out Pixel pixel, int level, int x, float yvar)
{
    vec4 rgbh[2];
    vec4 material[2];
    vec3 normal[2];
    vec2 coord;

    // calulate row mixture
    float yf = yvar * (float(OE_TEX_DIM_Y) - 1.0);
    float yf_floor = floor(yf);
    int y = int(yf_floor);
    float y_mix = yf - yf_floor;

    // the "*2" is because each material is a pair of samplers (rgbh, nnsa)
    int i = (y*OE_TEX_DIM_X + x); // *2;
    coord = get_coord(i, level);
    rgbh[0] = get_rgbh(i, coord);
    material[0] = get_material(i, coord);
    normal[0] = unpackNormal(material[0]);

    if (y < OE_TEX_DIM_Y - 1) {
        i += (OE_TEX_DIM_X); // *2); // advance to next row
        coord = get_coord(i, level);
    }

    rgbh[1] = get_rgbh(i, coord);
    material[1] = get_material(i, coord);
    normal[1] = unpackNormal(material[1]);

    // blend with working image using both heightmap and effect:
    float rr = max(material[0][2], material[1][2]);
    float m = heightAndEffectMix(rgbh[0].a, 1.0-y_mix, rgbh[1].a, y_mix, rr);

    pixel.rgbh = mix(rgbh[0], rgbh[1], m);
    pixel.normal = mix(normal[0], normal[1], m);
    pixel.roughness = mix(material[0][2], material[1][2], m);
    pixel.ao = mix(material[0][3], material[1][3], m);
}

void resolveLevel(out Pixel pixel, int level, float xvar, float yvar)
{
    Pixel col[2];

    // calulate col mixture
    float xf = xvar * (float(OE_TEX_DIM_X) - 1.0);
    float xf_floor = floor(xf);
    int x = int(xf_floor);
    float x_mix = xf - xf_floor;

    resolveColumn(col[0], level, x, yvar);

#if 1 // ifdef out for one-column testing
    resolveColumn(col[1], level, clamp(x + 1, 0, OE_TEX_DIM_X - 1), yvar);

    // blend with working image using both heightmap and effect:
    float rr = max(col[0].roughness, col[1].roughness);
    float m = heightAndEffectMix(col[0].rgbh.a, 1.0 - x_mix, col[1].rgbh.a, x_mix, rr);

    if (level == 0)
    {
        pixel.rgbh = mix(col[0].rgbh, col[1].rgbh, m);
        pixel.normal = mix(col[0].normal, col[1].normal, m);
        pixel.roughness = mix(col[0].roughness, col[1].roughness, m);
        pixel.ao = mix(col[0].ao, col[1].ao, m);
    }
    else
    {
        Pixel temp;

        temp.rgbh = mix(col[0].rgbh, col[1].rgbh, m);
        temp.normal = mix(col[0].normal, col[1].normal, m);
        temp.roughness = mix(col[0].roughness, col[1].roughness, m);
        temp.ao = mix(col[0].ao, col[1].ao, m);

        pixel.rgbh = mix(pixel.rgbh, temp.rgbh, min(splatLevelBlend, oe_splat_blend_rgbh_mix));
        pixel.normal = mix(pixel.normal, temp.normal, min(splatLevelBlend, oe_splat_blend_normal_mix));

        //pixel.normal = normalize(pixel.normal + temp.normal*min(splatLevelBlend, oe_splat_blend_normal_mix));

        pixel.roughness = mix(pixel.roughness, temp.roughness, splatLevelBlend);
        pixel.ao = min(pixel.ao, temp.ao); // mix(pixel.ao, temp.ao, splatLevelBlend);
    }
#else
    pixel = col[0];
#endif
}

void oe_splat_Frag(inout vec4 quad)
{
    vec2 uv = (OE_LIFEMAP_MAT * oe_layer_tilec).st;

    quad = texture(OE_LIFEMAP_TEX, uv);

    dense = quad[DENSE];
    dense = modify(dense, dense_power);

    lush = quad[LUSH];
    lush = modify(lush, lush_power);

    rugged = quad[RUGGED];
    rugged = modify(rugged, rugged_power);

    Pixel pixel;
    for (int i = 0; i < NUM_LEVELS; ++i)
    {
        resolveLevel(pixel, i, rugged, lush);
    }

    pixel.normal.xy = decel(pixel.normal.xy, normal_power);

    // Note the flipped X and Y. Not sure what's up. Prob the source data.
    vp_Normal = normalize(vp_Normal + oe_normalMapTBN * pixel.normal.yxz);

    oe_roughness = pixel.roughness;

    // AO effect is very subtle. Consider dumping it.
    oe_ao = pow(pixel.ao, ao_power);

    vec3 color;

    pixel.rgbh.rgb = clamp(((pixel.rgbh.rgb - 0.5)*contrast + 0.5) * brightness, 0, 1);

#if 1
    // perma-snow caps:
    float coldness = mapTo01(oe_elev, oe_snow_min_elev, oe_snow_max_elev);
    float min_snow_cos_angle = 1.0 - soften(oe_snow*coldness);
    const float snow_buf = 0.01;
    float b = min(min_snow_cos_angle + snow_buf, 1.0);
    float cos_angle = dot(vp_Normal, oe_UpVectorView);
    //float snowiness = smoothstep(min_snow_cos_angle, b, cos_angle);
    float snowiness = step(min_snow_cos_angle, cos_angle);
    color = mix(pixel.rgbh.rgb, vec3(1), snowiness);
    oe_roughness = mix(oe_roughness, 0.1, snowiness);
#else
    color = pixel.rgbh.rgb;
#endif

    float alpha = 1.0;

#ifdef OE_COLOR_LAYER_TEX
    vec3 cltexel = texture(OE_COLOR_LAYER_TEX, (OE_COLOR_LAYER_MAT*oe_layer_tilec).st).rgb;
    vec3 clcolor = clamp(2.0 * cltexel * color, 0.0, 1.0);
    //color = mix(color, clcolor, smoothstep(0.0, 0.5, 1.0 - 0.33*(dense+lush+rugged)));
    color = mix(color, clcolor, smoothstep(0.0, 0.5, 1.0 - 0.5*(dense + lush)));
#else
    //float alpha = mix((0.5 + 0.25*(dense + lush)), 1.0, oe_layer_opacity);
    //alpha = 1.0; // oe_layer_opacity;
#endif

    alpha *= oe_layer_opacity;
    // final color output:
    quad = vec4(color, alpha);

#if 0 //debug
    if (oe_snow > 0.99)
    {
        vec3 n = vp_Normal;
        //if (n.x > 0.0 && n.x > abs(n.y))
        //    quad = vec4(1, 0, 0, 1);
        //else if (n.x <= 0.0 && n.x < -abs(n.y))
        //    quad = vec4(1, 1, 0, 1);
        //else if (n.y > 0.0 && n.y > abs(n.x))
        //    quad = vec4(0, 1, 0, 1);
        //else
        //    quad = vec4(0, 0, 1, 1);

        quad = vec4(n.x, 0, n.y, 1)*0.5 + 0.5;
    }
#endif
}
