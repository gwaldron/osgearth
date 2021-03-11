#version 430
#pragma vp_name Texture Splatter VV
#pragma vp_entryPoint oe_splat_View
#pragma vp_location vertex_view
#extension GL_ARB_gpu_shader_int64 : enable

#define NUM_LEVELS 1
const int levels[1] = int[](18); // , 19);

// from REX SDK:
vec4 oe_terrain_getNormalAndCurvature();
vec2 oe_terrain_scaleCoordsToRefLOD(in vec2 tc, in float refLOD);

out vec4 oe_layer_tilec;

out vec2 splatCoords[1];
out float splatLevelBlend;


float mapToNormalizedRange(in float value, in float lo, in float hi)
{
    return clamp((value - lo) / (hi - lo), 0.0, 1.0);
}

void oe_splat_View(inout vec4 vertex_view)
{
    // texture coordinates
    for (int i = 0; i < NUM_LEVELS; ++i)
    {
        vec2 uv = ((i & 1) == 0) ? oe_layer_tilec.st : oe_layer_tilec.ts;
        splatCoords[i] = oe_terrain_scaleCoordsToRefLOD(uv, levels[i]);
    }

    // transition b/w levels
    const float start = 175.0;
    const float end = 100.0;
    splatLevelBlend = mapToNormalizedRange(-vertex_view.z, start, end);
}

[break]

#version 430
#pragma vp_name Texture Splatter FS
#pragma vp_entryPoint oe_splat_Frag
#pragma vp_location fragment
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

layout(binding = 5, std430) buffer TextureLUT {
    uint64_t texHandle[];
};

#define DENSE 0
#define LUSH 1
#define RUGGED 2

vec3 vp_Normal;
vec3 oe_UpVectorView;
float oe_roughness;
float oe_ao;
in float splatLevelBlend;
in vec4 oe_layer_tilec;

#define NUM_LEVELS 1
in vec2 splatCoords[1];

flat in int maxLevel;

uniform float dense_power = 1.0;
uniform float lush_power = 1.0;
uniform float rugged_power = 1.0;
uniform float normal_power = 1.0;
uniform float depth = 0.02; 
uniform float snow = 0.0;

in float oe_layer_opacity;

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

vec4 get_rgbh(in int index, in int level)
{
    return texture(sampler2D(texHandle[index]), splatCoords[level]);
}

vec4 get_material(in int index, in int level)
{
    return texture(sampler2D(texHandle[index + 1]), splatCoords[level]);
}

float amplify(in float val, in float modifier)
{
    //return modifier;
    return clamp(val * modifier, 0.0, 1.0);
}

float soften(in float x)
{
    return x * x;
}

float harden(in float x)
{
    return 1.0 - (1.0 - x)*(1.0 - x);
}

float heightAndEffectMix(in float h1, in float a1, in float h2, in float a2)
{
    // https://tinyurl.com/y5nkw2l9
    //float depth = 0.02;
    float ma = max(h1 + a1, h2 + a2) - depth;
    float b1 = max(h1 + a1 - ma, 0.0);
    float b2 = max(h2 + a2 - ma, 0.0);
    return b2 / (b1 + b2);
}

// 3x3 material matrix
const int TEX_DIM = 3;
const float TEX_DIM_F = 3.0;
//const int TEX_DIM = 2;
//const float TEX_DIM_F = 2.0;

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

    // calulate row mixture
    float yf = yvar * (TEX_DIM_F - 1.0);
    float yf_floor = floor(yf);
    int y = int(yf_floor);
    float y_mix = yf - yf_floor;

    // the "*2" is because each material is a pair of samplers (rgbh, nnsa)
    int i = (y*TEX_DIM + x) * 2;
    rgbh[0] = get_rgbh(i, level);
    material[0] = get_material(i, level);
    normal[0] = unpackNormal(material[0]);

    if (y < TEX_DIM - 1)
        i += (TEX_DIM * 2);
    rgbh[1] = get_rgbh(i, level);
    material[1] = get_material(i, level);
    normal[1] = unpackNormal(material[1]);

    // blend with working image using both heightmap and effect:
    float m = heightAndEffectMix(rgbh[0].a, 1.0-y_mix, rgbh[1].a, y_mix);

    pixel.rgbh = mix(rgbh[0], rgbh[1], m);
    pixel.normal = mix(normal[0], normal[1], m);
    pixel.roughness = mix(material[0][2], material[1][2], m);
    pixel.ao = mix(material[0][2], material[1][2], m);
}

void resolveLevel(out Pixel pixel, int level, float xvar, float yvar)
{
    Pixel col[2];

    // calulate col mixture
    float xf = xvar * (TEX_DIM_F - 1.0);
    float xf_floor = floor(xf);
    int x = int(xf_floor);
    float x_mix = xf - xf_floor;

    resolveColumn(col[0], level, x, yvar);

#if 1 // ifdef out for one-column testing
    resolveColumn(col[1], level, clamp(x + 1, 0, TEX_DIM - 1), yvar);

    // blend with working image using both heightmap and effect:
    float m = heightAndEffectMix(col[0].rgbh.a, 1.0 - x_mix, col[1].rgbh.a, x_mix);

    pixel.rgbh = mix(col[0].rgbh, col[1].rgbh, m);
    pixel.normal = mix(col[0].normal, col[1].normal, m);
    pixel.roughness = mix(col[0].roughness, col[1].roughness, m);
    pixel.ao = mix(col[0].ao, col[1].ao, m);
#else
    pixel = col[0];
#endif
}

vec2 decel(in vec2 v, in float p)
{
    return vec2(
        1.0 - pow(1.0 - v.x, p),
        1.0 - pow(1.0 - v.y, p));
}

void oe_splat_Frag(inout vec4 quad)
{
    quad = texture(OE_LIFEMAP_TEX, (OE_LIFEMAP_MAT * oe_layer_tilec).st);

    // local reference frame for normals:
    vec3 tangent = cross(gl_NormalMatrix * vec3(0, 0, 1), vp_Normal);
    vec3 binormal = cross(tangent, vp_Normal);
    mat3 tbn = mat3(normalize(tangent), normalize(binormal), vp_Normal);

    dense = quad[DENSE];
    dense = amplify(dense, dense_power);

    lush = quad[LUSH];
    lush = amplify(lush, lush_power);

    rugged = quad[RUGGED];
    rugged = amplify(rugged, rugged_power);

    Pixel pixel;
    resolveLevel(pixel, 0, rugged, lush);

    pixel.normal.xy = decel(pixel.normal.xy, normal_power);

    // final normal output:
    vp_Normal = normalize(tbn * pixel.normal);

    oe_roughness = pixel.roughness;

    // AO effect is very subtle. Consider dumping it.
    oe_ao = pixel.ao;

    vec3 color;

#if 0
    // perma-show caps:
    float snowiness = 0.0;
    float coldness = 1.0 - clamp(temperature, -1, 1);
    float min_snow_cos_angle = 1.0 - soften(snow*coldness*rugged);
    const float snow_buf = 0.05;
    float b = min(min_snow_cos_angle + snow_buf, 1.0);
    float cos_angle = dot(vp_Normal, oe_UpVectorView);
    snowiness = smoothstep(min_snow_cos_angle, b, cos_angle);
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
}
