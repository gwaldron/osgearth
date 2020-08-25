#version 430
#pragma vp_name Texture Splatter VV
#pragma vp_entryPoint oe_splat_View
#pragma vp_location vertex_view
#extension GL_ARB_gpu_shader_int64 : enable

#define NUM_LEVELS 1
const int levels[1] = int[](17); // , 19);

// from REX SDK:
vec4 oe_terrain_getNormalAndCurvature();
vec2 oe_terrain_scaleCoordsToRefLOD(in vec2 tc, in float refLOD);

vec4 oe_layer_tilec;

out vec2 splatCoords[1];
out vec3 splatTexBlend;
out float splatLevelBlend;



// BEGIN DISP TESTING
vec3 vp_Normal;
vec4 vp_Color;
const float depth = 0.01;
vec2 oe_normalMapCoords;
vec3 oe_normalMapBinormal;
vec3 oe_UpVectorView;
vec4 oe_terrain_getNormalAndCurvature(in vec2);

float harden(in float x) {
    return 1.0 - (1.0 - x)*(1.0 - x);
}
float soften(in float x) {
    return x * x;
}
float bc(in float x, in float b, in float c) {
    return ((x - 0.5)*c + 0.5)*b;
}


const int TEX_DIM = 3;
const float TEX_DIM_F = 3.0;
#define DENSITY 0
#define MOISTURE 1
#define RUGGED 2
#define BIOME 3

struct Pixel {
    vec4 rgbh;
    vec3 normal;
    float roughness;
    float ao;
};
float density, moisture, temperature, rugged;

layout(binding = 5, std430) buffer TextureLUT {
    uint64_t texHandle[];
};
vec3 unpackNormal(in vec4 p)
{
    vec3 n;
    n.xy = p.xy*2.0 - 1.0;
    n.z = 1.0 - abs(n.x) - abs(n.y);
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
float heightAndEffectMix(in float h1, in float a1, in float h2, in float a2)
{
    // https://tinyurl.com/y5nkw2l9
    //float depth = 0.02;
    float ma = max(h1 + a1, h2 + a2) - depth;
    float b1 = max(h1 + a1 - ma, 0.0);
    float b2 = max(h2 + a2 - ma, 0.0);
    return b2 / (b1 + b2);
}


void resolveColumn(out Pixel pixel, int x, int level)
{
    vec4 rgbh[2];
    vec4 material[2];
    vec3 normal[2];

    // calulate row mixture
    float density_sb = density * ((TEX_DIM_F - 1.0) / TEX_DIM_F) + (0.5 / TEX_DIM_F);
    float yf = density_sb * (TEX_DIM_F - 1.0);
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
    float m = heightAndEffectMix(rgbh[0].a, 1.0 - y_mix, rgbh[1].a, y_mix);

    pixel.rgbh = mix(rgbh[0], rgbh[1], m);
    pixel.normal = mix(normal[0], normal[1], m);
    pixel.roughness = mix(material[0][2], material[1][2], m);
    pixel.ao = mix(material[0][2], material[1][2], m);
}

void resolveLevel(out Pixel pixel, int level)
{
    Pixel col[2];

    // calulate col mixture
    float rugged_sb = rugged * ((TEX_DIM_F - 1.0) / TEX_DIM_F) + (0.5 / TEX_DIM_F);
    float xf = rugged_sb * (TEX_DIM_F - 1.0);
    float xf_floor = floor(xf);
    int x = int(xf_floor);
    float x_mix = xf - xf_floor;

    resolveColumn(col[0], x, level);

#if 1 // ifdef out for one-column testing
    resolveColumn(col[1], clamp(x + 1, 0, TEX_DIM - 1), level);

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

uniform float shmoo;
uniform float bri, con;

void displace(inout vec4 vertex)
{
    vec3 normal = gl_NormalMatrix * oe_terrain_getNormalAndCurvature(oe_normalMapCoords).xyz;

    vec4 quad = vp_Color;
    density = quad[DENSITY];
    moisture = float(int(quad[MOISTURE] * 255.0) >> 4) / 16.0;
    temperature = float(int(quad[MOISTURE] * 255.0) & 0x0F) / 16.0;
    rugged = quad[RUGGED];
    Pixel pixel;
    resolveLevel(pixel, 0);

    vertex.xyz += normalize(normal) * bc(pixel.rgbh.a, bri, con) * shmoo;
}
//END DISP TESTING



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

    //displace(vertex_view);
}

[break]

#version 430
#pragma vp_name Texture Splatter FS
#pragma vp_entryPoint oe_splat_Frag
#pragma vp_location fragment
#extension GL_ARB_gpu_shader_int64 : enable

layout(binding = 5, std430) buffer TextureLUT {
    uint64_t texHandle[];
};

#define DENSITY 0
#define MOISTURE 1
#define RUGGED 2
#define BIOME 3

vec3 vp_Normal;
vec3 oe_UpVectorView;
float oe_roughness;
float oe_ao;
in vec3 splatTexBlend;
in float splatLevelBlend;

#define NUM_LEVELS 1
in vec2 splatCoords[1];

flat in int maxLevel;

uniform float density_power = 1.0;
uniform float moisture_power = 1.0;
uniform float temperature_power = 1.0;
uniform float rugged_power = 1.0;
uniform float depth = 0.02; 
uniform float snow = 0.0;

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

float bc(in float x, in float b, in float c)
{
    return ((x - 0.5)*c + 0.5)*b;
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

struct Pixel {
    vec4 rgbh;
    vec3 normal;
    float roughness;
    float ao;
};

float density, moisture, temperature, rugged;


void resolveColumn(out Pixel pixel, int x, int level)
{
    vec4 rgbh[2];
    vec4 material[2];
    vec3 normal[2];

    // calulate row mixture
    float yf = density * (TEX_DIM_F - 1.0);
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

void resolveLevel(out Pixel pixel, int level)
{
    Pixel col[2];

    // calulate col mixture
    float xf = rugged * (TEX_DIM_F - 1.0);
    float xf_floor = floor(xf);
    int x = int(xf_floor);
    float x_mix = xf - xf_floor;

    resolveColumn(col[0], x, level);

#if 1 // ifdef out for one-column testing
    resolveColumn(col[1], clamp(x + 1, 0, TEX_DIM - 1), level);

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

uniform float shmoo;
uniform float bri, con;

void oe_splat_Frag(inout vec4 quad)
{
    // local reference frame for normals:
    vec3 tangent = cross(gl_NormalMatrix * vec3(0, 0, 1), vp_Normal);
    vec3 binormal = cross(tangent, vp_Normal);
    mat3 tbn = mat3(normalize(tangent), normalize(binormal), vp_Normal);

    density = quad[DENSITY];
    density = amplify(density, density_power);

    moisture = float(int(quad[MOISTURE] * 255.0) >> 4) / 16.0;
    moisture = amplify(moisture, moisture_power);

    temperature = float(int(quad[MOISTURE] * 255.0) & 0x0F) / 16.0;
    temperature = amplify(temperature, temperature_power);

    rugged = quad[RUGGED];
    rugged = amplify(rugged, rugged_power);

    Pixel pixel;
    resolveLevel(pixel, 0);

    // final normal output:
    vp_Normal = normalize(tbn * pixel.normal);

    oe_roughness = pixel.roughness;

    // AO effect is very subtle. Consider dumping it.
    oe_ao = pixel.ao;

    vec3 color;

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

    // final color output:
    quad = vec4(color, 1.0);

    //quad = mix(quad, vec4(vec3(oe_ao), 1.0), shmoo);
}
