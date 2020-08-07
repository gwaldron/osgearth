#version 430
#pragma vp_name Texture Splatter VV
#pragma vp_entryPoint oe_splat_View
#pragma vp_location vertex_view
#extension GL_ARB_gpu_shader_int64 : enable

#define NUM_LEVELS 2             
const int levels[NUM_LEVELS] = int[](15, 19);

// from REX SDK:
vec4 oe_terrain_getNormalAndCurvature();
vec2 oe_terrain_scaleCoordsToRefLOD(in vec2 tc, in float refLOD);

vec4 oe_layer_tilec;

out vec2 splatCoords[NUM_LEVELS];
out vec3 splatTexBlend;
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

    //vec3 normal_model = oe_terrain_getNormalAndCurvature().xyz;   

    // triplanar blending parameters:
    //splatTexBlend = abs(normal_model);
    //splatTexBlend = normalize(max(splatTexBlend, 0.00001));
    //float b = splatTexBlend.x + splatTexBlend.y + splatTexBlend.z;
    //splatTexBlend /= vec3(b);

    // reference frame of vertex normal:
    //vec3 normal_view = gl_NormalMatrix * normal_model;                    
    //vec3 tangent = cross(gl_NormalMatrix*vec3(0,0,1), normal_view);
    //vec3 binormal = cross(tangent, normal_view);
    //oe_splat_TBN = mat3(normalize(tangent), normalize(binormal), normal_view);

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

#define NUM_LEVELS 2
in vec2 splatCoords[2];

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
    return val * modifier;
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

//vec3 triplanar(in int index)
//{
//  uint64_t h = texHandle[index];
//  const float scale = 1.0/1024.0;
//  return
//      //vec3(1,0,0) * splatTexBlend.x + vec3(0,1,0) * splatTexBlend.y + vec3(0,0,1) * splatTexBlend.z;
//   texture(sampler2D(h), splatVertex_model.yz / scale).rgb * splatTexBlend.x +
//   texture(sampler2D(h), splatVertex_model.xz / scale).rgb * splatTexBlend.y +
//   texture(sampler2D(h), splatVertex_model.xy / scale).rgb * splatTexBlend.z;
//}

struct Rec {
    int index;
    float density_effect;
    float moisture_effect;
    float rugged_effect;
};

#define NUM_TEX 3

const int TEX_ROCK = 0;
const int TEX_DIRT = 2;
const int TEX_FOREST = 4;
const int TEX_GRASS = 6;

Rec lut[4] = Rec[](
    //              DEN  MOI   RUG
    Rec(TEX_ROCK,   0.0, 0.0,  0.0),
    Rec(TEX_DIRT,   0.0, 0.0, -1.0),
    Rec(TEX_FOREST, 1.0, 0.0, -1.0),
    Rec(TEX_GRASS, -1.0, 1.0, -1.0)
);

float density, moisture, temperature, rugged;

void resolveLevel(out vec3 out_color, out vec3 out_normal, out float out_roughness, out float out_ao, int level)
{
    vec4 rgbh[NUM_TEX];
    vec4 material[NUM_TEX];

    for (int i = 0; i < NUM_TEX; ++i)
    {
        rgbh[i] = get_rgbh(lut[i].index, level);
        material[i] = get_material(lut[i].index, level);
    }

    // starting point.
    vec4 final_rgbh = rgbh[0];
    vec3 final_normal = unpackNormal(material[0]);
    float final_roughness = material[0][2];
    float final_ao = material[0][3];

    for (int i = 1; i < NUM_TEX; ++i)
    {
        // effect of each terroir component on this texture;
        // remember, each component is [-1..1] and effects are [-1..1]

        float e_density = density * lut[i].density_effect;
        float e_moisture = moisture * lut[i].moisture_effect;
        float e_rugged = rugged * lut[i].rugged_effect;

        // cummulative effect:
        float e_total = 0.5*(1.0 + (e_density + e_moisture + e_rugged));
        e_total = clamp(e_total, 0, 1);

        vec4 this_rgbh = rgbh[i];
        vec4 this_material = material[i];
        vec3 this_normal = unpackNormal(this_material);

        // blend with working image using both heightmap and effect:
        float m = heightAndEffectMix(this_rgbh.a, e_total, final_rgbh.a, 1.0 - e_total);

        final_rgbh = mix(this_rgbh, final_rgbh, m);
        final_normal = mix(this_normal, final_normal, m);
        final_roughness = mix(this_material[2], final_roughness, m);
        final_ao = mix(this_material[3], final_ao, m);
    }

    out_color = final_rgbh.rgb;
    out_normal = final_normal;
    out_roughness = final_roughness;
    out_ao = final_ao;
}

void oe_splat_Frag(inout vec4 quad)
{
    // local reference frame for normals:
    vec3 tangent = cross(gl_NormalMatrix * vec3(0, 0, 1), vp_Normal);
    vec3 binormal = cross(tangent, vp_Normal);
    mat3 tbn = mat3(normalize(tangent), normalize(binormal), vp_Normal);

    density = quad[DENSITY];
    density = density * 2.0 - 1.0;
    density = amplify(density, density_power);

    moisture = float(int(quad[MOISTURE] * 255.0) >> 4) / 16.0;
    moisture = moisture * 2.0 - 1.0;
    moisture = amplify(moisture, moisture_power);

    temperature = float(int(quad[MOISTURE] * 255.0) & 0x0F) / 16.0;
    temperature = temperature * 2.0 - 1.0;
    temperature = amplify(temperature, temperature_power);

    rugged = quad[RUGGED];
    rugged = rugged * 2.0 - 1.0;
    rugged = amplify(rugged, rugged_power);

    vec3 final_color;
    vec3 final_normal;
    float final_roughness;
    float final_ao;

    for (int level = 0; level <= 1; ++level)
    {
        vec3 color;
        vec3 normal;
        float roughness, ao;

        resolveLevel(color, normal, roughness, ao, level);

        if (level == 0)
        {
            final_color = color;
            final_normal = normal;
            final_roughness = roughness;
            final_ao = ao;
        }
        else
        {
            float mono = (color.r*0.2126 + color.g*0.7152 + color.b*0.0722);
            final_color = mix(final_color, (final_color*mono)*2.5, splatLevelBlend);
            final_normal += normal * splatLevelBlend;
            final_roughness = mix(final_roughness, roughness, splatLevelBlend);
            final_ao = mix(final_ao, ao, splatLevelBlend);
        }
    }

    // final normal output:
    vp_Normal = normalize(tbn * final_normal);

    oe_roughness = final_roughness;

    // AO effect is very subtle. Consider dumping it.
    oe_ao = final_ao;

    // perma-show caps:
    float snowiness = 0.0;
    float coldness = 1.0 - (0.5*(clamp(temperature, -1, 1) + 1.0));
    float rockiness = clamp(0.5*(rugged + 1.0), 0, 1);
    float min_snow_cos_angle = 1.0 - soften(snow*coldness*rockiness);
    const float snow_buf = 0.05;
    float b = min(min_snow_cos_angle + snow_buf, 1.0);
    float cos_angle = dot(vp_Normal, oe_UpVectorView);
    snowiness = smoothstep(min_snow_cos_angle, b, cos_angle);
    final_color.rgb = mix(final_color.rgb, vec3(1), snowiness);
    oe_roughness = mix(oe_roughness, 0.1, snowiness);

    // final color output:
    quad = vec4(final_color.rgb, 1.0);
}
