#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint atmos_fragment_main_pbr
#pragma vp_location   fragment_lighting
#pragma vp_order      0.8

#pragma import_defines(OE_LIGHTING)
#pragma import_defines(OE_NUM_LIGHTS)

uniform float oe_sky_exposure = 3.3; // HDR scene exposure (ground level)
uniform float oe_sky_contrast = 1.0;

in vec3 atmos_color;       // atmospheric lighting color
in vec3 atmos_vert; 
in vec3 atmos_atten;
        
vec3 vp_Normal;          // surface normal (from osgEarth)

// Parameters of each light:
struct osg_LightSourceParameters 
{   
   vec4 ambient;
   vec4 diffuse;
   vec4 specular;
   vec4 position;
   vec3 spotDirection;
   float spotExponent;
   float spotCutoff;
   float spotCosCutoff;
   float constantAttenuation;
   float linearAttenuation;
   float quadraticAttenuation;

   bool enabled;
};  
uniform osg_LightSourceParameters osg_LightSource[OE_NUM_LIGHTS];


// https://learnopengl.com/PBR/Lighting

const float PI = 3.14159265359;

float DistributionGGX(vec3 N, vec3 H, float roughness)
{
    float a = roughness * roughness;
    float a2 = a * a;
    float NdotH = max(dot(N, H), 0.0);
    float NdotH2 = NdotH * NdotH;

    float num = a2;
    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    denom = PI * denom * denom;

    return num / denom;
}

float GeometrySchlickGGX(float NdotV, float roughness)
{
    float r = (roughness + 1.0);
    float k = (r*r) / 8.0;

    float nom = NdotV;
    float denom = NdotV * (1.0 - k) + k;

    return nom / denom;
}

float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness)
{
    float NdotV = max(dot(N, V), 0.0);
    float NdotL = max(dot(N, L), 0.0);
    float ggx2 = GeometrySchlickGGX(NdotV, roughness);
    float ggx1 = GeometrySchlickGGX(NdotL, roughness);

    return ggx1 * ggx2;
}

vec3 FresnelSchlick(float cosTheta, vec3 F0)
{
    //return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
    return F0 + (1.0 - F0) * pow(max(1.0 - cosTheta, 0.0), 5.0);
}

// todo: these will be ins
in float oe_roughness;
in float oe_ao;
const float oe_metallic = 0.0;

void atmos_fragment_main_pbr(inout vec4 color)
{
#ifndef OE_LIGHTING
    return;
#endif

    vec3 albedo = color.rgb;

    vec3 N = normalize(vp_Normal);
    vec3 V = normalize(-atmos_vert);

    vec3 F0 = vec3(0.04);
    F0 = mix(F0, albedo, vec3(oe_metallic));

    vec3 Lo = vec3(0.0);
    float contrast = 0.0;
    for (int i = 0; i < OE_NUM_LIGHTS; ++i)
    {
        // per-light radiance:
        vec3 L = normalize(osg_LightSource[i].position.xyz - atmos_vert);
        vec3 H = normalize(V + L);
        //float distance = length(osg_LightSource[i].position.xyz - atmos_vert);
        //float attenuation = 1.0 / (distance * distance);
        vec3 radiance = vec3(1.0); // osg_LightSource[i].diffuse.rgb * attenuation;

        radiance *= atmos_atten;

        // cook-torrance BRDF:
        float NDF = DistributionGGX(N, H, oe_roughness);
        float G = GeometrySmith(N, V, L, oe_roughness);
        vec3 F = FresnelSchlick(max(dot(H, V), 0.0), F0);

        vec3 kS = F;
        vec3 kD = vec3(1.0) - kS;
        kD *= 1.0 - oe_metallic;

        float NdotL = max(dot(N, L), 0.0);

        vec3 numerator = NDF * G * F;
        float denominator = 4.0 * max(dot(N, V), 0.0) * NdotL;
        vec3 specular = numerator / max(denominator, 0.001);

        Lo += (kD * albedo / PI + specular) * radiance * NdotL;

        contrast += oe_sky_contrast * NdotL;
    }

    vec3 ambient = osg_LightSource[0].ambient.rgb * albedo * oe_ao;
    color.rgb = ambient + Lo;

    // tone map:
    color.rgb = color.rgb / (color.rgb + vec3(1.0));

    // gamma correction:
    //color.rgb = pow(color.rgb, vec3(1.0 / 2.2));

    // boost:
    color.rgb *= 2.2;

    // add in the haze
    color.rgb += atmos_color;

    // exposure:
    color.rgb = 1.0 - exp(-oe_sky_exposure * color.rgb);

    // final contrast:
    contrast = clamp(contrast, 1.0, 3.0);
    color.rgb = ((color.rgb - 0.5)*contrast + 0.5);
}
