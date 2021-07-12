#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint atmos_fragment_main
#pragma vp_location   fragment_coloring
#pragma vp_order      0.8

uniform vec3 atmos_v3LightDir; 

const float atmos_mie_g = -0.095;
const float atmos_mie_g2 = atmos_mie_g * atmos_mie_g;
const float atmos_fWeather = 1.0;

in vec3 atmos_v3Direction; 	
in vec3 atmos_mieColor; 
in vec3 atmos_rayleighColor; 
in float atmos_renderFromSpace;

uniform float oe_sky_exposure;

void atmos_fragment_main(inout vec4 color) 
{
    if (gl_ProjectionMatrix[3][3] != 0.0)
        discard;

    float fCos = dot(atmos_v3LightDir, atmos_v3Direction) / length(atmos_v3Direction); 
    float fRayleighPhase = 1.0;  // 0.75 * (1.0 + fCos*fCos); 
    float fMiePhase = 1.5 * ((1.0 - atmos_mie_g2) / (2.0 + atmos_mie_g2)) * (1.0 + fCos*fCos) / pow(1.0 + atmos_mie_g2 - 2.0*atmos_mie_g*fCos, 1.5);
    vec3 f4Color = fRayleighPhase * atmos_rayleighColor + fMiePhase * atmos_mieColor;
    
    vec3 skyColor = 1.0 - exp(f4Color * -oe_sky_exposure);
    vec4 atmosColor;
    atmosColor.rgb = skyColor.rgb*atmos_fWeather; 
    atmosColor.a = (skyColor.r+skyColor.g+skyColor.b) * 2.0;

    color = mix(atmosColor, vec4(f4Color,1.0), atmos_renderFromSpace);
}
