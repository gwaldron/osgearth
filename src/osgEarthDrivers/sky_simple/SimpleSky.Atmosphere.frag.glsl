#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint atmos_fragment_main
#pragma vp_location   fragment_coloring
#pragma vp_order      0.8

uniform vec3 atmos_v3LightDir; 

uniform float atmos_g; 				
uniform float atmos_g2; 
uniform float atmos_fWeather; 

varying vec3 atmos_v3Direction; 	
varying vec3 atmos_mieColor; 
varying vec3 atmos_rayleighColor; 

const float fExposure = 4.0; 

float atmos_fastpow(in float x, in float y) 
{ 
    return x/(x+y-y*x); 
} 

void atmos_fragment_main(inout vec4 color) 
{ 				
    float fCos = dot(atmos_v3LightDir, atmos_v3Direction) / length(atmos_v3Direction); 
    float fRayleighPhase = 1.0;  // 0.75 * (1.0 + fCos*fCos); 
    float fMiePhase = 1.5 * ((1.0 - atmos_g2) / (2.0 + atmos_g2)) * (1.0 + fCos*fCos) / atmos_fastpow(1.0 + atmos_g2 - 2.0*atmos_g*fCos, 1.5); 
    vec3 f4Color = fRayleighPhase * atmos_rayleighColor + fMiePhase * atmos_mieColor; 
    vec3 skyColor = 1.0 - exp(f4Color * -fExposure); 
    color.rgb = skyColor.rgb*atmos_fWeather; 
    color.a = (skyColor.r+skyColor.g+skyColor.b) * 2.0; 
}
