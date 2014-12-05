#version $GLSL_VERSION_STR 
$GLSL_DEFAULT_PRECISION_FLOAT 

uniform float atmos_sunAlpha; 
varying vec3 atmos_v3Direction; 

float atmos_fastpow(in float x, in float y) 
{ 
    return x/(x+y-y*x); 
} 

void main( void ) 
{ 
   float fCos = -atmos_v3Direction[2];          
   float fMiePhase = 0.050387596899224826 * (1.0 + fCos*fCos) / atmos_fastpow(1.9024999999999999 - -1.8999999999999999*fCos, 1.5); 
   gl_FragColor.rgb = fMiePhase*vec3(.3,.3,.2); 
   gl_FragColor.a = atmos_sunAlpha*gl_FragColor.r; 
}
