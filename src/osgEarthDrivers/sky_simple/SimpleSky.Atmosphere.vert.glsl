#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint atmos_vertex_main
#pragma vp_location   vertex_view
#pragma vp_order      0.5

// Atmospheric Scattering and Sun Shaders
// Adapted from code that is Copyright (c) 2004 Sean ONeil

uniform mat4 osg_ViewMatrixInverse;   // camera position in [3].xyz
uniform vec3 atmos_v3LightDir;        // The direction vector to the light source 
uniform vec3 atmos_v3InvWavelength;   // 1 / pow(wavelength,4) for the rgb channels 
uniform float atmos_fOuterRadius;     // Outer atmosphere radius 
uniform float atmos_fOuterRadius2;    // fOuterRadius^2 		
uniform float atmos_fInnerRadius;     // Inner planetary radius 
uniform float atmos_fInnerRadius2;    // fInnerRadius^2 
uniform float atmos_fKrESun;          // Kr * ESun 	
uniform float atmos_fKmESun;          // Km * ESun 		
uniform float atmos_fKr4PI;           // Kr * 4 * PI 	
uniform float atmos_fKm4PI;           // Km * 4 * PI 		
uniform float atmos_fScale;           // 1 / (fOuterRadius - fInnerRadius) 	
uniform float atmos_fScaleDepth;      // The scale depth 
uniform float atmos_fScaleOverScaleDepth;     // fScale / fScaleDepth 	
uniform int atmos_nSamples; 	
uniform float atmos_fSamples; 				

varying vec3 atmos_v3Direction; 
varying vec3 atmos_mieColor; 
varying vec3 atmos_rayleighColor; 

vec3 vVec; 
float atmos_fCameraHeight;    // The camera's current height 		
float atmos_fCameraHeight2;   // fCameraHeight^2 

float atmos_fastpow(in float x, in float y) 
{ 
    return x/(x+y-y*x); 
} 

float atmos_scale(float fCos) 	
{ 
    float x = 1.0 - fCos; 
    return atmos_fScaleDepth * exp(-0.00287 + x*(0.459 + x*(3.83 + x*(-6.80 + x*5.25)))); 
} 

void atmos_SkyFromSpace(void) 
{ 
    // Get the ray from the camera to the vertex and its length (which is the far point of the ray passing through the atmosphere) 
    vec3 v3Pos = gl_Vertex.xyz; 
    vec3 v3Ray = v3Pos - vVec; 
    float fFar = length(v3Ray); 
    v3Ray /= fFar; 

    // Calculate the closest intersection of the ray with the outer atmosphere 
    // (which is the near point of the ray passing through the atmosphere) 
    float B = 2.0 * dot(vVec, v3Ray); 
    float C = atmos_fCameraHeight2 - atmos_fOuterRadius2; 
    float fDet = max(0.0, B*B - 4.0 * C); 	
    float fNear = 0.5 * (-B - sqrt(fDet)); 		

    // Calculate the ray's starting position, then calculate its scattering offset 
    vec3 v3Start = vVec + v3Ray * fNear; 			
    fFar -= fNear; 	
    float fStartAngle = dot(v3Ray, v3Start) / atmos_fOuterRadius; 			
    float fStartDepth = exp(-1.0 / atmos_fScaleDepth); 
    float fStartOffset = fStartDepth*atmos_scale(fStartAngle); 		

    // Initialize the atmos_ing loop variables 	
    float fSampleLength = fFar / atmos_fSamples; 		
    float fScaledLength = fSampleLength * atmos_fScale; 					
    vec3 v3SampleRay = v3Ray * fSampleLength; 	
    vec3 v3SamplePoint = v3Start + v3SampleRay * 0.5; 	

    // Now loop through the sample rays 
    vec3 v3FrontColor = vec3(0.0, 0.0, 0.0); 
    vec3 v3Attenuate;   
    for(int i=0; i<atmos_nSamples; i++) 		
    { 
        float fHeight = length(v3SamplePoint); 			
        float fDepth = exp(atmos_fScaleOverScaleDepth * (atmos_fInnerRadius - fHeight)); 
        float fLightAngle = dot(atmos_v3LightDir, v3SamplePoint) / fHeight; 		
        float fCameraAngle = dot(v3Ray, v3SamplePoint) / fHeight; 			
        float fscatter = (fStartOffset + fDepth*(atmos_scale(fLightAngle) - atmos_scale(fCameraAngle))); 	
        v3Attenuate = exp(-fscatter * (atmos_v3InvWavelength * atmos_fKr4PI + atmos_fKm4PI)); 	
        v3FrontColor += v3Attenuate * (fDepth * fScaledLength); 					
        v3SamplePoint += v3SampleRay; 		
    } 		

    // Finally, scale the Mie and Rayleigh colors and set up the varying 			
    // variables for the pixel shader 	
    atmos_mieColor      = v3FrontColor * atmos_fKmESun; 				
    atmos_rayleighColor = v3FrontColor * (atmos_v3InvWavelength * atmos_fKrESun); 						
    atmos_v3Direction = vVec  - v3Pos; 			
} 		

void atmos_SkyFromAtmosphere(void) 		
{ 
    // Get the ray from the camera to the vertex, and its length (which is the far 
    // point of the ray passing through the atmosphere) 
    vec3 v3Pos = gl_Vertex.xyz; 	
    vec3 v3Ray = v3Pos - vVec; 			
    float fFar = length(v3Ray); 					
    v3Ray /= fFar; 				

    // Calculate the ray's starting position, then calculate its atmos_ing offset 
    vec3 v3Start = vVec; 
    float fHeight = length(v3Start); 		
    float fDepth = exp(atmos_fScaleOverScaleDepth * (atmos_fInnerRadius - atmos_fCameraHeight)); 
    float fStartAngle = dot(v3Ray, v3Start) / fHeight; 	
    float fStartOffset = fDepth*atmos_scale(fStartAngle); 

    // Initialize the atmos_ing loop variables 		
    float fSampleLength = fFar / atmos_fSamples; 			
    float fScaledLength = fSampleLength * atmos_fScale; 				
    vec3 v3SampleRay = v3Ray * fSampleLength; 		
    vec3 v3SamplePoint = v3Start + v3SampleRay * 0.5; 

    // Now loop through the sample rays 		
    vec3 v3FrontColor = vec3(0.0, 0.0, 0.0); 		
    vec3 v3Attenuate;   
    for(int i=0; i<atmos_nSamples; i++) 			
    { 	
        float fHeight = length(v3SamplePoint); 	
        float fDepth = exp(atmos_fScaleOverScaleDepth * (atmos_fInnerRadius - fHeight)); 
        float fLightAngle = dot(atmos_v3LightDir, v3SamplePoint) / fHeight; 
        float fCameraAngle = dot(v3Ray, v3SamplePoint) / fHeight; 	
        float fscatter = (fStartOffset + fDepth*(atmos_scale(fLightAngle) - atmos_scale(fCameraAngle))); 	
        v3Attenuate = exp(-fscatter * (atmos_v3InvWavelength * atmos_fKr4PI + atmos_fKm4PI)); 	
        v3FrontColor += v3Attenuate * (fDepth * fScaledLength); 		
        v3SamplePoint += v3SampleRay; 		
    } 

    // Finally, scale the Mie and Rayleigh colors and set up the varying 
    // variables for the pixel shader 					
    atmos_mieColor      = v3FrontColor * atmos_fKmESun; 			
    atmos_rayleighColor = v3FrontColor * (atmos_v3InvWavelength * atmos_fKrESun); 				
    atmos_v3Direction = vVec - v3Pos; 				
} 

void atmos_vertex_main(inout vec4 VertexVIEW) 
{ 
    // Get camera position and height 
    vVec = osg_ViewMatrixInverse[3].xyz; 
    atmos_fCameraHeight = length(vVec); 
    atmos_fCameraHeight2 = atmos_fCameraHeight*atmos_fCameraHeight; 
    if(atmos_fCameraHeight >= atmos_fOuterRadius)
    { 
        atmos_SkyFromSpace(); 
    } 
    else
    { 
        atmos_SkyFromAtmosphere(); 
    } 
}
