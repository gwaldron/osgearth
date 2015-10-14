#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint atmos_vertex_main
#pragma vp_location   vertex_view
#pragma vp_order      0.5

uniform bool oe_mode_GL_LIGHTING; 

uniform mat4 osg_ViewMatrixInverse;   // world camera position in [3].xyz 
uniform mat4 osg_ViewMatrix;          // GL view matrix 
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

varying vec3 atmos_color;          // primary sky light color
varying vec3 atmos_atten;          // sky light attenuation factor
varying vec3 atmos_lightDir;       // light direction in view space
        
float atmos_fCameraHeight;            // The camera's current height 		
float atmos_fCameraHeight2;           // fCameraHeight^2 

varying vec3 atmos_up;             // earth up vector at vertex location (not the normal)
varying float atmos_space;         // [0..1]: camera: 0=inner radius (ground); 1.0=outer radius
varying vec3 atmos_vert; 

vec3 vp_Normal;             // surface normal (from osgEarth)

float atmos_scale(float fCos) 	
{ 
    float x = 1.0 - fCos; 
    return atmos_fScaleDepth * exp(-0.00287 + x*(0.459 + x*(3.83 + x*(-6.80 + x*5.25)))); 
} 

void atmos_GroundFromSpace(in vec4 vertexVIEW) 
{ 
    // Get the ray from the camera to the vertex and its length (which is the far point of the ray passing through the atmosphere) 
    vec3 v3Pos = vertexVIEW.xyz; 
    vec3 v3Ray = v3Pos; 
    float fFar = length(v3Ray); 
    v3Ray /= fFar; 
                
    vec4 ec4 = osg_ViewMatrix * vec4(0,0,0,1); 
    vec3 earthCenter = ec4.xyz/ec4.w; 
    vec3 normal = normalize(v3Pos-earthCenter); 
    atmos_up = normal; 

    // Calculate the closest intersection of the ray with the outer atmosphere 
    // (which is the near point of the ray passing through the atmosphere) 
    float B = 2.0 * dot(-earthCenter, v3Ray); 
    float C = atmos_fCameraHeight2 - atmos_fOuterRadius2; 
    float fDet = max(0.0, B*B - 4.0*C); 	
    float fNear = 0.5 * (-B - sqrt(fDet)); 		

    // Calculate the ray's starting position, then calculate its scattering offset 
    vec3 v3Start = v3Ray * fNear; 			
    fFar -= fNear; 
    float fDepth = exp((atmos_fInnerRadius - atmos_fOuterRadius) / atmos_fScaleDepth);
    float fCameraAngle = dot(-v3Ray, normal);  // try max(0, ...) to get rid of yellowing building tops
    float fLightAngle = dot(atmos_lightDir, normal); 
    float fCameraScale = atmos_scale(fCameraAngle); 
    float fLightScale = atmos_scale(fLightAngle); 
    float fCameraOffset = fDepth*fCameraScale; 
    float fTemp = fLightScale * fCameraScale; 		

    // Initialize the scattering loop variables 
    float fSampleLength = fFar / atmos_fSamples; 		
    float fScaledLength = fSampleLength * atmos_fScale; 					
    vec3 v3SampleRay = v3Ray * fSampleLength; 	
    vec3 v3SamplePoint = v3Start + v3SampleRay * 0.5; 	

    // Now loop through the sample rays 
    vec3 v3FrontColor = vec3(0.0, 0.0, 0.0); 
    vec3 v3Attenuate = vec3(1,0,0); 

    for(int i=0; i<atmos_nSamples; ++i) 
    {         
        float fHeight = length(v3SamplePoint-earthCenter); 			
        float fDepth = exp(atmos_fScaleOverScaleDepth * (atmos_fInnerRadius - fHeight)); 
        float fScatter = fDepth*fTemp - fCameraOffset; 
        v3Attenuate = exp(-fScatter * (atmos_v3InvWavelength * atmos_fKr4PI + atmos_fKm4PI)); 	
        v3FrontColor += v3Attenuate * (fDepth * fScaledLength); 					
        v3SamplePoint += v3SampleRay; 		
    } 	

    atmos_color = v3FrontColor * (atmos_v3InvWavelength * atmos_fKrESun + atmos_fKmESun); 
    atmos_atten = v3Attenuate; 
} 		

void atmos_GroundFromAtmosphere(in vec4 vertexVIEW) 		
{ 
    // Get the ray from the camera to the vertex and its length (which is the far point of the ray passing through the atmosphere) 
    vec3 v3Pos = vertexVIEW.xyz / vertexVIEW.w; 
    vec3 v3Ray = v3Pos; 
    float fFar = length(v3Ray); 
    v3Ray /= fFar; 
        
    vec4 ec4 = osg_ViewMatrix * vec4(0,0,0,1); 
    vec3 earthCenter = ec4.xyz/ec4.w; 
    vec3 normal = normalize(v3Pos-earthCenter); 
    atmos_up = normal; 

    // Calculate the ray's starting position, then calculate its scattering offset 
    float fDepth = exp((atmos_fInnerRadius - atmos_fCameraHeight) / atmos_fScaleDepth);
    float fCameraAngle = max(0.0, dot(-v3Ray, normal)); 
    float fLightAngle = dot(atmos_lightDir, normal); 
    float fCameraScale = atmos_scale(fCameraAngle); 
    float fLightScale = atmos_scale(fLightAngle); 
    float fCameraOffset = fDepth*fCameraScale; 
    float fTemp = fLightScale * fCameraScale; 

    // Initialize the scattering loop variables 	
    float fSampleLength = fFar / atmos_fSamples; 		
    float fScaledLength = fSampleLength * atmos_fScale; 					
    vec3 v3SampleRay = v3Ray * fSampleLength; 	
    vec3 v3SamplePoint = v3SampleRay * 0.5; 	

    // Now loop through the sample rays 
    vec3 v3FrontColor = vec3(0.0, 0.0, 0.0); 
    vec3 v3Attenuate;   
    for(int i=0; i<atmos_nSamples; i++) 		
    { 
        float fHeight = length(v3SamplePoint-earthCenter); 			
        float fDepth = exp(atmos_fScaleOverScaleDepth * (atmos_fInnerRadius - fHeight)); 
        float fScatter = fDepth*fTemp - fCameraOffset; 
        v3Attenuate = exp(-fScatter * (atmos_v3InvWavelength * atmos_fKr4PI + atmos_fKm4PI)); 	
        v3FrontColor += v3Attenuate * (fDepth * fScaledLength); 					
        v3SamplePoint += v3SampleRay; 		
    } 		

    atmos_color = v3FrontColor * (atmos_v3InvWavelength * atmos_fKrESun + atmos_fKmESun); 			
    atmos_atten = v3Attenuate; 
} 

void atmos_vertex_main(inout vec4 vertexVIEW) 
{ 
    if ( oe_mode_GL_LIGHTING == false ) return; 

    atmos_fCameraHeight = length(osg_ViewMatrixInverse[3].xyz); 
    atmos_fCameraHeight2 = atmos_fCameraHeight*atmos_fCameraHeight; 
    atmos_lightDir = normalize(gl_LightSource[0].position.xyz);  // view space
    atmos_vert = vertexVIEW.xyz; 

    atmos_space = max(0.0, (atmos_fCameraHeight-atmos_fInnerRadius)/(atmos_fOuterRadius-atmos_fInnerRadius));

    if(atmos_fCameraHeight >= atmos_fOuterRadius) 
    { 
        atmos_GroundFromSpace(vertexVIEW); 
    } 
    else 
    { 
        atmos_GroundFromAtmosphere(vertexVIEW); 
    } 
}
