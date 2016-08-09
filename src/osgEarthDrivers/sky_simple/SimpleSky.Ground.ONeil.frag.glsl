#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint atmos_fragment_main
#pragma vp_location   fragment_lighting
#pragma vp_order      0.8

uniform bool oe_mode_GL_LIGHTING; 
uniform float atmos_exposure;   // scene exposure (ground level)
varying vec3 atmos_lightDir;    // light direction (view coords)
varying vec3 atmos_color;       // atmospheric lighting color
varying vec3 atmos_atten;       // atmospheric lighting attentuation factor
varying vec3 atmos_up;          // earth up vector at fragment (in view coords)
varying float atmos_space;      // camera altitude (0=ground, 1=atmos outer radius)
varying vec3 atmos_vert; 
        
vec3 vp_Normal;          // surface normal (from osgEarth)


// Toatl number of lights in the scene
#define MAX_LIGHTS 8
uniform int osg_NumLights;

// Parameters of each light:
struct osg_LightSourceParameters 
{   
   vec4 ambient;              // Aclarri   
   vec4 diffuse;              // Dcli   
   vec4 specular;             // Scli   
   vec4 position;             // Ppli   
   //vec4 halfVector;           // Derived: Hi   
   vec3 spotDirection;        // Sdli   
   float spotExponent;        // Srli   
   float spotCutoff;          // Crli                              
                              // (range: [0.0,90.0], 180.0)   
   float spotCosCutoff;       // Derived: cos(Crli)                 
                              // (range: [1.0,0.0],-1.0)   
   float constantAttenuation; // K0   
   float linearAttenuation;   // K1   
   float quadraticAttenuation;// K2  

   bool enabled;
};  
uniform osg_LightSourceParameters osg_LightSource[MAX_LIGHTS];

// Surface material:
struct osg_MaterialParameters  
{   
   vec4 emission;    // Ecm   
   vec4 ambient;     // Acm   
   vec4 diffuse;     // Dcm   
   vec4 specular;    // Scm   
   float shininess;  // Srm  
};  
uniform osg_MaterialParameters osg_FrontMaterial; 



void atmos_fragment_main(inout vec4 color) 
{ 
    if ( oe_mode_GL_LIGHTING == false )
    {
        return; 
    }

    vec3 totalLighting = osg_FrontMaterial.emission.rgb;

    if (osg_LightSource[0].enabled)
    {
        vec3 ambient = osg_LightSource[0].ambient.rgb;
        float minAmbient = ambient.r;

        vec3 N = normalize(vp_Normal); 
        vec3 L = normalize(atmos_lightDir);
        vec3 U = normalize(atmos_up); 

        const float maxAmbient = 0.5;
        float daytime = max(0.0, dot(U,L));
        float ambientLightLevel = clamp(daytime, minAmbient, maxAmbient);

        float NdotL = max(dot(N,L), 0.0);

        const float lowAlt  = 1.0;
        const float highAlt = 14.0;
        float altitudeInfluence = 1.0 - clamp( (atmos_space-lowAlt)/(highAlt-lowAlt), 0.0, 1.0);
        float useNormals = altitudeInfluence * (1.0-ambientLightLevel);

        // try to brighten up surfaces the sun is shining on
        float overExposure = 1.0;

        // calculate the base scene color. Skip ambience since we'll be
        // factoring that in later.
        vec4 sceneColor = mix(color*overExposure, color*NdotL, useNormals);

        if (NdotL > 0.0 ) { 
            vec3 V = normalize(atmos_vert); 
            vec3 H = reflect(-L, N);
            float HdotN = max(dot(H,N), 0.0); 
            float shine = clamp(osg_FrontMaterial.shininess, 1.0, 128.0); 
            sceneColor += osg_LightSource[0].specular * osg_FrontMaterial.specular * pow(HdotN, shine); 
        } 

        // clamp the attentuation to the minimum ambient lighting:
        vec3 attenuation = max(atmos_atten, ambient); 

        // ramp exposure from ground (full) to space (50%).
        float exposure = atmos_exposure*clamp(1.0-atmos_space, 0.5, 1.0); 

        vec3 atmosColor = 1.0 - exp(-exposure * (atmos_color + sceneColor.rgb * attenuation));

        totalLighting += atmosColor;
    }

    //color.rgb += totalLighting; //*= totalLighting;
    color.rgb = totalLighting;
}
