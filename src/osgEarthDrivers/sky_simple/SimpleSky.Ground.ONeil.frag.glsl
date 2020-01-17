#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint atmos_fragment_main
#pragma vp_location   fragment_lighting
#pragma vp_order      0.8

#pragma import_defines(OE_LIGHTING)
#pragma import_defines(OE_NUM_LIGHTS)

uniform float oe_sky_exposure;           // HDR scene exposure (ground level)
uniform float oe_sky_ambientBoostFactor; // ambient sunlight booster for daytime

in vec3 atmos_lightDir;    // light direction (view coords)
in vec3 atmos_color;       // atmospheric lighting color
in vec3 atmos_atten;       // atmospheric lighting attenuation factor
in vec3 atmos_up;          // earth up vector at fragment (in view coords)
in float atmos_space;      // camera altitude (0=ground, 1=atmos outer radius)
in vec3 atmos_vert; 
        
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
#ifndef OE_LIGHTING
    return;
#endif

    // See:
    // https://en.wikipedia.org/wiki/Phong_reflection_model
    // https://www.opengl.org/sdk/docs/tutorials/ClockworkCoders/lighting.php
    // https://en.wikibooks.org/wiki/GLSL_Programming/GLUT/Multiple_Lights
    // https://en.wikibooks.org/wiki/GLSL_Programming/GLUT/Specular_Highlights

    // normal vector at vertex
    vec3 N = normalize(vp_Normal);

    float shine = clamp(osg_FrontMaterial.shininess, 1.0, 128.0); 
    vec4 surfaceSpecularity = osg_FrontMaterial.specular;
    
    // up vector at vertex
    vec3 U = normalize(atmos_up);

    // Accumulate the lighting for each component separately.
    vec3 totalDiffuse = vec3(0.0);
    vec3 totalAmbient = vec3(0.0);
    vec3 totalSpecular = vec3(0.0);

    int numLights = OE_NUM_LIGHTS;

    for (int i=0; i<numLights; ++i)
    {
        if (osg_LightSource[i].enabled)
        {
            float attenuation = 1.0;

            // L is the normalized camera-to-light vector.
            vec3 L = normalize(osg_LightSource[i].position.xyz);

            // V is the normalized vertex-to-camera vector.
            vec3 V = -normalize(atmos_vert);

            // point or spot light:
            if (osg_LightSource[i].position.w != 0.0)
            {
                // VLu is the unnormalized vertex-to-light vector
                vec3 Lu = osg_LightSource[i].position.xyz - atmos_vert;

                // calculate attenuation:
                float distance = length(Lu);
                attenuation = 1.0 / (
                    osg_LightSource[i].constantAttenuation +
                    osg_LightSource[i].linearAttenuation * distance +
                    osg_LightSource[i].quadraticAttenuation * distance * distance);

                // for a spot light, the attenuation help form the cone:
                if (osg_LightSource[i].spotCutoff <= 90.0)
                {
                    vec3 D = normalize(osg_LightSource[i].spotDirection);
                    float clampedCos = max(0.0, dot(-L,D));
                    attenuation = clampedCos < osg_LightSource[i].spotCosCutoff ?
                        0.0 :
                        attenuation * pow(clampedCos, osg_LightSource[i].spotExponent);
                }
            }

            // a term indicating whether it's daytime for light 0 (the sun).
            float dayTerm = i==0? dot(U,L) : 1.0;

            // This term boosts the ambient lighting for the sun (light 0) when it's daytime.
            // TODO: make the boostFactor a uniform?
            float ambientBoost = i==0? 1.0 + oe_sky_ambientBoostFactor*clamp(2.0*(dayTerm-0.5), 0.0, 1.0) : 1.0;

            vec3 ambientReflection =
                attenuation
                * osg_LightSource[i].ambient.rgb
                * ambientBoost;

            float NdotL = max(dot(N,L), 0.0);

            // this term, applied to light 0 (the sun), attenuates the diffuse light
            // during the nighttime, so that geometry doesn't get lit based on its
            // normals during the night.
            float diffuseAttenuation = clamp(dayTerm+0.35, 0.0, 1.0);
            
            vec3 diffuseReflection =
                attenuation
                * diffuseAttenuation
                * osg_LightSource[i].diffuse.rgb
                * NdotL;
                
            vec3 specularReflection = vec3(0.0);
            if (NdotL > 0.0)
            {
                // prevent a sharp edge where NdotL becomes positive
                // by fading in the spec between (0.0 and 0.1)
                float specAttenuation = clamp(NdotL*10.0, 0.0, 1.0);

                vec3 H = reflect(-L,N);
                float HdotV = max(dot(H,V), 0.0); 

                specularReflection =
                      specAttenuation
                    * attenuation
                    * osg_LightSource[i].specular.rgb
                    * surfaceSpecularity.rgb
                    * pow(HdotV, shine);
            }

            totalDiffuse += diffuseReflection;
            totalAmbient += ambientReflection;
            totalSpecular += specularReflection;
        }
    }
    
    // add the atmosphere color, and incorpoate the lights.
    color.rgb += atmos_color;

    vec3 lightColor =
        osg_FrontMaterial.emission.rgb +
        totalDiffuse * osg_FrontMaterial.diffuse.rgb +
        totalAmbient * osg_FrontMaterial.ambient.rgb;

    color.rgb =
        color.rgb * lightColor +
        totalSpecular; // * osg_FrontMaterial.specular.rgb;
    
    // Simulate HDR by applying an exposure factor (1.0 is none, 2-3 are reasonable)
    color.rgb = 1.0 - exp(-oe_sky_exposure * color.rgb);
}
