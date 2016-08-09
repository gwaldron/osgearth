#version $GLSL_VERSION_STR 

#pragma vp_name       Phong Lighting Vertex Stage
#pragma vp_entryPoint oe_phong_fragment
#pragma vp_location   fragment_lighting

uniform bool oe_mode_GL_LIGHTING; 

in vec3 oe_phong_vertexView3; 

// stage global
vec3 vp_Normal;


// Toatl number of lights in the scene
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


void oe_phong_fragment(inout vec4 color) 
{         
    //if ( oe_mode_GL_LIGHTING == false )
    //    return; 

    // See:
    // https://en.wikipedia.org/wiki/Phong_reflection_model
    // https://www.opengl.org/sdk/docs/tutorials/ClockworkCoders/lighting.php

    vec3 N = normalize(vp_Normal);
    vec3 V = normalize(oe_phong_vertexView3); 

    float shine = clamp(osg_FrontMaterial.shininess, 1.0, 128.0); 

    // Accumulate the lighting, starting with material emission. We are currently
    // omitting the ambient term for now since we are not using LightModel ambience.

    vec3 totalLighting =
        osg_FrontMaterial.emission.rgb;
        // + osg_FrontMaterial.ambient.rgb * osg_LightModel.ambient.rgb;

    for (int i=0; i<osg_NumLights; ++i)
    {
        const float attenuation = 1.0;

        if (osg_LightSource[i].enabled)
        {
            vec3 ambientReflection =
                attenuation
                * osg_FrontMaterial.ambient.rgb
                * osg_LightSource[i].ambient.rgb;

            vec3 L = normalize(osg_LightSource[i].position.xyz);
            float NdotL = max(dot(N,L), 0.0); 

            vec3 diffuseReflection =
                attenuation
                * osg_LightSource[i].diffuse.rgb * osg_FrontMaterial.diffuse.rgb
                * NdotL;
                
            vec3 specularReflection = vec3(0.0);
            if (NdotL > 0.0)
            {
                vec3 H = reflect(-L,N); 
                float HdotN = max(dot(H,N), 0.0); 

                specularReflection =
                    attenuation
                    * osg_LightSource[i].specular.rgb
                    * osg_FrontMaterial.specular.rgb
                    * pow(HdotN, shine);
            }

            totalLighting += ambientReflection + diffuseReflection + specularReflection;
        }
    }
    
    color.rgb *= totalLighting;
}
