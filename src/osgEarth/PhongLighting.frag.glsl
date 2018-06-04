#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name       Phong Lighting Vertex Stage
#pragma vp_entryPoint oe_phong_fragment
#pragma vp_location   fragment_lighting

#pragma import_defines(OE_LIGHTING)
#pragma import_defines(OE_NUM_LIGHTS)

#ifdef OE_LIGHTING

in vec3 oe_phong_vertexView3; 

// stage global
vec3 vp_Normal;

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


void oe_phong_fragment(inout vec4 color) 
{
    // See:
    // https://en.wikipedia.org/wiki/Phong_reflection_model
    // https://www.opengl.org/sdk/docs/tutorials/ClockworkCoders/lighting.php
    // https://en.wikibooks.org/wiki/GLSL_Programming/GLUT/Multiple_Lights

    vec3 N = normalize(vp_Normal);

    float shine = clamp(osg_FrontMaterial.shininess, 1.0, 128.0); 

    // Accumulate the lighting, starting with material emission. We are currently
    // omitting the ambient term for now since we are not using LightModel ambience.
    vec3 totalLighting =
        osg_FrontMaterial.emission.rgb;
        // + osg_FrontMaterial.ambient.rgb * osg_LightModel.ambient.rgb;
    
    int numLights = OE_NUM_LIGHTS; //min(osg_NumLights, MAX_LIGHTS);

    for (int i=0; i<numLights; ++i)
    {
        const float attenuation = 1.0;

        if (osg_LightSource[i].enabled)
        {
            float attenuation = 1.0;
            vec3 L; // vertex-to-light-source vector.

            // directional light:
            if (osg_LightSource[i].position.w == 0.0)
            {
                L = normalize(osg_LightSource[i].position.xyz);
            }

            // point or spot light:
            else
            {
                // calculate VL, the vertex-to-light vector:
                vec4 V = vec4(oe_phong_vertexView3, 1.0) * osg_LightSource[i].position.w;
                vec4 VL4 = osg_LightSource[i].position - V;
                L = normalize(VL4.xyz);

                // calculate attenuation:
                float distance = length(VL4);
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

            vec3 ambientReflection =
                attenuation
                * osg_FrontMaterial.ambient.rgb
                * osg_LightSource[i].ambient.rgb;

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

#else

// nop
void oe_phong_fragment(inout vec4 color) { }

#endif