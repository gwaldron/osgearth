#version $GLSL_VERSION_STR 

#pragma vp_name       Phong Lighting Vertex Stage
#pragma vp_entryPoint oe_phong_fragment
#pragma vp_location   fragment_lighting

uniform bool oe_mode_GL_LIGHTING; 

in vec3 oe_phong_vertexView3; 

// stage global
vec3 vp_Normal;


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
};  
uniform osg_LightSourceParameters osg_LightSource[];

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
    if ( oe_mode_GL_LIGHTING == false )
        return; 
        
    vec4 ambient = osg_LightSource[0].ambient * osg_FrontMaterial.ambient;

    vec3 L = normalize(osg_LightSource[0].position.xyz);
    vec3 N = normalize(vp_Normal);

    float NdotL = max(dot(N,L), 0.0); 

    vec4 diffuse = osg_LightSource[0].diffuse * osg_FrontMaterial.diffuse * NdotL;
        
    vec4 specular = vec4(0); 
    if (NdotL > 0.0) 
    { 
        vec3 V = normalize(oe_phong_vertexView3); 
        vec3 H = reflect(-L,N); 
        float HdotN = max(dot(H,N), 0.0); 
        float shine = clamp(osg_FrontMaterial.shininess, 1.0, 128.0); 
        specular = osg_LightSource[0].specular * osg_FrontMaterial.specular * pow(HdotN, shine);
    } 
    
    color.rgb *= ambient.rgb + diffuse.rgb + specular.rgb;
}
