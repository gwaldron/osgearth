#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint "atmos_fragment_main"
#pragma vp_location   "fragment_lighting"

uniform bool oe_mode_GL_LIGHTING; 
uniform float atmos_exposure;   // scene exposure (ground level)
varying vec3 atmos_lightDir;    // light direction (view coords)
varying vec3 atmos_color;       // atmospheric lighting color
varying vec3 atmos_atten;       // atmospheric lighting attentuation factor
varying vec3 atmos_up;          // earth up vector at fragment (in view coords)
varying float atmos_space;      // camera altitude (0=ground, 1=atmos outer radius)
varying vec3 atmos_vert; 
        
vec3 oe_global_Normal;          // surface normal (from osgEarth)

void atmos_fragment_main(inout vec4 color) 
{ 
    if ( oe_mode_GL_LIGHTING == false )
    {
        return; 
    }

    vec3 ambient = gl_LightSource[0].ambient.rgb;

    vec3 N = normalize(oe_global_Normal); 
    vec3 L = normalize(gl_LightSource[0].position.xyz); 
    vec3 U = normalize(atmos_up); 

    float NdotL = max(dot(N,L), 0.0); 
    float NdotLnormalized = (dot(N,L)+1.0)*0.5;
    NdotL = mix(NdotL, NdotLnormalized, ambient.r); 

    // Calculate how much normal-based shading should be applied.
    // Between low and high (which are NdotL's) we phase out the
    // normal-based shading (since the atmospheric scattering will
    // take over at that point). This provides a nice dawn or dusk
    // shining effect on the sides of buildings or terrain.
    // Adjust the low: Higher value will increase the effect of
    // normal-based shading at shallower sun angles (i.e. it will
    // become more prominent. Perhaps it makes sense in the future to
    // make this a scattering effect uniform or something -gw)
    const float low = 0.5; 
    const float high = 0.9; 
    float UdotL = clamp(dot(U,L), low, high); 
    float shadeFactor = 1.0 - (UdotL-low)/(high-low); 

    // start applying normal-based shading when we're at twice the 
    // altitude of the atmosphere's outer radius:
    float normFactor = max(0.0, 1.0-(2.0*atmos_space)); 

    // try to brighten up surfaces the sun is shining on
    float overExposure = 1.0;  //1.0+(max(NdotL-0.5, 0.0)*0.25);

    // calculate the base scene color. Skip ambience since we'll be
    // factoring that in later.
    vec4 sceneColor = mix(color*overExposure, color*NdotL, normFactor*shadeFactor); 

    if (NdotL > 0.0 ) { 
        vec3 V = normalize(atmos_vert); 
        vec3 H = normalize(L-V); 
        float HdotN = max(dot(H,N), 0.0); 
        float shine = clamp(gl_FrontMaterial.shininess, 1.0, 128.0); 
        sceneColor += gl_FrontLightProduct[0].specular * pow(HdotN, shine); 
    } 

    // clamp the attentuation to the minimum ambient lighting:
    vec3 attenuation = max(atmos_atten, ambient); 

    // ramp exposure from ground (full) to space (50%).
    float exposure = atmos_exposure*clamp(1.0-atmos_space, 0.5, 1.0); 

    vec3 atmosColor = 1.0 - exp(-exposure * (atmos_color + sceneColor.rgb * attenuation)); 
    color.rgb = gl_FrontMaterial.emission.rgb + atmosColor; 
}
