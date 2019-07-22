#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_Stars_VS
#pragma vp_location vertex_clip

uniform float oe_GL_PointSize;
uniform vec3 atmos_v3LightDir; 
uniform mat4 osg_ViewMatrixInverse; 
out float oe_Stars_visibility; 
vec4 vp_Color;

float remap( float val, float vmin, float vmax, float r0, float r1 ) 
{ 
    float vr = (clamp(val, vmin, vmax)-vmin)/(vmax-vmin); 
    return r0 + vr * (r1-r0); 
} 

void oe_Stars_VS(inout vec4 vertexClip)
{ 
    gl_PointSize = vp_Color.r * oe_GL_PointSize;
    vec3 eye = osg_ViewMatrixInverse[3].xyz; 
    float hae = length(eye) - 6378137.0; 
    // highness: visibility increases with altitude
    float highness = remap( hae, 25000.0, 150000.0, 0.0, 1.0 ); 
    eye = normalize(eye); 
    // darkness: visibility increase as the sun goes around the other side of the earth
    float darkness = 1.0-remap(dot(eye, atmos_v3LightDir), -0.25, 0.0, 0.0, 1.0); 
    oe_Stars_visibility = clamp(highness + darkness, 0.0, 1.0); 

    // clamp stars to the far clip plane to prevent any flickering or precision
    // issues based on the extreme distance.
    vertexClip.z = vertexClip.w;
}

