#version $GLSL_VERSION_STR

uniform vec3 atmos_v3LightDir; 
uniform mat4 osg_ViewMatrixInverse; 
out float visibility; 
out vec4 osg_FrontColor; 

float remap( float val, float vmin, float vmax, float r0, float r1 ) 
{ 
    float vr = (clamp(val, vmin, vmax)-vmin)/(vmax-vmin); 
    return r0 + vr * (r1-r0); 
} 

void main() 
{ 
    osg_FrontColor = gl_Color; 
    gl_PointSize = gl_Color.r * 14.0; 
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; 
    vec3 eye = osg_ViewMatrixInverse[3].xyz; 
    float hae = length(eye) - 6378137.0; 
    // highness: visibility increases with altitude
    float highness = remap( hae, 25000.0, 150000.0, 0.0, 1.0 ); 
    eye = normalize(eye); 
    // darkness: visibility increase as the sun goes around the other side of the earth
    float darkness = 1.0-remap(dot(eye,atmos_v3LightDir), -0.25, 0.0, 0.0, 1.0); 
    visibility = clamp(highness + darkness, 0.0, 1.0); 
}
