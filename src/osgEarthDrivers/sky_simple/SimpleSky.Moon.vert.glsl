#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

uniform vec3 moonToSun;
out vec4 moon_TexCoord;
out float moon_Lighting;

void main() 
{ 
    moon_TexCoord = gl_MultiTexCoord0;
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; 

    // dot product results in a "lighting" factor, 0=none, 1=full,
    // to send to the fragment shader
    moon_Lighting = clamp(dot(gl_Normal, moonToSun), 0, 1);
    moon_Lighting = pow(moon_Lighting, 0.4);
}
