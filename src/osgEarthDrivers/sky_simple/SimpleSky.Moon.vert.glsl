#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

out vec4 moon_TexCoord;

void main() 
{ 
    moon_TexCoord = gl_MultiTexCoord0; 
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; 
}
