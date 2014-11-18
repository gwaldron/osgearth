#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

uniform mat4 osg_ModelViewProjectionMatrix;
varying vec4 moon_TexCoord;

void main() 
{ 
    moon_TexCoord = gl_MultiTexCoord0; 
    gl_Position = osg_ModelViewProjectionMatrix * gl_Vertex; 
}
