#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

varying vec3 atmos_v3Direction; 

void main() 
{ 
    vec3 v3Pos = gl_Vertex.xyz; 
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; 
    atmos_v3Direction = vec3(0.0,0.0,1.0) - v3Pos; 
    atmos_v3Direction = atmos_v3Direction/length(atmos_v3Direction); 
}
