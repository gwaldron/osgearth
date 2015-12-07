#version $GLSL_VERSION_STR 
$GLSL_DEFAULT_PRECISION_FLOAT 

varying float visibility; 
varying vec4 osg_FrontColor; 
void main( void ) 
{ 
    gl_FragColor = osg_FrontColor * visibility; 
}
