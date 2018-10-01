#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

in float visibility; 
in vec4 osg_FrontColor; 

out vec4 out_FragColor;

void main( void ) 
{ 
    out_FragColor = osg_FrontColor * visibility; 
}
