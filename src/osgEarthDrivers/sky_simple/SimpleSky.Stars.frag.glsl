#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

in float visibility; 
in vec4 osg_FrontColor; 

out vec4 out_FragColor;

void main( void ) 
{ 
    float b1 = 1.0-(2.0*abs(gl_PointCoord.s-0.5)); 
    float b2 = 1.0-(2.0*abs(gl_PointCoord.t-0.5)); 
    float i = b1*b1 * b2*b2; 
    out_FragColor = osg_FrontColor * i * visibility; 
}
