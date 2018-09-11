#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_Stars_FS
#pragma vp_location fragment_coloring

in float oe_Stars_visibility; 

void oe_Stars_FS(inout vec4 color)
{ 
    float b1 = 1.0-(2.0*abs(gl_PointCoord.s-0.5)); 
    float b2 = 1.0-(2.0*abs(gl_PointCoord.t-0.5)); 
    float i = b1*b1 * b2*b2; 
    color = color * i * oe_Stars_visibility;
}
