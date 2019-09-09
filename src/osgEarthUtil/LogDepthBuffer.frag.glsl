#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_logDepth_frag
#pragma vp_location   fragment_lighting
#pragma vp_order      0.99

in float oe_LogDepth_logz;

void oe_logDepth_frag(inout vec4 color)
{
    gl_FragDepth = oe_LogDepth_logz >= 0? oe_LogDepth_logz : gl_FragCoord.z;
}
