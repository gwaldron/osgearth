#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint "oe_logDepth_frag"
#pragma vp_location   "fragment_lighting"
#pragma vp_order      "FLT_MAX"

uniform float oe_logDepth_FC;
varying float oe_logDepth_logz;

void oe_logDepth_frag(inout vec4 color)
{
    gl_FragDepth = log2(oe_logDepth_logz)*0.5*oe_logDepth_FC;
}