#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint "oe_logDepth_vert"
#pragma vp_location   "vertex_clip"
#pragma vp_order      "FLT_MAX"

uniform float oe_logDepth_C;
uniform float oe_logDepth_FC;

void oe_logDepth_vert(inout vec4 clip)
{
    clip.z = (log2(max(1e-6,oe_logDepth_C*clip.w+1.0))*oe_logDepth_FC - 1.0) * clip.w;
}