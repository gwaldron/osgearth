#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint "oe_logDepth_vert"
#pragma vp_location   "vertex_clip"
#pragma vp_order      "FLT_MAX"

uniform float oe_logDepth_C;
uniform float oe_logDepth_FC;
varying float oe_logDepth_logz;

void oe_logDepth_vert(inout vec4 clip)
{
    oe_logDepth_logz = max(1e-6, clip.w*oe_logDepth_C + 1.0);
    clip.z = log2(oe_logDepth_logz)*oe_logDepth_FC - 1.0;
}
