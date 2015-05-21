#version 110

#pragma vp_entryPoint "oe_logDepth_vert"
#pragma vp_location   "vertex_clip"
#pragma vp_order      "0.99"

uniform float oe_logDepth_C;
uniform float oe_logDepth_FC;
varying float oe_logDepth_logz;

const float C = 0.0005;

void oe_logDepth_vert(inout vec4 clip)
{
    if ( oe_logDepth_FC > 0.0 )
    {
        oe_logDepth_logz = log2(max(1e-6, clip.w*oe_logDepth_C+1.0));
        clip.z = log2(oe_logDepth_logz)*oe_logDepth_FC - 1.0;
    }
}
