#version 110

#pragma vp_entryPoint oe_logDepth_vert
#pragma vp_location   vertex_clip
#pragma vp_order      0.99

uniform float oe_logDepth_FC;

void oe_logDepth_vert(inout vec4 clip)
{
    if ( oe_logDepth_FC > 0.0 )
    {
        clip.z = (log2(max(1e-6, clip.w+1.0))*oe_logDepth_FC - 1.0) * clip.w;
    }
}
