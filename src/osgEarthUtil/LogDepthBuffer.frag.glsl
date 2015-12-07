#version 110

#pragma vp_entryPoint oe_logDepth_frag
#pragma vp_location   fragment_lighting
#pragma vp_order      0.99

uniform float oe_logDepth_FC;
varying float oe_logDepth_clipz;

void oe_logDepth_frag(inout vec4 color)
{
    if ( oe_logDepth_FC > 0.0 )
    {
        gl_FragDepth = log2(oe_logDepth_clipz) * 0.5*oe_logDepth_FC;
    }
}
