#version $GLSL_VERSION_STR

#pragma vp_entryPoint oe_alphaEffect_frag
#pragma vp_location   fragment_coloring
#pragma vp_order      0.5

uniform float oe_alphaEffect_alpha;

void oe_alphaEffect_frag(inout vec4 color)
{
    color.a = color.a * oe_alphaEffect_alpha;
}
