#version 110

varying float oe_clamp_alphaFactor;

void oe_clamp_fragment(inout vec4 color)
{
    // adjust the alpha component to "hide" geometry beyond the visible horizon.
    color.a *= oe_clamp_alphaFactor;
}