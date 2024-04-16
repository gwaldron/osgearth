#pragma vp_function oe_pbr_init, fragment, first

// fragment stage global PBR parameters.
struct OE_PBR { float displacement, roughness, ao, metal; } oe_pbr;

void oe_pbr_init(inout vec4 ignore_me)
{
    oe_pbr.displacement = 0.5;
    oe_pbr.roughness = 1.0;
    oe_pbr.ao = 1.0;
    oe_pbr.metal = 0.0;
}
