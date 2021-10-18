#version 330
#pragma vp_function oe_pbr_init, fragment, first

// fragment stage global PBR parameters.
struct OE_PBR {
    float roughness;
    float ao;
    float metal;
    float brightness;
    float contrast;
} oe_pbr;

void oe_pbr_init(inout vec4 ignore_me)
{
    oe_pbr.roughness = 1.0;
    oe_pbr.ao = 1.0;
    oe_pbr.metal = 0.0;
    oe_pbr.brightness = 1.0;
    oe_pbr.contrast = 1.0;
}
