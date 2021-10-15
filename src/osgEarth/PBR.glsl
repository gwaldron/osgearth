#version 330
#pragma vp_function oe_pbr_init, fragment, first

// fragment stage global PBR parameters.
float oe_roughness;
float oe_ao;
float oe_metal;
float oe_brightness;
float oe_contrast;

void oe_pbr_init(inout vec4 ignore_me)
{
    oe_roughness = 1.0;
    oe_ao = 1.0;
    oe_metal = 0.0;
    oe_brightness = 1.0;
    oe_contrast = 1.0;
}
