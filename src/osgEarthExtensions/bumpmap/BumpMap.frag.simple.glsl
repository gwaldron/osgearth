#version 110

#pragma vp_entryPoint "oe_bumpmap_fragment"
#pragma vp_location   "fragment_coloring"
#pragma vp_order      "0.3"

#pragma include "BumpMap.frag.common.glsl"

vec3 oe_global_Normal;

uniform sampler2D oe_bumpmap_tex;
uniform float oe_bumpmap_intensity;
in vec2 oe_bumpmap_coords;

void oe_bumpmap_fragment(inout vec4 color)
{
	// sample the bump map
    vec3 bump = gl_NormalMatrix * normalize(texture2D(oe_bumpmap_tex, oe_bumpmap_coords).xyz*2.0-1.0);

	// permute the normal with the bump.
    float slope = oe_bumpmap_getSlope();
	oe_global_Normal = normalize(oe_global_Normal + bump*oe_bumpmap_intensity*slope);
}
