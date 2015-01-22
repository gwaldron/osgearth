#version 110

vec3 oe_global_Normal;

uniform sampler2D oe_bumpmap_tex;
uniform float oe_bumpmap_intensity;
in vec2 oe_bumpmap_coords;
in float oe_bumpmap_slope;

void oe_bumpmap_fragment(inout vec4 color)
{
	// sample the bump map
    vec3 bump = gl_NormalMatrix * normalize(texture2D(oe_bumpmap_tex, oe_bumpmap_coords).xyz*2.0-1.0);

	// permute the normal with the bump.
	oe_global_Normal = normalize(oe_global_Normal + bump*oe_bumpmap_intensity*oe_bumpmap_slope);
}
