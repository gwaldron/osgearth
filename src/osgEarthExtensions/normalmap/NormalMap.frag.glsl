#version 110

vec3 oe_global_Normal;

uniform sampler2D oe_nmap_tex;
uniform float oe_nmap_intensity;
varying vec2 oe_nmap_coords;
varying float oe_nmap_slope;

void oe_nmap_fragment(inout vec4 color)
{
	// sample the bump map
	vec3 bump = gl_NormalMatrix * normalize(texture2D(oe_nmap_tex, oe_nmap_coords).xyz * 2.0 - 1.0);

	// permute the normal with the bump.
	oe_global_Normal = normalize(oe_global_Normal + bump*oe_nmap_intensity*oe_nmap_slope);
}
