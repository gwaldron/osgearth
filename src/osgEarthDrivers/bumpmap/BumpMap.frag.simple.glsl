#version $GLSL_VERSION_STR

#pragma vp_entryPoint oe_bumpmap_fragment
#pragma vp_location   fragment_coloring
#pragma vp_order      0.3

#pragma include BumpMap.frag.common.glsl

in vec3 vp_Normal;
in vec2 oe_bumpmap_coords;
flat in mat3 oe_bumpmap_normalMatrix;
in vec3 oe_UpVectorView;

uniform sampler2D oe_bumpmap_tex;
uniform float     oe_bumpmap_intensity;
uniform float     oe_bumpmap_slopeFactor;


void oe_bumpmap_fragment(inout vec4 color)
{
	// sample the bump map
    vec3 bump = oe_bumpmap_normalMatrix * normalize(texture2D(oe_bumpmap_tex, oe_bumpmap_coords).xyz*2.0-1.0);
    
    // calculate slope from normal:
    float slope = clamp( (1.0-dot(oe_UpVectorView, vp_Normal))*oe_bumpmap_slopeFactor, 0.0, 1.0);
    
	// permute the normal with the bump.
	vp_Normal = normalize(vp_Normal + bump*oe_bumpmap_intensity*slope);
}
