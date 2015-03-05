#version 110

#pragma vp_entryPoint "oe_bumpmap_fragment"
#pragma vp_location   "fragment_coloring"
#pragma vp_order      "0.3"

#pragma include "BumpMap.frag.common.glsl"

uniform sampler2D oe_bumpmap_tex;
uniform float oe_bumpmap_intensity;
uniform int oe_bumpmap_octaves;
uniform float oe_bumpmap_maxRange;

// stage global
vec3 oe_global_Normal;

// from BumpMap.model.vert.glsl
in vec2 oe_bumpmap_coords;

// from BumpMap.view.vert.glsl
in float oe_bumpmap_range;

// Entry point for progressive blended bump maps
void oe_bumpmap_fragment(inout vec4 color)
{
	// sample the bump map
    const float amplitudeDecay = 1.0; // no decay.
    float maxLOD = float(oe_bumpmap_octaves)+1.0;

    // starter vector:
    vec3 bump = vec3(0.0);    
    float scale = 1.0;
    float amplitude = 1.0;
    float limit = oe_bumpmap_range;
    float range = oe_bumpmap_maxRange;
    float lastRange = oe_bumpmap_maxRange;
    for(float lod = 1.0; lod < maxLOD; lod += 1.0, scale *= 2.0, amplitude *= amplitudeDecay)
    {
        float fadeIn = 1.0;
        if ( range <= limit && limit < oe_bumpmap_maxRange )
            fadeIn = clamp((lastRange-limit)/(lastRange-range), 0.0, 1.0);
        bump += (texture2D(oe_bumpmap_tex, oe_bumpmap_coords*scale).xyz*2.0-1.0)*amplitude*fadeIn;
        if ( range <= limit )
            break;
        lastRange = range;
        range = oe_bumpmap_maxRange/exp(lod);
    }

    // finally, transform into view space and normalize the vector.
    bump = normalize(gl_NormalMatrix*bump);

    float slope = oe_bumpmap_getSlope();

	// permute the normal with the bump.
	oe_global_Normal = normalize(oe_global_Normal + bump*oe_bumpmap_intensity*slope);
}
