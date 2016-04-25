#version $GLSL_VERSION_STR
#pragma vp_entryPoint oe_detail_fragment
#pragma vp_location   fragment_coloring
#pragma vp_order      1
                
uniform sampler2D oe_detail_tex; // uniform of detail texture
uniform float oe_detail_alpha;   // The detail textures alpha.
in vec2 detailCoords;            // input from vertex stage
in float detailIntensity;        // The intensity of the detail effect.
                
void oe_detail_fragment(inout vec4 color)
{
    vec4 texel = texture(oe_detail_tex, detailCoords);
    color.rgb = mix(color.rgb, texel.rgb, oe_detail_alpha * detailIntensity);
}                