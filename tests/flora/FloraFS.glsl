#version 330
            
#pragma vp_entryPoint "oe_flora_fragment"
#pragma vp_location   "fragment_coloring"
                     
uniform bool      oe_terrain_hasMultiSamples;
//uniform sampler2D floraColor;
//uniform mat4      floraMatrix;

uniform sampler2D oe_flora_tex;
uniform float oe_flora_exposure;

in vec4 vp_Color;

in vec2 oe_flora_texCoord;
in float oe_flora_falloff;
in vec4 oe_layer_tilec;

uniform float mixer;

void oe_flora_fragment(inout vec4 color)
{    
    // modulate the texture
    color = texture(oe_flora_tex, oe_flora_texCoord) * vp_Color;
    color.rgb *= oe_flora_exposure;
    
    // if multisampling is off, use alpha-discard.
    if ( !oe_terrain_hasMultiSamples && color.a < 0.15 )
        discard;
        
    // sample and mix in the green channel of the main color texture
    //vec3 groundColor = texture(floraColor, (floraMatrix*oe_layer_tilec).st).rgb;
    //color.rgb = mix(color.rgb, groundColor, oe_flora_falloff);
}
