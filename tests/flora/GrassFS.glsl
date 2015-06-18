#version 330
            
#pragma vp_entryPoint "oe_grass_fragment"
#pragma vp_location   "fragment_coloring"
                     
uniform bool      oe_terrain_hasMultiSamples;
uniform sampler2D oe_layer_tex;

uniform sampler2D oe_grass_tex;

in vec4 vp_Color;

in vec2 oe_grass_texCoord;
in vec4 oe_layer_texc;

            
void oe_grass_fragment(inout vec4 color)
{    
    // modulate the grass texture
    color = texture(oe_grass_tex, oe_grass_texCoord) * vp_Color;
    
    // if multisampling is off, use alpha-discard.
    if ( !oe_terrain_hasMultiSamples && color.a < 0.2 )
        discard;
        
    // sample and mix in the green channel of the main color texture
    float green = texture(oe_layer_tex, oe_layer_texc.st).g;
    color.g = clamp( color.g*(1.0+green), 0.0, 1.0);
}
