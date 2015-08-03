#version 330
            
#pragma vp_entryPoint "oe_grass_fragment"
#pragma vp_location   "fragment_coloring"
                     
uniform bool      oe_terrain_hasMultiSamples;
uniform sampler2D imageryColor;
uniform mat4      imageryMatrix;

uniform sampler2D oe_grass_tex;
uniform float oe_grass_exposure;

in vec4 vp_Color;

in vec2 oe_grass_texCoord;
in float oe_grass_falloff;
in vec4 oe_layer_tilec;

uniform float mixer;

void oe_grass_fragment(inout vec4 color)
{    
    // modulate the grass texture
    color = texture(oe_grass_tex, oe_grass_texCoord) * vp_Color;
    color.rgb *= oe_grass_exposure;
    
    // if multisampling is off, use alpha-discard.
    if ( !oe_terrain_hasMultiSamples && color.a < 0.15 )
        discard;
        
    // sample and mix in the green channel of the main color texture
    vec3 groundColor = texture(imageryColor, (imageryMatrix*oe_layer_tilec).st).rgb;
    color.rgb = mix(color.rgb, groundColor, oe_grass_falloff);
}
