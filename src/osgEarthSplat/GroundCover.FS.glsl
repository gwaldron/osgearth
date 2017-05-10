#version 330
#pragma vp_name       Land cover billboard texture application
#pragma vp_entryPoint oe_GroundCover_fragment
#pragma vp_location   fragment_coloring

#define IS_ARRAY
                     
uniform bool oe_terrain_hasMultiSamples;

#ifdef IS_ARRAY

uniform sampler2DArray oe_GroundCover_texArray;
uniform float oe_GroundCover_exposure;
in vec2 oe_GroundCover_texCoord;
in vec4 oe_layer_tilec;

flat in float oe_GroundCover_arrayIndex; // from GroundCover.GS.glsl

void oe_GroundCover_fragment(inout vec4 color)
{    
    // modulate the texture
    color = texture(oe_GroundCover_texArray, vec3(oe_GroundCover_texCoord, oe_GroundCover_arrayIndex)) * color;
    color.rgb *= oe_GroundCover_exposure;
    
    // if multisampling is off, use alpha-discard.
    if ( !oe_terrain_hasMultiSamples && color.a < 0.15 )
        discard;
}

#else

uniform sampler2D oe_GroundCover_tex;
uniform float oe_GroundCover_exposure;

in vec2 oe_GroundCover_texCoord;
in vec4 oe_layer_tilec;

void oe_GroundCover_fragment(inout vec4 color)
{    
    // modulate the texture
    color = texture(oe_GroundCover_tex, oe_GroundCover_texCoord) * color;
    color.rgb *= oe_GroundCover_exposure;
    
    // if multisampling is off, use alpha-discard.
    if ( !oe_terrain_hasMultiSamples && color.a < 0.15 )
        discard;
}

#endif