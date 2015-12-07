#version 330
#pragma vp_name       Land cover billboard texture application
#pragma vp_entryPoint oe_landcover_fragment
#pragma vp_location   fragment_coloring

#define IS_ARRAY
                     
uniform bool oe_terrain_hasMultiSamples;

#ifdef IS_ARRAY

uniform sampler2DArray oe_landcover_texArray;
uniform float oe_landcover_exposure;
in vec2 oe_landcover_texCoord;
in vec4 oe_layer_tilec;

flat in float oe_landcover_arrayIndex; // from LandCover.GS.glsl

void oe_landcover_fragment(inout vec4 color)
{    
    // modulate the texture
    color = texture(oe_landcover_texArray, vec3(oe_landcover_texCoord, oe_landcover_arrayIndex)) * color;
    color.rgb *= oe_landcover_exposure;
    
    // if multisampling is off, use alpha-discard.
    if ( !oe_terrain_hasMultiSamples && color.a < 0.15 )
        discard;
}

#else

uniform sampler2D oe_landcover_tex;
uniform float oe_landcover_exposure;

in vec2 oe_landcover_texCoord;
in vec4 oe_layer_tilec;

void oe_landcover_fragment(inout vec4 color)
{    
    // modulate the texture
    color = texture(oe_landcover_tex, oe_landcover_texCoord) * color;
    color.rgb *= oe_landcover_exposure;
    
    // if multisampling is off, use alpha-discard.
    if ( !oe_terrain_hasMultiSamples && color.a < 0.15 )
        discard;
}

#endif