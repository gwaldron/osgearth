#version 330
#pragma vp_name       Land cover billboard texture application
#pragma vp_entryPoint oe_GroundCover_fragment
#pragma vp_location   fragment_coloring

#pragma import_defines(OE_GROUNDCOVER_HAS_MULTISAMPLES)
#pragma import_defines(OE_GROUNDCOVER_SHOW_TESSELLATION)


uniform sampler2DArray oe_GroundCover_billboardTex;
uniform float oe_GroundCover_exposure;
in vec2 oe_GroundCover_texCoord;
in vec4 oe_layer_tilec;

flat in float oe_GroundCover_arrayIndex; // from GroundCover.GS.glsl

void oe_GroundCover_fragment(inout vec4 color)
{    
#ifdef OE_GROUNDCOVER_SHOW_TESSELLATION
    color = vec4(1,1,1,1);
    return;
#endif

    // modulate the texture
    color = texture(oe_GroundCover_billboardTex, vec3(oe_GroundCover_texCoord, oe_GroundCover_arrayIndex)) * color;
    color.rgb *= oe_GroundCover_exposure;
    
    // if multisampling is off, use alpha-discard.
#ifndef OE_GROUNDCOVER_HAS_MULTISAMPLES
    if (color.a < 0.15)
        discard;
#endif
    //if ( !oe_terrain_hasMultiSamples && color.a < 0.15 )
    //    discard;
}
