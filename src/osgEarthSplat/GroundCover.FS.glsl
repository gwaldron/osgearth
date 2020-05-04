#version $GLSL_VERSION_STR
#pragma vp_name       Land cover billboard texture application
#pragma vp_entryPoint oe_GroundCover_FS
#pragma vp_location   fragment_coloring

#pragma import_defines(OE_IS_SHADOW_CAMERA)

uniform sampler2DArray oe_GroundCover_billboardTex;

uniform int oe_GroundCover_instancedModel;

uniform float oe_GroundCover_maxAlpha;
uniform int oe_GroundCover_A2C;

in vec2 oe_GroundCover_texCoord;
in float oe_layer_opacity;

flat in float oe_GroundCover_atlasIndex;

void oe_GroundCover_FS(inout vec4 color)
{
    if (oe_GroundCover_atlasIndex < 0.0)
        discard;

    color.a *= oe_layer_opacity;

    // modulate the texture
    color *= texture(oe_GroundCover_billboardTex, vec3(oe_GroundCover_texCoord, oe_GroundCover_atlasIndex));
    
#ifdef OE_IS_SHADOW_CAMERA
    //if (color.a < oe_GroundCover_maxAlpha)
    //{
    //    discard;
    //}
#else
    if (oe_GroundCover_A2C == 0 && color.a < oe_GroundCover_maxAlpha)
    {
        discard;
    }
#endif
}
