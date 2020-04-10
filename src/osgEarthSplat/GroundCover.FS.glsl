#version $GLSL_VERSION_STR
#pragma vp_name       Land cover billboard texture application
#pragma vp_entryPoint oe_GroundCover_FS
#pragma vp_location   fragment_coloring

//#pragma import_defines(OE_GROUNDCOVER_HAS_MULTISAMPLES)
#pragma import_defines(OE_IS_SHADOW_CAMERA)
#pragma import_defines(OE_IS_SHADOW_CAMERA)

#pragma import_defines(OE_GROUNDCOVER_COLOR_SAMPLER)
#pragma import_defines(OE_GROUNDCOVER_COLOR_MATRIX)
#ifdef OE_GROUNDCOVER_COLOR_SAMPLER
uniform sampler2D OE_GROUNDCOVER_COLOR_SAMPLER ;
uniform mat4 OE_GROUNDCOVER_COLOR_MATRIX ;
in vec4 oe_layer_tilec;
#endif

uniform sampler2DArray oe_GroundCover_billboardTex;

uniform int oe_GroundCover_instancedModel;

uniform float oe_GroundCover_maxAlpha;

in vec2 oe_GroundCover_texCoord;

flat in float oe_GroundCover_atlasIndex;

void oe_GroundCover_FS(inout vec4 color)
{
    if (oe_GroundCover_atlasIndex < 0.0)
        discard;

    if (oe_GroundCover_instancedModel == 0)
    {
        // modulate the texture
        color *= texture(oe_GroundCover_billboardTex, vec3(oe_GroundCover_texCoord, oe_GroundCover_atlasIndex));
    
        // if multisampling is off, use alpha-discard.
//#if !defined(OE_GROUNDCOVER_HAS_MULTISAMPLES) || defined(OE_IS_SHADOW_CAMERA)
        if (color.a < oe_GroundCover_maxAlpha)
            discard;
//#endif
    }
    else
    {
        color.rgba = vec4(1,.6,.2,1);
    }

#ifdef OE_GROUNDCOVER_COLOR_SAMPLER
    const float modulation = 0.75;
    float mono = (color.r*0.2126 + color.g*0.7152 + color.b*0.0722);
    vec4 mod_color = texture(OE_GROUNDCOVER_COLOR_SAMPLER, (OE_GROUNDCOVER_COLOR_MATRIX*oe_layer_tilec).st);
    color.rgb = mix(color.rgb, mod_color.rgb*vec3(mono)*2.0, modulation);
#endif
}
