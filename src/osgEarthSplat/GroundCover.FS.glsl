#version $GLSL_VERSION_STR
#pragma vp_name       Land cover billboard texture application
#pragma vp_entryPoint oe_GroundCover_FS
#pragma vp_location   fragment_coloring

uniform sampler2DArray oe_GroundCover_billboardTex;

uniform int oe_GroundCover_instancedModel;

uniform float oe_GroundCover_maxAlpha;
uniform int oe_GroundCover_A2C;

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
    
#ifdef OE_IS_SHADOW_CAMERA
        if (color.a < oe_GroundCover_maxAlpha)
        {
            discard;
        }
#else
        if (oe_GroundCover_A2C == 1)
        {
            // https://medium.com/@bgolus/anti-aliased-alpha-test-the-esoteric-alpha-to-coverage-8b177335ae4f
            // good for distant trees, but bad for top/side url blending - comment out for now
            //color.a = (color.a - oe_GroundCover_maxAlpha) / max(fwidth(color.a), 0.0001) + 0.5;
        }
        else if (color.a < oe_GroundCover_maxAlpha)
        {
            discard;
        }
#endif
    }
    else
    {
        color.rgba = vec4(1,.6,.2,1);
    }
}
