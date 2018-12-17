#version $GLSL_VERSION_STR

#pragma vp_name       Draping Vertex Shader
#pragma vp_entryPoint oe_Draping_VS
#pragma vp_location   vertex_view

out vec4 oe_Draping_vertexView;

void oe_Draping_VS(inout vec4 vertexView)
{
    oe_Draping_vertexView = vertexView;
}


[break]


#version $GLSL_VERSION_STR

#pragma vp_name       Draping Fragment Shader
#pragma vp_entryPoint oe_Draping_FS
#pragma vp_location   fragment_coloring
#pragma vp_order      0.6

#pragma import_defines(OE_IS_PICK_CAMERA)
#pragma import_defines(OE_DRAPING_MAX_CASCADES)

uniform sampler2DArray oe_Draping_tex;
uniform mat4 oe_Draping_texMatrix[OE_DRAPING_MAX_CASCADES];
in vec4 oe_Draping_vertexView;

void oe_Draping_FS(inout vec4 color)
{
    vec4 texel = vec4(0,0,0,0);

    // the [3][3] slot will be zero if we have exceeded the current number of cascades
    for (int i=0; i<OE_DRAPING_MAX_CASCADES && oe_Draping_texMatrix[i][3][3] != 0; ++i)
    {
        vec4 coord = oe_Draping_texMatrix[i] * oe_Draping_vertexView;
        coord.xy /= coord.w;
        if (clamp(coord.xy, 0, 1) == coord.xy)
        {
            texel = texture(oe_Draping_tex, vec3(coord.xy, i));
            break;
        }
    }

#ifdef OE_IS_PICK_CAMERA
    color = texel;
#else
    color = vec4( mix( color.rgb, texel.rgb, texel.a ), color.a);
#endif
}
