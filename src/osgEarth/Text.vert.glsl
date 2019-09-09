#version $GLSL_VERSION_STR

out vec2 oe_Text_texCoord;

#pragma vp_entryPoint oe_Text_VS
#pragma vp_location   vertex_view

void oe_Text_VS(inout vec4 position)
{
    oe_Text_texCoord = gl_MultiTexCoord0.xy;
}
