#version $GLSL_VERSION_STR
#pragma vp_entryPoint oe_Text_VS
#pragma vp_location   vertex_view

out vec2 oe_Text_texcoord;

void oe_Text_VS(inout vec4 position)
{
    oe_Text_texcoord = gl_MultiTexCoord0.xy;

#if !defined(GL_ES) && __VERSION__<140
    gl_ClipVertex = gl_ModelViewMatrix * gl_Vertex;
#endif
}

[break]

#version $GLSL_VERSION_STR
#pragma vp_entryPoint oe_Text_FS
#pragma vp_location   fragment_coloring

uniform sampler2D glyphTexture;
in vec2 oe_Text_texcoord;

void oe_Text_FS(inout vec4 color)
{
    color.a *= texture(glyphTexture, oe_Text_texcoord).a;
}
