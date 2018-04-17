#version $GLSL_VERSION_STR
$OSG_PRECISION_FLOAT

$OSG_VARYING_OUT vec2 texCoord;
$OSG_VARYING_OUT vec4 vertexColor;

#pragma vp_entryPoint text_vert
#pragma vp_location   vertex_view

void text_vert(inout vec4 position)
{
    texCoord = gl_MultiTexCoord0.xy;
    vertexColor = gl_Color;

#if !defined(GL_ES) && __VERSION__<140
    gl_ClipVertex = gl_ModelViewMatrix * gl_Vertex;
#endif
}
