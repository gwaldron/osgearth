#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_overlay_vertex
#pragma vp_location   vertex_view

uniform mat4 oe_overlay_texmatrix;
uniform float oe_overlay_rttLimitZ;

out vec4 oe_overlay_texcoord;

void oe_overlay_vertex(inout vec4 vertexVIEW)
{
    oe_overlay_texcoord = oe_overlay_texmatrix * vertexVIEW;
}