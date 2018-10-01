#version 330
#pragma vp_name       ClipPlane.glsl
#pragma vp_entryPoint oe_ClipPlane_VS
#pragma vp_location   vertex_view
#pragma vp_order      last
#pragma import_defines(OE_CLIPPLANE_NUM)

// OSG built-in to transform from view to world
uniform mat4 osg_ViewMatrixInverse;

// clipping plane
uniform vec4 oe_ClipPlane_plane;

void oe_ClipPlane_VS(inout vec4 vertex_view)
{
#ifndef GL_ES
    gl_ClipDistance[OE_CLIPPLANE_NUM] = dot(osg_ViewMatrixInverse * vertex_view, oe_ClipPlane_plane);
#endif
}
