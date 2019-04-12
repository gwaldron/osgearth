#version $GLSL_VERSION_STR
#pragma vp_name PointDrawable
#pragma vp_entryPoint oe_PointDrawable_VS_VIEW
#pragma vp_location vertex_view
#pragma vp_order last

uniform float oe_GL_PointSize;

void oe_PointDrawable_VS_VIEW(inout vec4 vertexView)
{
    gl_PointSize = oe_GL_PointSize;
}

[break]

#version $GLSL_VERSION_STR
#pragma vp_name PointDrawable FS
#pragma vp_entryPoint oe_PointDrawable_FS
#pragma vp_location fragment_coloring
#pragma import_defines(OE_POINT_SMOOTH)

// Some GLES impls don't have fwidth yet:
#if !defined(GL_ES) || defined(GL_OES_standard_derivatives)
#define HAVE_FWIDTH
#endif

void oe_PointDrawable_FS(inout vec4 color)
{
#ifdef OE_POINT_SMOOTH
    vec2 c = 2.0*gl_PointCoord-1.0;
    float r = dot(c, c);

#ifdef HAVE_FWIDTH
    float d = fwidth(r);
    color.a = 1.0 - smoothstep(1.0 - d, 1.0 + d, r);
#else
    color.a = 1.0 - smoothstep(-0.1, 0.1, r - 1.0);
#endif

    if (color.a < 0.1)
        discard;
#endif
}
