#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_logDepth_vert
#pragma vp_location   vertex_clip
#pragma vp_order      0.99

uniform float oe_logDepth_FC;
out float oe_logDepth_clipz;

void oe_logDepth_vert(inout vec4 clip)
{
    if (gl_ProjectionMatrix[3][3] == 0.0) // perspective
    {
        clip.z = (log2(max(1e-6, 1.0 + clip.w)) * oe_logDepth_FC - 1.0) * clip.w;
        oe_logDepth_clipz = 1.0 + clip.w;
    }
    else
    {
        // Set clipz to 2.0 to avoid warnings on value not being initialized
        oe_logDepth_clipz = 2.0;
    }
}
