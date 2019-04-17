#version $GLSL_VERSION_STR

#pragma vp_entryPoint oe_logDepth_vert
#pragma vp_location   vertex_clip
#pragma vp_order      0.99

void oe_logDepth_vert(inout vec4 clip)
{
    if (gl_ProjectionMatrix[3][3] == 0.0) // perspective only
    {
        mat4 clip2view = inverse(gl_ProjectionMatrix);
        vec4 farPoint = clip2view * vec4(0,0,1,1);
        float FAR = -farPoint.z / farPoint.w;

        float FC = 2.0 / log2(FAR + 1);
        clip.z = (log2(max(1e-6, clip.w+1.0))*FC - 1.0) * clip.w;
    }
}
