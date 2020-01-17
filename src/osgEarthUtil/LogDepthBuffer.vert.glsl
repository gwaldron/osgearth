#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_logDepth_vert
#pragma vp_location   vertex_clip
#pragma vp_order      0.99

out float oe_LogDepth_logz;

void oe_logDepth_vert(inout vec4 clip)
{
    if (gl_ProjectionMatrix[3][3] == 0) // perspective only
    {
        mat4 clip2view = inverse(gl_ProjectionMatrix);
        vec4 farPoint = clip2view * vec4(0,0,1,1);
        float FAR = -farPoint.z / farPoint.w;

        const float C = 0.001;
        float FC = 1.0 / log(FAR*C + 1);
        oe_LogDepth_logz = log(max(1e-6, clip.w*C + 1.0))*FC;
        clip.z = (2.0*oe_LogDepth_logz - 1.0)*clip.w;
    }
    else
    {
        oe_LogDepth_logz = -1.0;
    }
}
