#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_logDepth_frag
#pragma vp_location   fragment_lighting
#pragma vp_order      0.99

uniform float oe_logDepth_FC;
in float oe_logDepth_clipz;

void oe_logDepth_frag(inout vec4 color)
{
    if (gl_ProjectionMatrix[3][3] == 0.0) // perspective
    {
        gl_FragDepth = log2(oe_logDepth_clipz) * 0.5*oe_logDepth_FC;
    }
    else
    {
        // must set gl_FragDepth is all branches even if it doesn't change
        gl_FragDepth = gl_FragCoord.z;
    }
}
