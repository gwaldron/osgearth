#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint "oe_fog_vertex"
#pragma vp_location   "vertex_view"
#pragma vp_order      "0.5"

varying float oe_fog_fogFactor;

void oe_fog_vertex(inout vec4 vertexVIEW)
{
    float z = length( vertexVIEW.xyz );
    const float LOG2 = 1.442695;
    oe_fog_fogFactor = exp2( -gl_Fog.density * gl_Fog.density * z * z * LOG2 );
    oe_fog_fogFactor = clamp(oe_fog_fogFactor, 0.0, 1.0);
}