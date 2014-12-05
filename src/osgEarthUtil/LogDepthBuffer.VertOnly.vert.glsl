#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

uniform float oe_ldb_FC;

void oe_ldb_vert(inout vec4 clip)
{
    const float C = $NEAR_RES_COEFF_STR;
    clip.z = (log2(max(1e-6,C*clip.w+1.0))*oe_ldb_FC - 1.0) * clip.w;
}