#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

uniform float oe_ldb_FC;
varying float oe_ldb_logz;

void oe_ldb_vert(inout vec4 clip)
{
    const float C = $NEAR_RES_COEFF_STR;
    oe_ldb_logz = max(1e-6, clip.w*C + 1.0);
    clip.z = log2(oe_ldb_logz)*oe_ldb_FC - 1.0;
}
