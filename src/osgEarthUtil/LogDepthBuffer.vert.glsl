#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma oe_entryPoint "oe_ldb_vert"
#pragma oe_location   "vertex_clip"
#pragma oe_order      "FLT_MAX"

uniform float oe_ldb_C;
uniform float oe_ldb_FC;
varying float oe_ldb_logz;

void oe_ldb_vert(inout vec4 clip)
{
    oe_ldb_logz = max(1e-6, clip.w*oe_ldb_C + 1.0);
    clip.z = log2(oe_ldb_logz)*oe_ldb_FC - 1.0;
}
