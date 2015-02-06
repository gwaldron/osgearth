#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma oe_entryPoint "oe_ldb_vert"
#pragma oe_location   "vertex_clip"
#pragma oe_order      "FLT_MAX"

uniform float oe_ldb_C;
uniform float oe_ldb_FC;

void oe_ldb_vert(inout vec4 clip)
{
    clip.z = (log2(max(1e-6,oe_ldb_C*clip.w+1.0))*oe_ldb_FC - 1.0) * clip.w;
}