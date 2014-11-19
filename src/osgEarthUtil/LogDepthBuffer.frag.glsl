#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

uniform float oe_ldb_FC;
varying float oe_ldb_logz;

void oe_ldb_frag(inout vec4 color)
{
    gl_FragDepth = log2(oe_ldb_logz)*0.5*oe_ldb_FC;
}