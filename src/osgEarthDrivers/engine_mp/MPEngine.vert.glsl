#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

varying vec4 oe_layer_texc;
varying vec4 oe_layer_tilec;

void oe_mp_setup_coloring(inout vec4 VertexModel)
{
    oe_layer_texc  = gl_MultiTexCoord$MP_PRIMARY_UNIT;
    oe_layer_tilec = gl_MultiTexCoord$MP_SECONDARY_UNIT;
}
