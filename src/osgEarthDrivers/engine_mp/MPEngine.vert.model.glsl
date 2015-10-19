#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_entryPoint oe_mp_vertModel
#pragma vp_location   vertex_model
#pragma vp_order      first

varying vec4 oe_layer_texc;
varying vec4 oe_layer_tilec;

varying vec3 oe_UpVectorView;
attribute vec4 oe_terrain_attr;

void oe_mp_vertModel(inout vec4 vertexModel)
{
    oe_layer_texc  = gl_MultiTexCoord$MP_PRIMARY_UNIT;
    oe_layer_tilec = gl_MultiTexCoord$MP_SECONDARY_UNIT;

    oe_UpVectorView = gl_NormalMatrix * oe_terrain_attr.xyz;
}
