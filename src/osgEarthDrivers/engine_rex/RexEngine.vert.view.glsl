#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name       REX Engine - Vertex/View
#pragma vp_entryPoint oe_rex_setTexCoords
#pragma vp_location   vertex_view
#pragma vp_order      0.4

// Stage globals
vec4 oe_layer_tilec;
vec2 oe_layer_texc;
vec2 oe_layer_texcParent;

uniform mat4 oe_layer_texMatrix;
uniform mat4 oe_layer_texParentMatrix;

void oe_rex_setTexCoords(inout vec4 vertexView)
{
    // calculate the texture coordinates:
    oe_layer_texc       = (oe_layer_texMatrix * oe_layer_tilec).st;
	oe_layer_texcParent = (oe_layer_texParentMatrix * oe_layer_tilec).st;
}
