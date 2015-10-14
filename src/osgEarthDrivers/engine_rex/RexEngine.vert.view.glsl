#version 330 compatibility

#pragma vp_name       REX Engine - Vertex/View
#pragma vp_entryPoint oe_rex_elevateVertexAndSetTexCoords
#pragma vp_location   vertex_view
#pragma vp_order      0.4

// Stage globals
vec4 oe_layer_tilec;
vec4 oe_layer_texc;
vec4 oe_layer_texcParent;
vec3 oe_UpVectorView;

uniform mat4 oe_layer_texMatrix;
uniform mat4 oe_layer_texParentMatrix;

// SDK functions:
float oe_terrain_getElevation(in vec2 uv);


void oe_rex_elevateVertexAndSetTexCoords(inout vec4 vertexView)
{
    float elev = oe_layer_tilec.z == 2.0 ? 0.0f : oe_terrain_getElevation( oe_layer_tilec.st );

    vertexView.xyz += normalize(oe_UpVectorView) * elev;

    // calculate the texture coordinates:
	oe_layer_texc		= oe_layer_texMatrix       * oe_layer_tilec;
	oe_layer_texcParent = oe_layer_texParentMatrix * oe_layer_tilec;
}
