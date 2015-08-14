#version 330 compatibility

#pragma vp_name       "REX Engine - Vertex/View"
#pragma vp_entryPoint "oe_rex_elevateVertexAndSetTexCoords"
#pragma vp_location   "vertex_view"
#pragma vp_order      "0.4"

// Stage globals
vec4 oe_layer_tilec;
vec4 oe_layer_texc;
vec4 oe_layer_texcParent;
vec3 oe_UpVectorView;

uniform mat4 oe_layer_texMatrix;
uniform mat4 oe_layer_texParentMatrix;

uniform sampler2D oe_tile_elevationTex;
uniform mat4      oe_tile_elevationTexMatrix;

void oe_rex_elevateVertexAndSetTexCoords(inout vec4 vertexView)
{
	// Sample the elevation texture and move the vertex accordingly.
	vec4 elevc = oe_tile_elevationTexMatrix * oe_layer_tilec;
	float elev = texture(oe_tile_elevationTex, elevc.st).r;

	// assumption: vp_Normal is normalized
    vertexView.xyz += normalize(oe_UpVectorView) * elev;

    // calculate the texture coordinates:
	oe_layer_texc		= oe_layer_texMatrix       * oe_layer_tilec;
	oe_layer_texcParent = oe_layer_texParentMatrix * oe_layer_tilec;
}
