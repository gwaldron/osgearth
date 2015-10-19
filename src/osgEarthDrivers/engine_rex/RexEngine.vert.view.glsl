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

// marker that indicates the vertex belongs to a masking geometry
// and has a hard-coded height value
#define MASK_MARKER 2.0


void oe_rex_elevateVertexAndSetTexCoords(inout vec4 vertexView)
{
    float elev = oe_layer_tilec.z == MASK_MARKER ? 0.0f : oe_terrain_getElevation( oe_layer_tilec.st );

    vertexView.xyz += oe_UpVectorView * elev;

#if 0
    // calculate the texture coordinates:
    oe_layer_texc       = oe_layer_texMatrix       * oe_layer_tilec;
	oe_layer_texcParent = oe_layer_texParentMatrix * oe_layer_tilec;
#else
    // faster (MAD)
	oe_layer_texc.xy	   = oe_layer_tilec.xy*oe_layer_texMatrix[0][0] + oe_layer_texMatrix[3].xy;
    oe_layer_texc.zw       = oe_layer_tilec.zw;
    oe_layer_texcParent.xy = oe_layer_tilec.xy*oe_layer_texParentMatrix[0][0] + oe_layer_texParentMatrix[3].xy;
    oe_layer_texcParent.zw = oe_layer_tilec.zw;
#endif
}
