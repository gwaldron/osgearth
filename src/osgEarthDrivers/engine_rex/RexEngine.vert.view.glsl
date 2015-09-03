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
uniform float     oe_tile_elevationSize;

void oe_rex_elevateVertexAndSetTexCoords(inout vec4 vertexView)
{
    // Texel-level scale and bias allow us to sample the elevation texture
    // on texel center instead of edge.
    float texelScale = (oe_tile_elevationSize-1.0)/oe_tile_elevationSize;
    float texelBias  = 0.5/oe_tile_elevationSize;

#if 0    
    vec2 elevc = oe_layer_tilec.st;

    // scale:
    elevc.st *= oe_tile_elevationTexMatrix[0][0] * texelScale;

    // bias:
    elevc.st += oe_tile_elevationTexMatrix[3].st*texelScale;

    // offset:
    elevc.st += 0.5/texelBias;
#endif

    // Apply the scale and bias.
    vec2 elevc = oe_layer_tilec.st
        * texelScale * oe_tile_elevationTexMatrix[0][0]     // scale
        + texelScale * oe_tile_elevationTexMatrix[3].st     // bias
        + texelBias;

    float elev = texture(oe_tile_elevationTex, elevc).r;

    vertexView.xyz += normalize(oe_UpVectorView) * elev;

    // calculate the texture coordinates:
	oe_layer_texc		= oe_layer_texMatrix       * oe_layer_tilec;
	oe_layer_texcParent = oe_layer_texParentMatrix * oe_layer_tilec;
}
