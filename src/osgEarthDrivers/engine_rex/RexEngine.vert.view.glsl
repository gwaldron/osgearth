#version $GLSL_VERSION_STR

#pragma vp_name       REX Engine - Vertex/View
#pragma vp_entryPoint oe_rex_elevateVertexAndSetTexCoords
#pragma vp_location   vertex_view
#pragma vp_order      0.4

// Stage globals
vec4 oe_layer_tilec;
vec4 oe_layer_texc;
vec4 oe_layer_texcParent;
vec3 oe_UpVectorView;

vec4 vp_Color;

uniform mat4 oe_layer_texMatrix;
uniform mat4 oe_layer_texParentMatrix;

uniform float oe_layer_minRange;
uniform float oe_layer_maxRange;
uniform float oe_layer_attenuationRange;

// SDK functions:
float oe_terrain_getElevation(in vec2 uv);

out float oe_layer_rangeOpacity;

// Vertex Markers:
#define MASK_MARKER_DISCARD  0.0
#define MASK_MARKER_NORMAL   1.0
#define MASK_MARKER_SKIRT    2.0
#define MASK_MARKER_BOUNDARY 3.0

void oe_rex_elevateVertexAndSetTexCoords(inout vec4 vertexView)
{
    float elev = 
        oe_layer_tilec.z == MASK_MARKER_BOUNDARY || oe_layer_tilec.z == MASK_MARKER_DISCARD ? 0.0f
        : oe_terrain_getElevation( oe_layer_tilec.st );

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

   float range = max(-vertexView.z, 0.0);

   float attenMin    = oe_layer_minRange - oe_layer_attenuationRange;
   float attenMax    = oe_layer_maxRange + oe_layer_attenuationRange;

   oe_layer_rangeOpacity =
       oe_layer_minRange >= oe_layer_maxRange                   ? 1.0 :
       range >= oe_layer_minRange && range < oe_layer_maxRange  ? 1.0 :
       range < oe_layer_minRange                                ? clamp((range-attenMin)/oe_layer_attenuationRange, 0.0, 1.0) :
       range > oe_layer_maxRange                                ? clamp((attenMax-range)/oe_layer_attenuationRange, 0.0, 1.0) :
       0.0;
}
