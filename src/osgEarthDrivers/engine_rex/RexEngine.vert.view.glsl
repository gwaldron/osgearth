#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name       REX Engine - Vertex/View
#pragma vp_entryPoint oe_rex_elevateVertexAndSetTexCoords
#pragma vp_location   vertex_view
#pragma vp_order      0.4

// Stage globals
vec4 oe_layer_tilec;
vec3 oe_UpVectorView;
vec4 oe_layer_texc;
vec4 oe_layer_texcParent;

vec4 vp_Color;

uniform mat4 oe_layer_texMatrix;
uniform mat4 oe_layer_texParentMatrix;

uniform float oe_layer_minRange;
uniform float oe_layer_maxRange;
uniform float oe_layer_attenuationRange;

out float oe_layer_rangeOpacity;

// SDK functions:
float oe_terrain_getElevation(in vec2 uv);

void oe_rex_elevateVertexAndSetTexCoords(inout vec4 vertexView)
{
    // calculate the texture coordinates:
    oe_layer_texc       = oe_layer_texMatrix       * oe_layer_tilec;
	oe_layer_texcParent = oe_layer_texParentMatrix * oe_layer_tilec;

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
