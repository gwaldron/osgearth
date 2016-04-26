#version $GLSL_VERSION_STR

#pragma vp_entryPoint oe_detail_vertexView
#pragma vp_location   vertex_view

uniform float oe_detail_lod;    // uniform of base LOD
uniform float oe_detail_maxRange;
uniform float oe_detail_attenDist;
vec4 oe_layer_tilec;        // stage global - tile coordinates
out vec2 detailCoords;      // output to fragment stage
out float detailIntensity;  // output to fragment stage.
                
// Terrain SDK function
vec2 oe_terrain_scaleCoordsToRefLOD(in vec2 tc, in float refLOD);
               
void oe_detail_vertexView(inout vec4 VertexView)
{
  detailCoords = oe_terrain_scaleCoordsToRefLOD(oe_layer_tilec.st, int(oe_detail_lod));
  detailIntensity = clamp((oe_detail_maxRange - (-VertexView.z))/oe_detail_attenDist, 0.0, 1.0);
}