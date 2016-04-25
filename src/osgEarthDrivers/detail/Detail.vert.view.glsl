#version $GLSL_VERSION_STR

#pragma vp_entryPoint oe_detail_vertexView
#pragma vp_location   vertex_view

uniform float oe_detail_lod;    // uniform of base LOD
vec4 oe_layer_tilec;        // stage global - tile coordinates
out vec2 detailCoords;      // output to fragment stage
                
// Terrain SDK function
vec2 oe_terrain_scaleCoordsToRefLOD(in vec2 tc, in float refLOD);
               
void oe_detail_vertexView(inout vec4 color)
{
  detailCoords = oe_terrain_scaleCoordsToRefLOD(oe_layer_tilec.st, int(oe_detail_lod));
}