#version 330 compatibility

#pragma vp_name       "REX Engine - Vertex/View"
#pragma vp_entryPoint "oe_rexEngine_applyElevation"
#pragma vp_location   "vertex_view"
#pragma vp_order      "0.5"

// Stage globals
out vec4 oe_layer_tilec;
out vec4 vp_Vertex;
out vec3 vp_Normal;

uniform sampler2D oe_tile_elevationTex;
uniform mat4      oe_tile_elevationTexMatrix;

void oe_rexEngine_applyElevation(inout vec4 vertexView)
{
    // Sample the elevation texture and move the vertex accordingly.
    vec4 elevc = oe_tile_elevationTexMatrix * oe_layer_tilec;
    float elev = texture(oe_tile_elevationTex, elevc.st).r;

    // assumption: vp_Normal is normalized
    vertexView.xyz += vp_Normal*elev;
}
