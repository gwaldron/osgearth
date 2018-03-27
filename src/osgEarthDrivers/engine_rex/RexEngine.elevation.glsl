#version $GLSL_VERSION_STR

#pragma vp_name       REX Engine - Elevation
#pragma vp_entryPoint oe_rexEngine_elevation
#pragma vp_location   vertex_model
#pragma vp_order      0.7

#pragma import_defines(OE_TERRAIN_RENDER_ELEVATION)

// Vertex Markers:
#define VERTEX_MARKER_DISCARD  1
#define VERTEX_MARKER_GRID     2
#define VERTEX_MARKER_PATCH    4
#define VERTEX_MARKER_BOUNDARY 8
#define VERTEX_MARKER_SKIRT    16

// stage
vec3 vp_Normal; // up vector
vec4 oe_layer_texc;
vec4 oe_layer_tilec;

flat out int oe_terrain_vertexMarker;

// SDK functions:
float oe_terrain_getElevation(in vec2 uv);

void oe_rexEngine_elevation(inout vec4 vertexModel)
{
#ifdef OE_TERRAIN_RENDER_ELEVATION

    bool ignore =
        ((oe_terrain_vertexMarker & VERTEX_MARKER_BOUNDARY) != 0) ||
        ((oe_terrain_vertexMarker & VERTEX_MARKER_DISCARD)  != 0);

    float elev = ignore ? 0.0f : oe_terrain_getElevation( oe_layer_tilec.st );

    vertexModel.xyz += normalize(vp_Normal) * elev;
#endif
}
