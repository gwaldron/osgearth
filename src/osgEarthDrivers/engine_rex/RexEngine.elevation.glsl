#version $GLSL_VERSION_STR

#pragma vp_name       REX Engine - Elevation
#pragma vp_entryPoint oe_rex_applyElevation
#pragma vp_location   vertex_view
#pragma vp_order      0

#pragma import_defines(OE_TERRAIN_RENDER_ELEVATION)

// Vertex Markers:
#define VERTEX_MARKER_DISCARD  1
#define VERTEX_MARKER_GRID     2
#define VERTEX_MARKER_PATCH    4
#define VERTEX_MARKER_BOUNDARY 8
#define VERTEX_MARKER_SKIRT    16

// stage
out vec4 oe_layer_tilec;
out vec3 oe_UpVectorView;
flat out int oe_terrain_vertexMarker;

uniform float oe_terrain_altitude;

// SDK functions:
float oe_terrain_getElevation(in vec2 uv);

void oe_rex_applyElevation(inout vec4 vertex)
{
#ifdef OE_TERRAIN_RENDER_ELEVATION

    bool ignore =
        ((oe_terrain_vertexMarker & VERTEX_MARKER_BOUNDARY) != 0) ||
        ((oe_terrain_vertexMarker & VERTEX_MARKER_DISCARD)  != 0);

    float elev = ignore ? 0.0f : oe_terrain_getElevation(oe_layer_tilec.st);

    vertex.xyz += oe_UpVectorView * elev;
#endif

    vertex.xyz += oe_UpVectorView * oe_terrain_altitude;
}
