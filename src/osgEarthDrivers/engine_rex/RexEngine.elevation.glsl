#version $GLSL_VERSION_STR

#pragma vp_name REX Engine - Elevation
#pragma vp_function oe_rex_applyElevation, vertex_view, 0.1

#pragma import_defines(OE_TERRAIN_RENDER_ELEVATION)

// Vertex Markers:
#define VERTEX_VISIBLE  1
#define VERTEX_BOUNDARY 2
#define VERTEX_HAS_ELEVATION 4
#define VERTEX_SKIRT 8
#define VERTEX_CONSTRAINT 16

// stage
out vec4 oe_layer_tilec;
out vec3 oe_UpVectorView;
flat out int oe_terrain_vertexMarker;

uniform float oe_terrain_altitude;

// SDK functions:
float oe_terrain_getElevation();

void oe_rex_applyElevation(inout vec4 vertex)
{
#ifdef OE_TERRAIN_RENDER_ELEVATION

    bool elevate =
        ((oe_terrain_vertexMarker & VERTEX_VISIBLE) != 0) &&
        ((oe_terrain_vertexMarker & VERTEX_HAS_ELEVATION) == 0);

    float elev = elevate ? oe_terrain_getElevation() : 0.0;

    vertex.xyz += oe_UpVectorView * elev;
#endif

    vertex.xyz += oe_UpVectorView * oe_terrain_altitude;
}
