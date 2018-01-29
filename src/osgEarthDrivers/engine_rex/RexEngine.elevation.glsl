#version $GLSL_VERSION_STR

#pragma vp_name       REX Engine - Elevation
#pragma vp_entryPoint oe_rexEngine_elevation
#pragma vp_location   vertex_model
#pragma vp_order      0.7

#pragma import_defines(OE_TERRAIN_RENDER_ELEVATION)

// Vertex Markers:
#define MASK_MARKER_DISCARD  0.0
#define MASK_MARKER_NORMAL   1.0
#define MASK_MARKER_PATCH    2.0
#define MASK_MARKER_BOUNDARY 3.0

// stage
vec3 vp_Normal; // up vector
vec4 oe_layer_texc;
vec4 oe_layer_tilec;

// SDK functions:
float oe_terrain_getElevation(in vec2 uv);

void oe_rexEngine_elevation(inout vec4 vertexModel)
{    
#ifdef OE_TERRAIN_RENDER_ELEVATION
    float elev = 
        oe_layer_tilec.z == MASK_MARKER_BOUNDARY || oe_layer_tilec.z == MASK_MARKER_DISCARD ? 0.0f
        : oe_terrain_getElevation( oe_layer_tilec.st );

    vertexModel.xyz += normalize(vp_Normal) * elev;
#endif
}
