#pragma include RexEngine.GL4.glsl

#pragma vp_name  REX Engine - Morphing
#pragma vp_function oe_rex_morph, vertex_view, 0.0

#pragma import_defines(OE_TERRAIN_MORPH_GEOMETRY)
#pragma import_defines(OE_TERRAIN_RENDER_ELEVATION)
#pragma import_defines(OE_IS_DEPTH_CAMERA)
#pragma import_defines(OE_TILE_SIZE)

out vec4 vp_Color;
out vec3 vp_Normal;
out vec3 oe_UpVectorView;
out vec4 oe_layer_tilec;
out float oe_rex_morphFactor;
flat out int oe_terrain_vertexMarker;

// stage globals (from init.gl4.glsl)
vec4 oe_tile_key;
mat4 oe_tile_mvm;

layout(location = 3) in vec3 neighbor;
layout(location = 4) in vec3 neighborNormal;

#ifdef OE_IS_DEPTH_CAMERA
uniform mat4 oe_shadowToPrimaryMatrix;
#endif

// SDK functions:
float oe_terrain_getElevation(in vec2 uv);

// Vertex Markers:
#define VERTEX_VISIBLE  1
#define VERTEX_BOUNDARY 2
#define VERTEX_HAS_ELEVATION 4
#define VERTEX_SKIRT 8
#define VERTEX_CONSTRAINT 16

void oe_rex_morph(inout vec4 vertex)
{
    return;

    oe_rex_morphFactor = 0.0;

    // compute the morphing factor to send down the pipe.
    // we need this even if vertex-morphing is off since we use it for
    // other things (like image blending)
    if ((oe_terrain_vertexMarker & VERTEX_CONSTRAINT) == 0)
    {
        // Start by computing the morphing factor.
        // Find the "would be" position of the vertex (the position the vertex would
        // assume with no morphing)
        vec4 elevated_vertex = vertex;

#ifdef OE_TERRAIN_RENDER_ELEVATION
        float elev = oe_terrain_getElevation(oe_layer_tilec.st);
        elevated_vertex.xyz += vp_Normal * elev;
#endif

#ifdef OE_IS_DEPTH_CAMERA
        // For a depth camera, we have to compute the morphed position
        // from the perspective of the primary camera so they match up:
        elevated_vertex = oe_shadowToPrimaryMatrix * elevated_vertex;
#endif
        
        int lod = int(oe_tile_key.z);
        float dist_to_eye = length(elevated_vertex.xyz);
        vec2 mc = oe_shared.morphConstants[lod];
        oe_rex_morphFactor = 1.0 - clamp(mc[0] - dist_to_eye * mc[1], 0.0, 1.0);

#ifdef OE_TERRAIN_MORPH_GEOMETRY
        vec4 neighbor_vertex = oe_tile_mvm * vec4(neighbor, 1); // vec4(gl_MultiTexCoord1.xyz, 1);
        vec3 neighbor_up = mat3(oe_tile_mvm) * neighborNormal; // gl_MultiTexCoord2.xyz;

        const float halfSize = (0.5*OE_TILE_SIZE) - 0.5;
        const float twoOverHalfSize = 2.0 / (OE_TILE_SIZE - 1.0);

        // Either 0 if point should not be morphed (in (x, y)), or the
        // delta to the neighbor point.
        vec2 fractionalPart = fract(oe_layer_tilec.st * halfSize) * twoOverHalfSize;
        vec2 neighbor_tilec = clamp(oe_layer_tilec.st - fractionalPart, 0.0, 1.0);

        // morph the vertex, normal, and uvs.
        vertex.xyz = mix(vertex.xyz, neighbor_vertex.xyz, oe_rex_morphFactor);
        vp_Normal = normalize(mix(vp_Normal, neighbor_up, oe_rex_morphFactor));
        oe_layer_tilec.st = mix(oe_layer_tilec.st, neighbor_tilec, oe_rex_morphFactor);
#endif
    }
}
