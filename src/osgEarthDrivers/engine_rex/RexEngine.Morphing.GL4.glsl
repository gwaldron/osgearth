#version 460
#pragma include RexEngine.GL4.glsl

#pragma vp_name  REX Engine - Morphing
#pragma vp_function oe_rex_morph, vertex_view, 0.0

#pragma import_defines(OE_TERRAIN_MORPH_GEOMETRY)
#pragma import_defines(OE_TERRAIN_RENDER_ELEVATION)
#pragma import_defines(OE_IS_DEPTH_CAMERA)

out vec4 vp_Color;
out vec3 vp_Normal;
out vec3 oe_UpVectorView;
out vec4 oe_layer_tilec;
out float oe_rex_morphFactor;
flat out int oe_terrain_vertexMarker;

// stage globals (from init.gl4.glsl
vec4 oe_tile_key;
mat4 oe_tile_mvm;


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

int oe_rex_getNeighborIndex()
{
    int off = 0;
    int col = gl_VertexID % OE_TILE_SIZE;
    int row = gl_VertexID / OE_TILE_SIZE;
    if ((col & 1) == 1) off -= 1;
    if ((row & 1) == 1) off -= OE_TILE_SIZE;
    return gl_VertexID + off;
}

// Compute a morphing factor
float oe_rex_ComputeMorphFactor(in vec4 position, in vec3 up)
{
    // Find the "would be" position of the vertex (the position the vertex would
    // assume with no morphing)
	vec4 final_position = position;

#ifdef OE_TERRAIN_RENDER_ELEVATION
    float elev = oe_terrain_getElevation( oe_layer_tilec.st );
    final_position.xyz += up*elev;
#endif

#ifdef OE_IS_DEPTH_CAMERA
    // For a depth camera, we have to compute the morphed position
    // from the perspective of the primary camera so they match up:
    final_position = oe_shadowToPrimaryMatrix * final_position;
#endif

    int lod = int(oe_tile_key.z);
    float dist_to_eye = length(final_position.xyz); // or just -z.
    vec2 mc = oe_global.morphConstants[lod];
    float morph = 1.0f - clamp(mc[0] - dist_to_eye * mc[1], 0.0, 1.0 );
    return morph;
}

void oe_rex_morph(inout vec4 vertex)
{
    // compute the morphing factor to send down the pipe.
    // we need this even if vertex-morphing is off since we use it for
    // other things (like image blending)
    if ((oe_terrain_vertexMarker & VERTEX_CONSTRAINT) == 0)
    {
        oe_rex_morphFactor = oe_rex_ComputeMorphFactor(vertex, oe_UpVectorView);

#ifdef OE_TERRAIN_MORPH_GEOMETRY

        int ni = oe_rex_getNeighborIndex();

        vec4 neighbor_vertex = oe_tile_mvm * vec4(oe_tile[oe_tileID].verts[ni].xyz, 1.0);
        vec3 neighbor_normal = mat3(oe_tile_mvm) * oe_tile[oe_tileID].normals[ni].xyz;

        const float halfSize = (0.5*OE_TILE_SIZE) - 0.5;
        const float twoOverHalfSize = 2.0 / (OE_TILE_SIZE - 1.0);

        // Either 0 if point should not be morphed (in (x, y)), or the
        // delta to the neighbor point.
        vec2 fractionalPart = fract(oe_layer_tilec.st * halfSize) * twoOverHalfSize;
        vec2 neighbor_tilec = clamp(oe_layer_tilec.st - fractionalPart, 0.0, 1.0);

        // morph the vertex:
        vertex.xyz = mix(vertex.xyz, neighbor_vertex.xyz, oe_rex_morphFactor);

        // morph the normal:
        vp_Normal = normalize(mix(vp_Normal, neighbor_normal, oe_rex_morphFactor));

        // morph hte uvs:
        oe_layer_tilec.st = mix(oe_layer_tilec.st, neighbor_tilec, oe_rex_morphFactor);
#endif
    }
    else
    {
        oe_rex_morphFactor = 0.0;
    }
}
