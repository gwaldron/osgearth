#pragma include RexEngine.GL4.glsl
#pragma vp_name REX Engine - Init Model Space
#pragma vp_function oe_rex_init_model, vertex_model, first

#pragma import_defines(OE_TERRAIN_RENDER_ELEVATION)
#pragma import_defines(OE_TERRAIN_MORPH_GEOMETRY)
#pragma import_defines(OE_TERRAIN_MORPH_IMAGERY)
#pragma import_defines(OE_IS_SHADOW_CAMERA)

// SDK functions:
float oe_terrain_getElevation(in vec2 uv);

// attributes
layout(location = 0) in vec3 a_vertex;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec3 a_uv;
layout(location = 3) in vec3 a_neighbor;
layout(location = 4) in vec3 a_neighborNormal;

#define VERTEX_CONSTRAINT 16

// uniforms
uniform vec4 oe_terrain_color;
#ifdef OE_IS_SHADOW_CAMERA
uniform mat4 oe_shadowToPrimaryMatrix;
#endif

// stage globals
vec4 oe_tile_key;
mat4 oe_tile_mvm;
vec4 oe_vertex_model;

// outputs
out vec4 oe_layer_tilec;
out vec3 vp_Normal;
out vec4 vp_Color;
out vec3 oe_UpVectorView;
out float oe_rex_morphFactor;
out vec4 oe_terrain_tessLevel;
flat out int oe_terrain_vertexMarker;

void oe_rex_morph_model();


void oe_rex_init_model(inout vec4 not_used)
{
    // instance ID from the DrawElementsIndirect cmd
    oe_tileID = gl_DrawID;

    // Color of the underlying map geometry (untextured)
    vp_Color = oe_terrain_color;

    // initialize:
    oe_rex_morphFactor = 0.0;

    // Default tessellation level (where applicable)
    oe_terrain_tessLevel = vec4(1);

    // Custom MVM (don't use gl_ModelViewMatrix)
    oe_tile_mvm = oe_tile[oe_tileID].modelViewMatrix;

    // assign vertex marker flags
    oe_terrain_vertexMarker = int(a_uv.z);

    // extract tile UV (global data)
    oe_layer_tilec = vec4(a_uv.xy, 0, 1);

    // the tile key
    oe_tile_key = oe_tile[oe_tileID].tileKey;

    vp_Normal = a_normal;

    // by default:
    oe_vertex_model = vec4(a_vertex, 1);

#if defined(OE_TERRAIN_MORPH_GEOMETRY) || defined(OE_TERRAIN_MORPH_IMAGERY)
    if ((oe_terrain_vertexMarker & VERTEX_CONSTRAINT) == 0)
        oe_rex_morph_model();
#endif
}

// Uses neighbor data to morph across LODs
void oe_rex_morph_model()
{
    // Compute the morphing factor. We need the distance to
    // the "final" vertex (elevation applied) to compute it

#ifdef OE_TERRAIN_RENDER_ELEVATION
    float elev = oe_terrain_getElevation(oe_layer_tilec.st);
    vec4 vertex_view = oe_tile_mvm * vec4(a_vertex + vp_Normal * elev, 1);
#else
    vec4 vertex_view = oe_tile_mvm * vec4(a_vertex, 1);
#endif

#ifdef OE_IS_SHADOW_CAMERA
    // For a depth camera, we have to compute the morphed position
    // from the perspective of the primary camera so they match up:
    vertex_view = oe_shadowToPrimaryMatrix * vertex_view;
#endif

    int lod = int(oe_tile_key.z);
    float dist_to_eye = length(vertex_view.xyz);
    vec2 mc = oe_shared.morphConstants[lod];
    oe_rex_morphFactor = 1.0 - clamp(mc[0] - dist_to_eye * mc[1], 0.0, 1.0);

    // if just morphing imagery, we're done - otherwise we also need
    // to morph the vertex/normal/uv:

#ifdef OE_TERRAIN_MORPH_GEOMETRY
    // Compute the delta to the neighbor point.
    const float halfSize = (0.5*OE_TILE_SIZE) - 0.5;
    const float twoOverHalfSize = 2.0 / (OE_TILE_SIZE - 1.0);
    vec2 fractionalPart = fract(oe_layer_tilec.st * halfSize) * twoOverHalfSize;
    vec2 neighbor_tilec = clamp(oe_layer_tilec.st - fractionalPart, 0.0, 1.0);

    // morph the vertex, normal, and uvs.
    oe_vertex_model.xyz = mix(a_vertex, a_neighbor, oe_rex_morphFactor);
    vp_Normal = normalize(mix(vp_Normal, a_neighborNormal, oe_rex_morphFactor));
    oe_layer_tilec.st = mix(oe_layer_tilec.st, neighbor_tilec, oe_rex_morphFactor);
#endif
}


[break]
#pragma include RexEngine.GL4.glsl
#pragma vp_name REX Engine - Init View Space
#pragma vp_function oe_rex_init_view, vertex_view, first

// attributes
layout(location = 1) in vec3 normal;

// outputs
out vec4 oe_layer_tilec;
out vec3 vp_Normal;
out vec3 vp_VertexView;
out vec3 oe_UpVectorView;

// stage globals
mat4 oe_tile_mvm;
vec4 oe_vertex_model;

void oe_rex_init_view(inout vec4 vert_view)
{
    oe_tile_mvm = oe_tile[oe_tileID].modelViewMatrix;

    // Vertex to view space
    vert_view = oe_tile_mvm * oe_vertex_model;

    // rewrite the internal VP variables
    vp_VertexView = vert_view.xyz;

    // Normal to view space (use the original normal here)
    vp_Normal = mat3(oe_tile_mvm) * normal;

    // "up" vector at this vertex in view space, which we will later
    // need in order to elevate the terrain. vp_Normal can change later
    // but UpVectorView will stay the same.
    oe_UpVectorView = vp_Normal;
}
