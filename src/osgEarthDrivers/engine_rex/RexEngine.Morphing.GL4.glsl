#version 460
#pragma include RexEngine.GL4.glsl

#pragma vp_name  REX Engine - Morphing
#pragma vp_function oe_rex_morph, vertex_view, 0.0

#pragma import_defines(OE_TERRAIN_MORPH_GEOMETRY)
#pragma import_defines(OE_TERRAIN_RENDER_ELEVATION)
#pragma import_defines(OE_IS_DEPTH_CAMERA)

out vec3 vp_Normal;
out vec3 oe_UpVectorView;
out vec4 oe_layer_tilec;
out float oe_rex_morphFactor;
flat out int oe_terrain_vertexMarker;

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
    // NYI in indirect. Todo. Remember...we tried moving this frmo model to vertex... reeval
    return;
}

#if 0
// Compute a morphing factor based on model-space inputs:
float oe_rex_ComputeMorphFactor(in vec4 position, in vec3 up)
{
    // Find the "would be" position of the vertex (the position the vertex would
    // assume with no morphing)
	vec4 wouldBePosition = position;

#ifdef OE_TERRAIN_RENDER_ELEVATION
        float elev = oe_terrain_getElevation( oe_layer_tilec.st );
		wouldBePosition.xyz += up*elev;
#endif

    vec4 wouldBePositionView = position; // gl_ModelViewMatrix * wouldBePosition;

#ifdef OE_IS_DEPTH_CAMERA
    // For a depth camera, we have to compute the morphed position
    // from the perspective of the primary camera so they match up:
    wouldBePositionView = oe_shadowToPrimaryMatrix * wouldBePositionView;
#endif

    float fDistanceToEye = length(wouldBePositionView.xyz); // or just -z.
    float fMorphLerpK  = 1.0f - clamp(tile[oe_tileID].morphConstants[0] - fDistanceToEye * tile[oe_tileID].morphConstants[1], 0.0, 1.0 );
    return fMorphLerpK;
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
        vec4 neighborVertex = vec4(gl_MultiTexCoord1.xyz, 1.0);
        vec3 neighborNormal = gl_MultiTexCoord2.xyz;

        float halfSize        = (0.5*OE_TILE_SIZE)-0.5;
        float twoOverHalfSize = 2.0/(OE_TILE_SIZE -1.0);
        // Either 0 if point should not be morphed (in (x, y)), or the
        // delta to the neighbor point.
        vec2 fractionalPart = fract(oe_layer_tilec.st * halfSize) * twoOverHalfSize;
        vec4 neighbor_tilec = oe_layer_tilec;
        neighbor_tilec.st = clamp(oe_layer_tilec.st - fractionalPart, 0.0, 1.0);

        // morph the vertex:
        vertexModel.xyz = mix(vertex.xyz, neighborVertex.xyz, oe_rex_morphFactor);

        // morph the normal:
        vp_Normal = normalize(mix(vp_Normal, neighborNormal, oe_rex_morphFactor));

        // morph hte uvs:
        oe_layer_tilec.st = mix(new_tilec.st, new_neighbor_tilec.st, oe_rex_morphFactor);
#endif
    }
    else
    {
        oe_rex_morphFactor = 0.0;
    }
}
#endif