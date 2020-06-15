#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name       REX Engine - Morphing
#pragma vp_entryPoint oe_rex_morph
#pragma vp_location   vertex_model
#pragma vp_order      0.5

#pragma import_defines(OE_TERRAIN_MORPH_GEOMETRY)
#pragma import_defines(OE_TERRAIN_RENDER_ELEVATION)
#pragma import_defines(OE_IS_DEPTH_CAMERA)
#pragma import_defines(OE_ELEVATION_CONSTRAINT_TEX)
#pragma import_defines(OE_ELEVATION_CONSTRAINT_TEX_MATRIX)

// stage
vec3 vp_Normal;

vec4 oe_layer_tilec;

out float oe_rex_morphFactor;

flat out int oe_terrain_vertexMarker;

uniform vec2  oe_tile_morph;
uniform float oe_tile_size;

// Constraint stuff
#ifdef OE_ELEVATION_CONSTRAINT_TEX
uniform sampler2DArray OE_ELEVATION_CONSTRAINT_TEX;
uniform mat4 OE_ELEVATION_CONSTRAINT_TEX_MATRIX;
#endif 
uniform vec2 oe_tile_elevTexelCoeff;

#ifdef OE_IS_DEPTH_CAMERA
uniform mat4 oe_shadowToPrimaryMatrix;
#endif

// SDK functions:
float oe_terrain_getElevation(in vec2 uv);

// Vertex Markers:
#define VERTEX_MARKER_DISCARD  1
#define VERTEX_MARKER_GRID     2
#define VERTEX_MARKER_PATCH    4
#define VERTEX_MARKER_BOUNDARY 8
#define VERTEX_MARKER_SKIRT    16

#ifdef OE_ELEVATION_CONSTRAINT_TEX
void moveToConstraint(in vec4 vertex, in vec4 layer_tilec, out vec4 newVertex, out vec4 new_layer_tilec)
{
    // Don't constrain edges
    if (any(equal(layer_tilec.st, vec2(0)))
        || any(equal(layer_tilec.st, vec2(1)))
        || OE_ELEVATION_CONSTRAINT_TEX_MATRIX[0][0] < .5)
    {
        newVertex = vertex;
        new_layer_tilec = layer_tilec;
        return;
    }
    vec2 elevc = layer_tilec.xy
        * oe_tile_elevTexelCoeff.x * OE_ELEVATION_CONSTRAINT_TEX_MATRIX[0][0] // scale
        + oe_tile_elevTexelCoeff.x * OE_ELEVATION_CONSTRAINT_TEX_MATRIX[3].st // bias
        + oe_tile_elevTexelCoeff.y;
    float tcMixer = 1.0;
    vec4 dir = texture(OE_ELEVATION_CONSTRAINT_TEX, vec3(elevc, 0.0));
    if (OE_ELEVATION_CONSTRAINT_TEX_MATRIX[0][0] == 1.0)
    {
        vec4 dirInParent = texture(OE_ELEVATION_CONSTRAINT_TEX, vec3(elevc, 1.0));
        dir = mix(dir, dirInParent, oe_rex_morphFactor);
        tcMixer = mix(1.0, 2.0, oe_rex_morphFactor);
    }
    else
    {
        tcMixer = 1.0/OE_ELEVATION_CONSTRAINT_TEX_MATRIX[0][0];
    }
    newVertex = vertex;
    newVertex.xy += dir.xy;
    new_layer_tilec = layer_tilec;
    new_layer_tilec.xy += dir.zw * tcMixer;
}
#else
void moveToConstraint(in vec4 vertex, in vec4 layer_tilec, out vec4 newVertex, out vec4 new_layer_tilec)
{
    newVertex = vertex;
    new_layer_tilec = layer_tilec;
}
#endif

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

    vec4 wouldBePositionView = gl_ModelViewMatrix * wouldBePosition;

#ifdef OE_IS_DEPTH_CAMERA
    // For a depth camera, we have to compute the morphed position
    // from the perspective of the primary camera so they match up:
    wouldBePositionView = oe_shadowToPrimaryMatrix * wouldBePositionView;
#endif

    float fDistanceToEye = length(wouldBePositionView.xyz); // or just -z.
    float fMorphLerpK  = 1.0f - clamp( oe_tile_morph[0] - fDistanceToEye * oe_tile_morph[1], 0.0, 1.0 );
    return fMorphLerpK;
}

// In the transition from an LOD to the next lower LOD, morphing moves
// tile grid points that will disappear towards grid points that exist
// in the lower LOD. When the transition is complete
// (oe_rex_morphFactor == 1.0), those points are coincident. If we
// consider grid points numbered on x,y from 0 to tilesize - 1, then
// the points that "survive" have even x,y indices and don't move. The
// neighbor vertex coordinates and normals are passed in as vertex
// attributes, but the neighbor texture coordinates are calculated
// here.
//
// XXX constraints

void oe_rex_morph(inout vec4 vertexModel)
{
    // compute the morphing factor to send down the pipe.
    // we need this even if vertex-morphing is off since we use it for
    // other things (like image blending)
    if ((oe_terrain_vertexMarker & VERTEX_MARKER_GRID) != 0)
    {
        oe_rex_morphFactor = oe_rex_ComputeMorphFactor(vertexModel, vp_Normal);
#ifdef OE_TERRAIN_MORPH_GEOMETRY
        vec4 neighborVertexModel = vec4(gl_MultiTexCoord1.xyz, 1.0);
        vec3 neighborNormal = gl_MultiTexCoord2.xyz;

        float halfSize        = (0.5*oe_tile_size)-0.5;
        float twoOverHalfSize = 2.0/(oe_tile_size-1.0);
        // Either 0 if point should not be morphed (in (x, y)), or the
        // delta to the neighbor point.
        vec2 fractionalPart = fract(oe_layer_tilec.st * halfSize) * twoOverHalfSize;
        vec4 neighbor_tilec = oe_layer_tilec;
        neighbor_tilec.st = clamp(oe_layer_tilec.st - fractionalPart, 0.0, 1.0);

        // oe_layer_tilec.st = clamp(oe_layer_tilec.st - (fractionalPart * oe_rex_morphFactor), 0.0, 1.0);
        vec4 newVertexModel, new_tilec;
        moveToConstraint(vertexModel, oe_layer_tilec, newVertexModel, new_tilec);
        vec4 newNeighborVertexModel, new_neighbor_tilec;
        moveToConstraint(neighborVertexModel, neighbor_tilec, newNeighborVertexModel, new_neighbor_tilec);
        // morph the vertex:
        vertexModel.xyz = mix(newVertexModel.xyz, newNeighborVertexModel.xyz, oe_rex_morphFactor);
        // morph the normal:
        vp_Normal = normalize(mix(vp_Normal, neighborNormal, oe_rex_morphFactor));
        oe_layer_tilec.st = mix(new_tilec.st, new_neighbor_tilec.st, oe_rex_morphFactor);
#endif
    }
    else
    {
        oe_rex_morphFactor = 0.0;
    }
}
