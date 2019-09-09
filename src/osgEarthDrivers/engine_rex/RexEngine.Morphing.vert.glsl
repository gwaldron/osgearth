#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name       REX Engine - Morphing
#pragma vp_entryPoint oe_rexEngine_morph
#pragma vp_location   vertex_model
#pragma vp_order      0.5

#pragma import_defines(OE_TERRAIN_MORPH_GEOMETRY)
#pragma import_defines(OE_TERRAIN_RENDER_ELEVATION)
#pragma import_defines(OE_IS_DEPTH_CAMERA)

// stage
vec3 vp_Normal;

vec4 oe_layer_tilec;

out float oe_rex_morphFactor;

flat out int oe_terrain_vertexMarker;

uniform vec2  oe_tile_morph;
uniform float oe_tile_size;

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


void oe_rexEngine_morph(inout vec4 vertexModel)
{    
    // compute the morphing factor to send down the pipe.
    // we need this even if vertex-morphing is off since we use it for 
    // other things (like image blending)
    if ((oe_terrain_vertexMarker & VERTEX_MARKER_GRID) != 0)
    {
        oe_rex_morphFactor = oe_rex_ComputeMorphFactor(vertexModel, vp_Normal);    

#ifdef OE_TERRAIN_MORPH_GEOMETRY
        vec3 neighborVertexModel = gl_MultiTexCoord1.xyz;        
        vec3 neighborNormal = gl_MultiTexCoord2.xyz;
        
        float halfSize        = (0.5*oe_tile_size)-0.5;
        float twoOverHalfSize = 2.0/(oe_tile_size-1.0);   
        vec2 fractionalPart = fract(oe_layer_tilec.st * halfSize) * twoOverHalfSize;
        oe_layer_tilec.st = clamp(oe_layer_tilec.st - (fractionalPart * oe_rex_morphFactor), 0.0, 1.0);

        // morph the vertex:
        vec3 morphVector = neighborVertexModel.xyz - vertexModel.xyz;
        vertexModel.xyz = vertexModel.xyz + morphVector*oe_rex_morphFactor;

        // morph the normal:
        morphVector = neighborNormal - vp_Normal;
        vp_Normal = normalize(vp_Normal + morphVector*oe_rex_morphFactor);
#endif
    }
    else
    {
        oe_rex_morphFactor = 0.0;
    }
}
