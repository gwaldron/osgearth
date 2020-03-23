#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name       GroundCover vertex shader
#pragma vp_entryPoint oe_Grass_VS_MODEL
#pragma vp_location   vertex_model
#pragma vp_order      0.1

#pragma import_defines(OE_GROUNDCOVER_USE_INSTANCING)
#pragma import_defines(OE_LANDCOVER_TEX)
#pragma import_defines(OE_LANDCOVER_TEX_MATRIX)
#pragma import_defines(OE_GROUNDCOVER_MASK_SAMPLER)
#pragma import_defines(OE_GROUNDCOVER_MASK_MATRIX)

//uncomment to activate
//#define OE_GROUNDCOVER_USE_ACTOR

uniform vec2 oe_GroundCover_numInstances;
uniform vec3 oe_GroundCover_LL, oe_GroundCover_UR;

// Noise texture:
uniform sampler2D oe_GroundCover_noiseTex;
vec4 oe_noise;
// Noise texture channels:
#define NOISE_SMOOTH   0
#define NOISE_RANDOM   1
#define NOISE_RANDOM_2 2
#define NOISE_CLUMPY   3

// LandCover texture
uniform sampler2D OE_LANDCOVER_TEX;
uniform mat4 OE_LANDCOVER_TEX_MATRIX;
float oe_LandCover_coverage;

vec3 vp_Normal;
vec4 vp_Color;
vec4 oe_layer_tilec;

uniform float osg_FrameTime;                  // Frame time (seconds) used for wind animation

uniform float oe_GroundCover_windFactor;      // wind blowing the foliage
uniform float oe_GroundCover_maxDistance;     // distance at which flora disappears

#ifdef OE_GROUNDCOVER_USE_ACTOR
///*uniform*/ vec3 actorPos = vec3(0.5, 0.5, 0.5); // tile pos of actor
uniform float actorRadius;
uniform float actorHeight;
uniform float actorPlace;
#endif

uniform vec3 oe_Camera;  // (vp width, vp height, lodscale)

// Output grass texture coordinates to the fragment shader
out vec2 oe_GroundCover_texCoord;

// Output that selects the land cover texture from the texture array (non interpolated)
flat out float oe_GroundCover_atlasIndex;

struct oe_GroundCover_Biome {
    int firstObjectIndex;
    int numObjects;
    float density;
    float fill;
    vec2 maxWidthHeight;
};
void oe_GroundCover_getBiome(in int index, out oe_GroundCover_Biome biome);

struct oe_GroundCover_Object {
    int type;             // 0=billboard 
    int objectArrayIndex; // index into the typed object array 
};
void oe_GroundCover_getObject(in int index, out oe_GroundCover_Object object);

struct oe_GroundCover_Billboard {
    int atlasIndexSide;
    int atlasIndexTop;
    float width;
    float height;
    float sizeVariation;
};
void oe_GroundCover_getBillboard(in int index, out oe_GroundCover_Billboard bb);

// SDK import
float oe_terrain_getElevation(in vec2);

// Generated in GroundCover.cpp
int oe_GroundCover_getBiomeIndex(in vec4);

#ifdef OE_GROUNDCOVER_MASK_SAMPLER
uniform sampler2D OE_GROUNDCOVER_MASK_SAMPLER;
uniform mat4 OE_GROUNDCOVER_MASK_MATRIX;
#endif

// Sample the elevation texture and move the vertex accordingly.
void oe_GroundCover_clamp(inout vec4 vertex, in vec3 up, in vec2 UV)
{
    float elev = oe_terrain_getElevation( UV );
    vertex.xyz += up*elev;
}

// Generate a wind-perturbation value
float oe_GroundCover_applyWind(float time, float factor, float randOffset)
{
    return sin(time + randOffset) * factor;
}

void oe_Grass_VS_MODEL(inout vec4 vertex_model)
{
    // intialize with a "no draw" value:
    oe_GroundCover_atlasIndex = -1.0;

    // input: 8 verts per instance so we can expand into a dual billboard
#ifdef OE_GROUNDCOVER_USE_INSTANCING
    int instanceID = gl_InstanceID;
#else
    //int instanceID = gl_VertexID / 12;
    int instanceID = gl_VertexID / 8;
#endif

    // Generate the UV tile coordinates (oe_layer_tilec) based on the current instance number
    vec2 numInstances = oe_GroundCover_numInstances;

    vec2 offset = vec2(
        float(instanceID % int(numInstances.x)),
        float(instanceID / int(numInstances.y)));

    // half the distance between cell centers
    vec2 halfSpacing = 0.5/numInstances;

    oe_layer_tilec = vec4( halfSpacing + offset/numInstances, 0, 1);

    // Sample our noise texture
    oe_noise = texture(oe_GroundCover_noiseTex, oe_layer_tilec.st);
    oe_noise = fract(oe_noise*5.5);

    // randomly shift each point off center
    //vec2 shift = vec2(fract(oe_noise[NOISE_RANDOM]*5.5), fract(oe_noise[NOISE_RANDOM_2]*5.5))*2-1;
    vec2 shift = vec2(oe_noise[NOISE_RANDOM], oe_noise[NOISE_RANDOM_2])*2-1;
    oe_layer_tilec.xy += shift*halfSpacing;

    // and place it correctly within the tile
    vertex_model.xy = mix(oe_GroundCover_LL.xy, oe_GroundCover_UR.xy, oe_layer_tilec.xy);
    vertex_model.z = 0.0;
    vertex_model.w = 1.0;

    vp_Normal = vec3(0,0,1);

    // Sample the landcover data. Must do this BEFORE calling getBiomeIndex.
    oe_LandCover_coverage = textureLod(OE_LANDCOVER_TEX, (OE_LANDCOVER_TEX_MATRIX*oe_layer_tilec).st, 0).r;

    // Look up the biome at this point:
    int biomeIndex = oe_GroundCover_getBiomeIndex(oe_layer_tilec);
    if ( biomeIndex < 0 )
    {
        // No biome defined; bail out without emitting any geometry.
        return;
    }

    // If we're using a mask texture, sample it now:
#ifdef OE_GROUNDCOVER_MASK_SAMPLER
    float mask = texture(OE_GROUNDCOVER_MASK_SAMPLER, (OE_GROUNDCOVER_MASK_MATRIX*oe_layer_tilec).st).a;
    if ( mask > 0.0 )
    {
        // Failed to pass the mask; no geometry emitted.
        return;
    }
#endif

    // Clamp the center point to the elevation.
    oe_GroundCover_clamp(vertex_model, vp_Normal, oe_layer_tilec.st);

    // Calculate the normalized camera range (oe_Camera.z = LOD Scale)
    float maxRange = oe_GroundCover_maxDistance / oe_Camera.z;
    float zv = (gl_ModelViewMatrix*vertex_model).z;
    float nRange = clamp(-zv/maxRange, 0.0, 1.0);

    // Distance culling:
    if ( nRange == 1.0 )
        return;

    // look up biome:
    oe_GroundCover_Biome biome;
    oe_GroundCover_getBiome(biomeIndex, biome);

    // discard instances based on noise value threshold (coverage). If it passes,
    // scale the noise value back up to [0..1]
    float fillEdgeFactor = 1.0;
    if ( oe_noise[NOISE_SMOOTH] > biome.fill )
    {
        return;
    }
    else
    {
        float d = (oe_noise[NOISE_SMOOTH] / biome.fill);
        if (d > 0.75)
            fillEdgeFactor = 1.0-((d-0.75)/0.25);
        oe_noise[NOISE_SMOOTH] /= biome.fill;
    }

    // select a billboard at random
    int objectIndex = biome.firstObjectIndex + int(floor(oe_noise[NOISE_RANDOM] * float(biome.numObjects)));
    objectIndex = min(objectIndex, biome.firstObjectIndex + biome.numObjects - 1);

    // Recover the object we randomly picked:
    oe_GroundCover_Object object;
    oe_GroundCover_getObject(objectIndex, object);

    // for now, assume type == BILLBOARD.
    // Find the billboard associated with the object:
    oe_GroundCover_Billboard billboard;
    oe_GroundCover_getBillboard(object.objectArrayIndex, billboard);

    oe_GroundCover_atlasIndex = float(billboard.atlasIndexSide);

    // push the falloff closer to the max distance.
    float falloff = 1.0-(nRange*nRange*nRange);

    // a pseudo-random scale factor to the width and height of a billboard
    float sizeScale = billboard.sizeVariation * (oe_noise[NOISE_RANDOM_2]*2.0-1.0);

    float width = (billboard.width + billboard.width*sizeScale) * clamp(fillEdgeFactor*2,0,1);

    float height = (billboard.height + billboard.height*sizeScale) * fillEdgeFactor;

    int which = gl_VertexID & 7; // mod8 - there are 8 verts per instance

    vp_Color = vec4(1,1,1,falloff);

    vec3 heightVector = vp_Normal*height;
    vec3 faceVector;    

#ifdef OE_GROUNDCOVER_USE_ACTOR
    vec4 actorPos = vec4(actorPlace, actorPlace,0,1);
    oe_GroundCover_clamp(actorPos, vp_Normal, vec2(0.5,0.5));
    actorPos.xyz += vp_Normal*actorHeight;
#endif

    if (which < 4)
    {
        // first quad
        faceVector = vec3(0,1,0);
    }
    else if (which < 8) // second quad
    {
        faceVector = vec3(-1,0,0);
    }

    vec3 halfWidthTangentVector = cross(faceVector, vp_Normal) * 0.5 * width;

    vertex_model.xyz =
        which==0 || which==4? vertex_model.xyz - halfWidthTangentVector :
        which==1 || which==5? vertex_model.xyz + halfWidthTangentVector :
        which==2 || which==6? vertex_model.xyz - halfWidthTangentVector + heightVector :
        vertex_model.xyz + halfWidthTangentVector + heightVector;

    if (which == 2 || which == 6) // top-left verts
    {
        float nw = oe_noise[NOISE_CLUMPY];
        float wind = width*oe_GroundCover_windFactor*nw;
        vertex_model.xy += faceVector.xy * oe_GroundCover_applyWind(osg_FrameTime*(1+nw), wind, oe_layer_tilec.s);
        vertex_model.xy += faceVector.xy * oe_GroundCover_applyWind(osg_FrameTime*(1-nw), wind, oe_layer_tilec.t);

#ifdef OE_GROUNDCOVER_USE_ACTOR
        vec3 pushVec = vertex_model.xyz - actorPos.xyz;
        float pushLen = length(pushVec);
        if (pushLen < actorRadius)
        {
            float z = vertex_model.z;
            vertex_model.xyz += pushVec/pushLen * actorRadius;
            vertex_model.z = min(vertex_model.z, z);
        }
#endif

        vp_Normal = faceVector;
    }
    else if (which == 3 || which == 7) // top-right verts
    {
        float nw = oe_noise[NOISE_CLUMPY];
        float wind = width*oe_GroundCover_windFactor*nw;
        vertex_model.xy -= faceVector.xy * oe_GroundCover_applyWind(osg_FrameTime*(1+nw), wind, oe_layer_tilec.s);
        vertex_model.xy -= faceVector.xy * oe_GroundCover_applyWind(osg_FrameTime*(1-nw), wind, oe_layer_tilec.t);

#ifdef OE_GROUNDCOVER_USE_ACTOR
        vec3 pushVec = vertex_model.xyz - actorPos.xyz;
        float pushLen = length(pushVec);
        if (pushLen < actorRadius)
        {
            float z = vertex_model.z;
            vertex_model.xyz += pushVec/pushLen * actorRadius;
            vertex_model.z = min(vertex_model.z, z);
        }
#endif

        vp_Normal = faceVector;
    }

    oe_GroundCover_texCoord =
        which == 0 || which == 4 ? vec2(0, 0) :
        which == 1 || which == 5 ? vec2(1, 0) :
        which == 2 || which == 6 ? vec2(0, 1) :
        vec2(1, 1);
}


[break]
#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name       GroundCover view shader
#pragma vp_entryPoint oe_Grass_VS_VIEW
#pragma vp_location   vertex_view
#pragma vp_order      0.1

vec3 vp_Normal;

void oe_Grass_VS_VIEW(inout vec4 vertex)
{
    // vp_Normal is set to faceVector coming in -- 
    // This flips the normal for backfacing polys (2-sided lighting)
    if (dot(vp_Normal, vertex.xyz) > 0.0)
        vp_Normal = -vp_Normal;
}
