#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name       GroundCover vertex shader
#pragma vp_entryPoint oe_Grass_VS
#pragma vp_location   vertex_view

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
vec3 oe_UpVectorView;

uniform float osg_FrameTime; // OSG frame time (seconds) used for wind animation

uniform float oe_GroundCover_wind;  // wind strength
uniform float oe_GroundCover_maxDistance; // distance at which flora disappears

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

// Generate a wind-perturbation value
float oe_GroundCover_applyWind(float time, float factor, float randOffset)
{
    return sin(time + randOffset) * factor;
}

uniform float shmoo;

void oe_Grass_VS(inout vec4 vertex)
{
    // intialize with a "no draw" value:
    oe_GroundCover_atlasIndex = -1.0;

    // input: 8 verts per instance so we can expand into a dual billboard
#ifdef OE_GROUNDCOVER_USE_INSTANCING
    int instanceID = gl_InstanceID;
#else
    int instanceID = gl_VertexID / 16;
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
    oe_noise = textureLod(oe_GroundCover_noiseTex, oe_layer_tilec.st, 0);

    // randomly shift each point off center
    vec2 shift = vec2(fract(oe_noise[NOISE_RANDOM]*5.5), fract(oe_noise[NOISE_RANDOM_2]*5.5))*2-1;
    oe_layer_tilec.st += shift*halfSpacing;

    // interpolate to correct position within the tile
    vertex.xyz +=
        gl_NormalMatrix *  // model to view
        vec3(mix(oe_GroundCover_LL.xy, oe_GroundCover_UR.xy, oe_layer_tilec.st), 0);

    // Sample the landcover data. Must do this BEFORE calling getBiomeIndex.
    oe_LandCover_coverage = textureLod(OE_LANDCOVER_TEX, (OE_LANDCOVER_TEX_MATRIX*oe_layer_tilec).st, 0).r;

    // Look up the biome at this point:
    int biomeIndex = oe_GroundCover_getBiomeIndex(oe_layer_tilec);
    if ( biomeIndex < 0 )
    {
        // No biome defined; bail out without emitting any geometry.
        return;
    }

    // Sample optional mask texture
#ifdef OE_GROUNDCOVER_MASK_SAMPLER
    float mask = texture(OE_GROUNDCOVER_MASK_SAMPLER, (OE_GROUNDCOVER_MASK_MATRIX*oe_layer_tilec).st).a;
    if ( mask > 0.0 )
        return;
#endif

    // Clamp the center point to the elevation.
    vertex.xyz += oe_UpVectorView * oe_terrain_getElevation(oe_layer_tilec.st);

    // Calculate the normalized camera range (oe_Camera.z = LOD Scale)
    float maxRange = oe_GroundCover_maxDistance / oe_Camera.z;
    float zv = vertex.z;
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
    float fill = biome.fill;
    if ( oe_noise[NOISE_SMOOTH] > fill )
    {
        return;
    }
    else
    {
        oe_noise[NOISE_SMOOTH] /= fill;
        if (oe_noise[NOISE_SMOOTH] > 0.75)
            fillEdgeFactor = 1.0-((oe_noise[NOISE_SMOOTH]-0.75)/0.25);
    }

    // select a billboard at "random"
    float pickNoise = (1.0-oe_noise[NOISE_SMOOTH]);
    int objectIndex = biome.firstObjectIndex + int(floor(pickNoise * float(biome.numObjects)));
    objectIndex = clamp(objectIndex, biome.firstObjectIndex, biome.firstObjectIndex + biome.numObjects - 1);

    // Recover the object we randomly picked:
    oe_GroundCover_Object object;
    oe_GroundCover_getObject(objectIndex, object);

    // for now, assume type == BILLBOARD.
    // Find the billboard associated with the object:
    oe_GroundCover_Billboard billboard;
    oe_GroundCover_getBillboard(object.objectArrayIndex, billboard);

    oe_GroundCover_atlasIndex = float(billboard.atlasIndexSide);

    // push the falloff closer to the max distance.
    float falloff = 1.0-(nRange*nRange); //*nRange);

    // a pseudo-random scale factor to the width and height of a billboard
    //float sizeScale = billboard.sizeVariation * (oe_noise[NOISE_RANDOM_2]*2.0-1.0);
    float sizeScale = billboard.sizeVariation * (oe_noise[NOISE_CLUMPY]*2.0-1.0);

    float width = (billboard.width + billboard.width*sizeScale) * clamp(fillEdgeFactor*2,0,1);

    // need abs here but not sure why... todo
    float height = abs((billboard.height + billboard.height*sizeScale) * fillEdgeFactor);

    // ratio of adjusted height to nonimal height
    float heightRatio = height/billboard.height;

    int which = gl_VertexID & 15; // mod16 - there are 16 verts per instance

    vp_Color = vec4(1,1,1,falloff);


#ifdef OE_GROUNDCOVER_USE_ACTOR
    vec4 actorPos = vec4(actorPlace, actorPlace,0,1);
    actorPos += oe_UpVectorView * oe_terrain_getElevation(vec2(0.5,0.5));
    actorPos.xyz += oe_UpVectorView*actorHeight;
#endif

    // texture coordinate:
    float row = float(which/4);
    oe_GroundCover_texCoord.t = (1.0/3.0)*row;

    // random rotation; do this is model space and then transform
    // the vector to view space.
    float a = 6.283185 * fract(oe_noise[NOISE_RANDOM_2]*5.5);
    vec2 sincos = vec2(sin(a), cos(a));
    vec3 faceVec = gl_NormalMatrix * vec3(dot(vec2(0,1), vec2(sincos.y, -sincos.x)), dot(vec2(0,1), sincos.xy), 0);

    // local frame side vector
    vec3 sideVec = cross(faceVec, oe_UpVectorView);

    if ((which&3) == 0) {
        vertex.xyz += -sideVec*width*0.5 -faceVec*width*0.1;
        oe_GroundCover_texCoord.s = 0.0;
    }
    else if (((which-1)&3) == 0) {
        vertex.xyz += -sideVec*width*0.15 +faceVec*width*0.1;
        oe_GroundCover_texCoord.s = (1.0/3.0);
    }
    else if (((which-2)&3) == 0) {
        vertex.xyz += sideVec*width*0.15 +faceVec*width*0.1;
        oe_GroundCover_texCoord.s = (2.0/3.0);
    }
    else {
        vertex.xyz += sideVec*width*0.5 -faceVec*width*0.1;
        oe_GroundCover_texCoord.s = 1.0;
    }

    // extrude to height:
    vertex.xyz += oe_UpVectorView * height * oe_GroundCover_texCoord.t;
    
    // For bending, exaggerate effect as we climb the stalk
    float bendPower = pow(oe_GroundCover_texCoord.t, 2.0);

    // effect of gravity:
    const float gravity = 0.16; // 0=no bend, 1=insane megabend
    vertex.xyz += faceVec * heightRatio * gravity * bendPower;

    // wind:
    if (oe_GroundCover_wind > 0.0)
    {
        const vec2 windFreq = vec2(0.01);
        vec2 windUV = oe_layer_tilec.xy + windFreq*osg_FrameTime;
        vec2 wind = textureLod(oe_GroundCover_noiseTex, windUV, 0).xw * 2 - 1;
        float windEffect = oe_GroundCover_wind * heightRatio * bendPower * falloff;

#if 0
        // directional wind:
        vec2 windsc = vec2(sin(shmoo), cos(shmoo));
        vec3 windVec = gl_NormalMatrix * vec3(dot(vec2(0,1), vec2(windsc.y, -windsc.x)), dot(vec2(0,1), windsc.xy), 0);
        windVec += gl_NormalMatrix * vec3(wind.x, wind.y, 0);
        vertex.xyz += windVec * windEffect;
#endif

        vertex.xyz += 
            gl_NormalMatrix * vec3(1,0,0) * windEffect * wind.x +
            gl_NormalMatrix * vec3(0,1,0) * windEffect * wind.y;
    }

    // normal:
    vp_Normal = mix(-faceVec, oe_UpVectorView, oe_GroundCover_texCoord.t/3.0);

    // AO:
    if (row == 0)
        vp_Color.rgb *= 0.2;
    else if (row == 1)
        vp_Color.rgb *= 0.6;
}


[break]
#pragma vp_name GroundCover frag shader
#pragma vp_entryPoint oe_Grass_FS
#pragma vp_location fragment

uniform sampler2DArray oe_GroundCover_billboardTex;
in vec2 oe_GroundCover_texCoord;
flat in float oe_GroundCover_atlasIndex;
vec3 vp_Normal;

void oe_Grass_FS(inout vec4 color)
{
    if (oe_GroundCover_atlasIndex < 0.0)
        discard;

    // modulate the texture
    color = texture(oe_GroundCover_billboardTex, vec3(oe_GroundCover_texCoord, oe_GroundCover_atlasIndex)) * color;

    if(color.a < 0.15)
        discard;

    if (gl_FrontFacing == false)
    // Flip the normal for backfacing polys (2-sided lighting)
    //if (dot(vp_Normal, vertex.xyz) > 0.0)
        vp_Normal = -vp_Normal;
}
