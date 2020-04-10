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


#pragma import_defines(OE_GROUNDCOVER_COLOR_SAMPLER)
#pragma import_defines(OE_GROUNDCOVER_COLOR_MATRIX)
#ifdef OE_GROUNDCOVER_COLOR_SAMPLER
uniform sampler2D OE_GROUNDCOVER_COLOR_SAMPLER ;
uniform mat4 OE_GROUNDCOVER_COLOR_MATRIX ;
#endif

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
uniform float actorRadius;
uniform float actorHeight;
//uniform float actorPlace;
uniform vec3 actorPos;
uniform mat4 osg_ViewMatrix;
#endif

uniform vec3 oe_Camera; // (vp width, vp height, LOD scale)

// VRV: use a custom LOD scale uniform
#pragma import_defines(VRV_OSG_LOD_SCALE)
#ifdef VRV_OSG_LOD_SCALE
uniform float VRV_OSG_LOD_SCALE;
#else
#define VRV_OSG_LOD_SCALE oe_Camera.z
#endif

// Output grass texture coords to the FS
out vec2 oe_GroundCover_texCoord;

// Output that selects the land cover texture from the texture array (flat)
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

// https://stackoverflow.com/a/17897228/4218920
vec3 rgb2hsv(vec3 c)
{
    const vec4 K = vec4(0.0, -1.0 / 3.0, 2.0 / 3.0, -1.0);
    vec4 p = mix(vec4(c.bg, K.wz), vec4(c.gb, K.xy), step(c.b, c.g));
    vec4 q = mix(vec4(p.xyw, c.r), vec4(c.r, p.yzx), step(p.x, c.r));
    float d = q.x - min(q.w, q.y);
    const float e = 1.0e-10;
    return vec3(abs(q.z + (q.w - q.y) / (6.0 * d + e)), d / (q.x + e), q.x);
}

const float oe_grass_hue = 80.0; // HSL hue value
const float oe_grass_hueWidth = 0.27;
const float oe_grass_saturation = 0.32;

float accel(float x) {
    return x*x;
}
float decel(float x) {
    return 1.0-(1.0-x)*(1.0-x);
}

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

    // Look up the biome and bail if not defined
    int biomeIndex = oe_GroundCover_getBiomeIndex(oe_layer_tilec);
    if ( biomeIndex < 0 )
        return;

    // Sample optional mask texture
#ifdef OE_GROUNDCOVER_MASK_SAMPLER
    float mask = texture(OE_GROUNDCOVER_MASK_SAMPLER, (OE_GROUNDCOVER_MASK_MATRIX*oe_layer_tilec).st).a;
    if ( mask > 0.0 )
        return;
#endif

    // Clamp the center point to the elevation.
    vertex.xyz += oe_UpVectorView * oe_terrain_getElevation(oe_layer_tilec.st);

    // Calculate the normalized camera range (oe_Camera.z = LOD Scale)
    float maxRange = oe_GroundCover_maxDistance / VRV_OSG_LOD_SCALE;
    float zv = vertex.z;
    float nRange = clamp(-zv/maxRange, 0.0, 1.0);

    // Distance culling:
    if ( nRange == 1.0 )
        return;

    // look up biome:
    oe_GroundCover_Biome biome;
    oe_GroundCover_getBiome(biomeIndex, biome);

    // discard instances based on noise value threshold (fill).

    float fill = biome.fill;

    float fillEdgeFactor = 1.0;

#ifdef OE_GROUNDCOVER_COLOR_SAMPLER
    //fill = 1.0; // for color sampling
    vec4 c = texture(OE_GROUNDCOVER_COLOR_SAMPLER, (OE_GROUNDCOVER_COLOR_MATRIX*oe_layer_tilec).st);
    vec3 hsv = rgb2hsv(c.rgb);
    float hue_dot = -cos((oe_grass_hue/360.0)*6.2831853); // [-1..1]
    float hsv_dot = -cos(hsv[0]*6.2831853); // [-1..1]
    float hue_delta = 0.5*abs(hue_dot - hsv_dot); // [0..1]
    if (oe_grass_hueWidth < hue_delta) {
        float f = (oe_grass_hueWidth/hue_delta);
        fill *= f*f*f;
    }
    if (hsv[1] < oe_grass_saturation) {
        float f = (hsv[1]/oe_grass_saturation);
        fill *= f*f;
    }
#endif

    if ( oe_noise[NOISE_SMOOTH] > fill )
    {
        return;
    }
    else
    {
        // scale the smooth-noise back up to [0..1] and compute an edge factor
        // that will shrink the foliage near the fill boundaries
        oe_noise[NOISE_SMOOTH] /= fill;
        const float xx = 0.5;
        if (oe_noise[NOISE_SMOOTH] > xx)
            fillEdgeFactor = 1.0-((oe_noise[NOISE_SMOOTH]-xx)/(1.0-xx));
    }

    // select a billboard at "random" .. TODO: still order-dependent; needs work
    float pickNoise = (1.0-oe_noise[NOISE_SMOOTH]);
    int objectIndex = biome.firstObjectIndex + int(floor(pickNoise * float(biome.numObjects)));
    objectIndex = clamp(objectIndex, biome.firstObjectIndex, biome.firstObjectIndex + biome.numObjects - 1);

    // Recover the object we randomly picked and its billboard
    oe_GroundCover_Object object;
    oe_GroundCover_getObject(objectIndex, object);
    oe_GroundCover_Billboard billboard;
    oe_GroundCover_getBillboard(object.objectArrayIndex, billboard);

    oe_GroundCover_atlasIndex = float(billboard.atlasIndexSide);

    // push the falloff closer to the max distance.
    float falloff = 1.0-(nRange*nRange*nRange);

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

    // darken as the fill level decreases
    vp_Color.rgb *= decel(fillEdgeFactor);

    // texture coordinate:
    float row = float(which/4);
    oe_GroundCover_texCoord.t = (1.0/3.0)*row;

    // random rotation; do this is model space and then transform
    // the vector to view space.
    float a = 6.283185 * fract(oe_noise[NOISE_RANDOM_2]*5.5);
    vec3 faceVec = gl_NormalMatrix * vec3(-sin(a), cos(a), 0);

    // local frame side vector
    vec3 sideVec = cross(faceVec, oe_UpVectorView);

    // make a curved billboard
    if ((which&3) == 0) { // col 0
        vertex.xyz += -sideVec*width*0.5 -faceVec*width*0.1;
        oe_GroundCover_texCoord.s = 0.0;
    }
    else if (((which-1)&3) == 0) { // col 1
        vertex.xyz += -sideVec*width*0.15 +faceVec*width*0.1;
        oe_GroundCover_texCoord.s = (1.0/3.0);
    }
    else if (((which-2)&3) == 0) { // col 2
        vertex.xyz += sideVec*width*0.15 +faceVec*width*0.1;
        oe_GroundCover_texCoord.s = (2.0/3.0);
    }
    else { // col 3
        vertex.xyz += sideVec*width*0.5 -faceVec*width*0.1;
        oe_GroundCover_texCoord.s = 1.0;
    }

    // extrude to height:
    vertex.xyz += oe_UpVectorView * height * oe_GroundCover_texCoord.t;

    // normal:
    vp_Normal = oe_UpVectorView;

    // For bending, exaggerate effect as we climb the stalk
    vec3 bendVec = vec3(0.0);
    float bendPower = pow(3.0*oe_GroundCover_texCoord.t, 2.0);

    // effect of gravity:
    const float gravity = 0.025; // 0=no bend, 1=insane megabend
    bendVec += faceVec * heightRatio * gravity * bendPower;

    // wind:
    if (oe_GroundCover_wind > 0.0)
    {
        float windEffect = oe_GroundCover_wind * heightRatio * bendPower * 0.2 * falloff;

#ifdef OE_GROUNDCOVER_USE_ACTOR
        vec3 windPos = (osg_ViewMatrix * vec4(actorPos, 1)).xyz;
        windPos += oe_UpVectorView * actorHeight;

        // macro:
        vec3 windvec = vertex.xyz - windPos;
        float attenuation = clamp(actorRadius/length(windvec), 0, 1);
        attenuation *= attenuation;
        bendVec += normalize(windvec) * windEffect * attenuation;

        // micro turbulence
        vec2 turbUV = oe_layer_tilec.xy + (1.0-oe_GroundCover_wind)*osg_FrameTime;
        vec2 turb = textureLod(oe_GroundCover_noiseTex, turbUV, 0).xw * 2 - 1;
        bendVec += gl_NormalMatrix * vec3(turb.xy, 0) * windEffect * attenuation;
#else
        const vec2 turbFreq = vec2(0.01);
        vec2 turbUV = oe_layer_tilec.xy + turbFreq*osg_FrameTime;
        vec2 turb = textureLod(oe_GroundCover_noiseTex, turbUV, 0).xw * 2 - 1;
        bendVec += gl_NormalMatrix * vec3(turb.xy, 0) * windEffect;
#endif
    }

    vertex.xyz += bendVec;

    // makeshift AO:
    if (row == 0)
        vp_Color.rgb *= 0.75;
    else if (row == 1)
        vp_Color.rgb *= 0.9;

    //oe_GroundCover_texCoord.t *= 0.4;
}


[break]
#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT
#pragma vp_name GroundCover frag shader
#pragma vp_entryPoint oe_Grass_FS
#pragma vp_location fragment

#pragma import_defines(OE_GROUNDCOVER_COLOR_SAMPLER)
#pragma import_defines(OE_GROUNDCOVER_COLOR_MATRIX)
#ifdef OE_GROUNDCOVER_COLOR_SAMPLER
uniform sampler2D OE_GROUNDCOVER_COLOR_SAMPLER ;
uniform mat4 OE_GROUNDCOVER_COLOR_MATRIX ;
in vec4 oe_layer_tilec;
#endif

uniform sampler2DArray oe_GroundCover_billboardTex;
in vec2 oe_GroundCover_texCoord;
flat in float oe_GroundCover_atlasIndex;
vec3 vp_Normal;

uniform float oe_GroundCover_maxAlpha;

void oe_Grass_FS(inout vec4 color)
{
    if (oe_GroundCover_atlasIndex < 0.0)
        discard;

    // paint the texture
    color = texture(oe_GroundCover_billboardTex, vec3(oe_GroundCover_texCoord, oe_GroundCover_atlasIndex)) * color;

    if(color.a < oe_GroundCover_maxAlpha) //0.15)
        discard;

    // VRV_PATCH
    // support double-sided geometry
    // counteract the normal-flipping in VRV's lighting shader
    if (gl_FrontFacing == false)
        vp_Normal = -vp_Normal;

#ifdef OE_GROUNDCOVER_COLOR_SAMPLER
    const float modulation = 0.75;
    float mono = (color.r*0.2126 + color.g*0.7152 + color.b*0.0722);
    vec4 mod_color = texture(OE_GROUNDCOVER_COLOR_SAMPLER, (OE_GROUNDCOVER_COLOR_MATRIX*oe_layer_tilec).st);
    color.rgb = mix(color.rgb, mod_color.rgb*vec3(mono)*2.0, modulation);
#endif
}
