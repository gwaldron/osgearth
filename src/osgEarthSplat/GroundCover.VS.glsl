#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name       GroundCover vertex shader
#pragma vp_entryPoint oe_GroundCover_VS_MODEL
#pragma vp_location   vertex_model
#pragma vp_order      0.1

#pragma import_defines(OE_LANDCOVER_TEX)
#pragma import_defines(OE_LANDCOVER_TEX_MATRIX)
#pragma import_defines(OE_GROUNDCOVER_USE_INSTANCING)

uniform vec2 oe_GroundCover_numInstances;
uniform vec3 oe_GroundCover_LL, oe_GroundCover_UR;

// different noise texture channels:
#define NOISE_RANDOM   1
#define NOISE_RANDOM_2 2

vec3 vp_Normal;
vec4 vp_Color;

uniform sampler2D oe_GroundCover_noiseTex;
vec4 noise;  // vertex stage global

uniform sampler2D OE_LANDCOVER_TEX;
uniform mat4 OE_LANDCOVER_TEX_MATRIX;

vec4 oe_layer_tilec;
flat out float oe_LandCover_coverage;

void oe_GroundCover_VS_MODEL(inout vec4 vertex_model)
{
#ifdef OE_GROUNDCOVER_USE_INSTANCING
    int instanceID = gl_InstanceID;
#else
    int instanceID = gl_VertexID / 8;
#endif

    // Instead of using oe_layer_tilec, generate the UV tile coordinates
    // based on the current instance number
    vec2 offset = vec2(
        float(instanceID % int(oe_GroundCover_numInstances.x)),
        float(instanceID / int(oe_GroundCover_numInstances.y)));

    oe_layer_tilec = vec4(offset/(oe_GroundCover_numInstances-1), 0, 1);

    vec2 variation = 0.5 / (oe_GroundCover_numInstances-1);

    // sample the noise texture.
    noise = texture(oe_GroundCover_noiseTex, oe_layer_tilec.xy);

    oe_layer_tilec.xy += variation * vec2(noise[NOISE_RANDOM]*2-1, noise[NOISE_RANDOM_2]*2-1);
    oe_layer_tilec.xy = clamp(oe_layer_tilec.xy,0,1);

    // just use average Z across tile - doesn't matter since we are going to clamp
    float z = mix(oe_GroundCover_LL.z, oe_GroundCover_UR.z, 0.5*(oe_layer_tilec.x+oe_layer_tilec.y));
    vertex_model.xyz = mix(oe_GroundCover_LL, oe_GroundCover_UR, vec3(oe_layer_tilec.xy, z));

    vp_Normal = vec3(0,0,1);
    vp_Color = vec4(1);

    vec2 coords = (OE_LANDCOVER_TEX_MATRIX * oe_layer_tilec).st;
    oe_LandCover_coverage = textureLod(OE_LANDCOVER_TEX, coords, 0).r;
}


[break]


#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name       GroundCover vertex shader
#pragma vp_entryPoint oe_GroundCover_VS
#pragma vp_location   vertex_view

#pragma import_defines(OE_IS_SHADOW_CAMERA)
#pragma import_defines(OE_GROUNDCOVER_MASK_SAMPLER)
#pragma import_defines(OE_GROUNDCOVER_MASK_MATRIX)

// Input is 8 verts per object

uniform float osg_FrameTime;                  // Frame time (seconds) used for wind animation

uniform float oe_GroundCover_ao;              // fake ambient occlusion of ground verts (0=full)
uniform float oe_GroundCover_fill;            // percentage of points that make it through, based on noise function
uniform float oe_GroundCover_windFactor;      // wind blowing the foliage
uniform float oe_GroundCover_maxDistance;     // distance at which flora disappears
uniform float oe_GroundCover_contrast;
uniform float oe_GroundCover_brightness;

// different noise texture channels:
#define NOISE_SMOOTH   0
#define NOISE_RANDOM   1
#define NOISE_RANDOM_2 2
#define NOISE_CLUMPY   3

// Generated in model stage
vec4 oe_layer_tilec;
vec4 noise;

// Stage globals
vec3 oe_UpVectorView;
vec4 vp_Color;
vec3 vp_Normal;

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

// Generated in code
int oe_GroundCover_getBiomeIndex(in vec4);

#ifdef OE_GROUNDCOVER_MASK_SAMPLER
uniform sampler2D OE_GROUNDCOVER_MASK_SAMPLER;
uniform mat4 OE_GROUNDCOVER_MASK_MATRIX;
#endif

// Sample the elevation texture and move the vertex accordingly.
void oe_GroundCover_clamp(inout vec4 vert_view, in vec3 up, in vec2 UV)
{
    float elev = oe_terrain_getElevation( UV );
    vert_view.xyz += up*elev;
}

// Generate a wind-perturbation value
float oe_GroundCover_applyWind(float time, float factor, float randOffset)
{
    return sin(time + randOffset) * factor;
}

float oe_GroundCover_fastpow(in float x, in float y)
{
    return x / (x + y - y * x);
}

// MAIN ENTRY POINT  
void oe_GroundCover_VS(inout vec4 vertex_view)
{
    // intialize with a "no draw" value:
    oe_GroundCover_atlasIndex = -1.0;

    // discard instances based on noise value threshold (coverage). If it passes,
    // scale the noise value back up to [0..1]
    if ( noise[NOISE_SMOOTH] > oe_GroundCover_fill )
        return;
    else
        noise[NOISE_SMOOTH] /= oe_GroundCover_fill;

    vec4 tileUV = oe_layer_tilec;

    // Look up the biome at this point:
    int biomeIndex = oe_GroundCover_getBiomeIndex(tileUV);
    if ( biomeIndex < 0 )
    {
        // No biome defined; bail out without emitting any geometry.
        return;
    }

    // If we're using a mask texture, sample it now:
#ifdef OE_GROUNDCOVER_MASK_SAMPLER
    float mask = texture(OE_GROUNDCOVER_MASK_SAMPLER, (OE_GROUNDCOVER_MASK_MATRIX*tileUV).st).a;
    if ( mask > 0.0 )
    {
        // Failed to pass the mask; no geometry emitted.
        return;
    }
#endif

    // Clamp the center point to the elevation.
    oe_GroundCover_clamp(vertex_view, oe_UpVectorView, tileUV.st);

    // Calculate the normalized camera range:
    float nRange = clamp(-vertex_view.z/oe_GroundCover_maxDistance, 0.0, 1.0);

    // Distance culling:
    if ( nRange == 1.0 )
        return;

    // look up biome:
    oe_GroundCover_Biome biome;
    oe_GroundCover_getBiome(biomeIndex, biome);

    // select a billboard seemingly at random. Need to scale n to account for the fill limit first though.
    int objectIndex = biome.firstObjectIndex + int(floor(noise[NOISE_RANDOM] * float(biome.numObjects)));
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
    float sizeScale = billboard.sizeVariation * (noise[NOISE_RANDOM_2]*2.0-1.0);

    float width = (billboard.width + billboard.width*sizeScale) * falloff;

    float height = (billboard.height + billboard.height*sizeScale) * falloff;


    int which = gl_VertexID & 7; // mod8

#ifdef OE_IS_SHADOW_CAMERA

    // For a shadow camera, draw the tree as a cross hatch model
    // instead of a billboard.
    vp_Color = vec4(1,1,1,falloff);
    vec3 heightVector = oe_UpVectorView*height;
    vec3 tangentVector;

    if (which < 4)
    {
        // first quad
        tangentVector = gl_NormalMatrix * vec3(1,0,0); // vector pointing east-ish.
    }
    else
    {
        // second quad
        tangentVector = gl_NormalMatrix * vec3(0,1,0);
    }

    vec3 halfWidthTangentVector = cross(tangentVector, oe_UpVectorView) * 0.5 * width;

    vertex_view.xyz =
        which==0? vertex_view.xyz - halfWidthTangentVector :
        which==1? vertex_view.xyz + halfWidthTangentVector :
        which==2? vertex_view.xyz - halfWidthTangentVector + heightVector :
                  vertex_view.xyz + halfWidthTangentVector + heightVector;

    vp_Normal = normalize(cross(tangentVector, heightVector));


#else // normal render camera - draw as a billboard:

    vec3 tangentVector = normalize(cross(vertex_view.xyz, oe_UpVectorView));
    vec3 halfWidthTangentVector = tangentVector * 0.5 * width;
    vec3 heightVector = oe_UpVectorView*height;

    // Color variation, brightness, and contrast:
    vec3 color = vec3( noise[NOISE_RANDOM_2] );
    color = ( ((color - 0.5) * oe_GroundCover_contrast + 0.5) * oe_GroundCover_brightness);

    float billboardAmount = 1.0;
    float topDownAmount = 0.0;

    // calculate a [0..1] factor for interpolating from a front billboard view
    // to a top-down view of the tree (0.0=billboard, 1.0=topdown)
    topDownAmount = abs(dot(vec3(0, 0, -1), oe_UpVectorView));
    billboardAmount = 1.0 - pow(topDownAmount, 10.0);
    topDownAmount = clamp(topDownAmount*1.5, 0.0, 1.0);

    const float topDownThreshold = 0.5;

    if (which < 4 && billboard.atlasIndexSide >= 0) // Front-facing billboard
    {
        vertex_view = 
            which == 0? vec4(vertex_view.xyz - halfWidthTangentVector, 1.0) :
            which == 1? vec4(vertex_view.xyz + halfWidthTangentVector, 1.0) :
            which == 2? vec4(vertex_view.xyz - halfWidthTangentVector + heightVector, 1.0) :
                        vec4(vertex_view.xyz + halfWidthTangentVector + heightVector, 1.0);

        // animate based on wind parameters.
        if (which >= 2 && oe_GroundCover_windFactor > 0)
        {
            float nw = noise[NOISE_SMOOTH];
            float wind = width*oe_GroundCover_windFactor*nw;
            vertex_view.x += oe_GroundCover_applyWind(osg_FrameTime*(1+nw), wind, tileUV.s);
            vertex_view.x += oe_GroundCover_applyWind(osg_FrameTime*(1-nw), wind, tileUV.t);
        }

        // calculates normals:
        vec3 faceNormalVector = normalize(cross(tangentVector, heightVector));

        // if we are looking straight-ish down on the billboard, don't bother with it
        if (billboard.atlasIndexTop < 0 && 
            abs(dot(normalize(vertex_view.xyz), faceNormalVector)) < 0.01)
        {
            return;
        }

        const float billboardThreshold = 0.15;

        if (billboardAmount > billboardThreshold)
        {
            vp_Color = vec4(color*oe_GroundCover_ao, falloff * billboardAmount);

            float blend = 0.25 + (noise[NOISE_RANDOM_2]*0.25);

            vp_Normal =
                which == 0 || which == 2? mix(-tangentVector, faceNormalVector, blend) :
                                          mix( tangentVector, faceNormalVector, blend);

            oe_GroundCover_atlasIndex = float(billboard.atlasIndexSide);
        }
    }

    else if (which >= 4 && billboard.atlasIndexTop >= 0 && topDownAmount > topDownThreshold) // top-down billboard
    {
        oe_GroundCover_atlasIndex = float(billboard.atlasIndexTop);

        // estiblish the local tangent plane:
        vec3 U = gl_NormalMatrix * vec3(0, 0, 1);
        vec3 E = cross(U, oe_UpVectorView);
        vec3 N = cross(oe_UpVectorView, E);

        // now introduce a "random" rotation
        vec2 b = vec2(noise[NOISE_RANDOM], noise[NOISE_RANDOM_2])*2.0-1.0;
        N = normalize(E*b.x + N*b.y);
        E = normalize(cross(N, U));

        float k = width * 0.5;
        vec3 C = vertex_view.xyz + (heightVector*0.4);
        vertex_view =
            which == 4? vec4(C - E*k - N*k, 1.0) :
            which == 5? vec4(C + E*k - N*k, 1.0) :
            which == 6? vec4(C - E*k + N*k, 1.0) :
                        vec4(C + E*k + N*k, 1.0);

        vp_Normal = vertex_view.xyz - C;

        vp_Color = vec4(color, (topDownAmount - topDownThreshold)*(topDownAmount / topDownThreshold));
    }

#endif // !OE_IS_SHADOW_CAMERA

    oe_GroundCover_texCoord =
        which == 0 || which == 4? vec2(0, 0) :
        which == 1 || which == 5? vec2(1, 0) :
        which == 2 || which == 6? vec2(0, 1) :
                                  vec2(1, 1);
}
