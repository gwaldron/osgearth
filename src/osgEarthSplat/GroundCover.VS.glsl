#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name       GroundCover vertex shader
#pragma vp_entryPoint oe_GroundCover_VS
#pragma vp_location   vertex_view

#pragma import_defines(OE_GROUNDCOVER_USE_INSTANCING)
#pragma import_defines(OE_LANDCOVER_TEX)
#pragma import_defines(OE_LANDCOVER_TEX_MATRIX)
#pragma import_defines(OE_GROUNDCOVER_MASK_SAMPLER)
#pragma import_defines(OE_GROUNDCOVER_MASK_MATRIX)
#pragma import_defines(OE_IS_SHADOW_CAMERA)

// Noise texture:
uniform sampler2D oe_GroundCover_noiseTex;

// noise texture channels:
#define NOISE_SMOOTH   0
#define NOISE_RANDOM   1
#define NOISE_RANDOM_2 2
#define NOISE_CLUMPY   3

// LandCover texture
uniform sampler2D OE_LANDCOVER_TEX;
uniform mat4 OE_LANDCOVER_TEX_MATRIX;
out float oe_LandCover_coverage;

#ifdef OE_GROUNDCOVER_MASK_SAMPLER
uniform sampler2D OE_GROUNDCOVER_MASK_SAMPLER;
uniform mat4 OE_GROUNDCOVER_MASK_MATRIX;
#endif

uniform vec2 oe_GroundCover_numInstances;
uniform vec3 oe_GroundCover_LL, oe_GroundCover_UR;

// 0=draw a textured billboard; 1=draw an instanced model
uniform int oe_GroundCover_instancedModel;

uniform float oe_GroundCover_ao;              // fake ambient occlusion of ground verts (0=full)
uniform float oe_GroundCover_fill;            // percentage of points that make it through, based on noise function
uniform float oe_GroundCover_windFactor;      // wind blowing the foliage
uniform float oe_GroundCover_maxDistance;     // distance at which flora disappears
uniform float oe_GroundCover_contrast;
uniform float oe_GroundCover_brightness;

uniform vec3 oe_Camera; // (vp width, vp height, lodscale)

// VRV: use a custom LOD scale uniform
#pragma import_defines(VRV_OSG_LOD_SCALE)
#ifdef VRV_OSG_LOD_SCALE
uniform float VRV_OSG_LOD_SCALE;
#else
#define VRV_OSG_LOD_SCALE oe_Camera.z
#endif

uniform float osg_FrameTime; // Frame time (seconds) used for wind animation
uniform mat4 osg_ViewMatrix;

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

// Generated in GroundCover.cpp
int oe_GroundCover_getBiomeIndex(in vec4);

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

float rescale(float d, float v0, float v1)
{
    return clamp((d-v0)/(v1-v0), 0, 1);
}

vec2 quantize(vec2 x, vec2 y)
{
    return (floor(x / y) * y);
}

vec4 texture2D_bilinear_mesa(in sampler2D sampler, in vec2 tex_coord)
{
    const vec2 tex_size = vec2(256.0);
    vec2 unit_texel = 1.0 / tex_size;
    vec2 unnorm_tex_coord = (tex_coord * tex_size) - vec2(0.5);
    vec2 f = fract(unnorm_tex_coord);
    vec2 snap_tex_coord = (floor(unnorm_tex_coord) + vec2(0.5)) / tex_size;
    vec4 s1 = texture2D(sampler, snap_tex_coord);
    vec4 s2 = texture2D(sampler, snap_tex_coord + vec2(unit_texel.x, 0.));
    vec4 s3 = texture2D(sampler, snap_tex_coord + vec2(0., unit_texel.y));
    vec4 s4 = texture2D(sampler, snap_tex_coord + unit_texel);
    return mix(mix(s1, s2, f.x), mix(s3, s4, f.x), f.y);
}

// MAIN ENTRY POINT  
void oe_GroundCover_VS(inout vec4 vertex_view)
{
    // intialize with a "no draw" value (consider using a compute/gs cull instead)
    oe_GroundCover_atlasIndex = -1.0;

    int instanceID;
    if (oe_GroundCover_instancedModel == 1)
    {
        instanceID = gl_InstanceID;
    }
    else
    {
#ifdef OE_GROUNDCOVER_USE_INSTANCING
        instanceID = gl_InstanceID;
#else
        instanceID = gl_VertexID / 8;
#endif
    }

    // Generate the UV tile coordinates (tilec) based on the instance number
    vec2 offset = vec2(
        float(instanceID % int(oe_GroundCover_numInstances.x)),
        float(instanceID / int(oe_GroundCover_numInstances.y)));

    // half the distance between cell centers
    vec2 halfSpacing = 0.5 / oe_GroundCover_numInstances;

    // tile coords [0..1]
    vec4 tilec = vec4(halfSpacing + offset / oe_GroundCover_numInstances, 0, 1);

    vec4 noise = textureLod(oe_GroundCover_noiseTex, tilec.st, 0);
    //vec4 noise = vec4(0);
    //vec4 noise = texture2D_bilinear_mesa(oe_GroundCover_noiseTex, tilec.st); // validate

    // discard instances based on noise value threshold (coverage). If it passes,
    // scale the noise value back up to [0..1]
    //if (noise[NOISE_SMOOTH] > oe_GroundCover_fill)
    //    return;
    //else
    //    noise[NOISE_SMOOTH] /= oe_GroundCover_fill;

    // randomly shift each point off center
    vec2 shift = vec2(fract(noise[NOISE_RANDOM]*1.5), fract(noise[NOISE_RANDOM_2]*1.5))*2.0-1.0;

    tilec.xy += shift * halfSpacing;

    // and place it correctly within the tile
    vec3 pos = gl_NormalMatrix * vec3(mix(oe_GroundCover_LL.xy, oe_GroundCover_UR.xy, tilec.xy), 0);

    vertex_view.xyz += pos;

    if (oe_GroundCover_instancedModel == 0)
    {
        vp_Color = vec4(1, 1, 1, 0);
    }

    // sample the landcover data
    oe_LandCover_coverage = textureLod(OE_LANDCOVER_TEX, (OE_LANDCOVER_TEX_MATRIX*tilec).st, 0).r;

    // Look up the biome at this point:
    int biomeIndex = oe_GroundCover_getBiomeIndex(tilec);
    if ( biomeIndex < 0 )
    {
        // No biome defined; bail out without emitting any geometry.
        return;
    }

    // If we're using a mask texture, sample it now:
#ifdef OE_GROUNDCOVER_MASK_SAMPLER
    float mask = texture(OE_GROUNDCOVER_MASK_SAMPLER, (OE_GROUNDCOVER_MASK_MATRIX*tilec).st).a;
    if ( mask > 0.0 )
    {
        // Failed to pass the mask; no geometry emitted.
        return;
    }
#endif

    // look up biome:
    oe_GroundCover_Biome biome;
    oe_GroundCover_getBiome(biomeIndex, biome);

    // discard instances based on noise value threshold (coverage). If it passes,
    // scale the noise value back up to [0..1]
    if (noise[NOISE_SMOOTH] > biome.fill)
        return;
    else
        noise[NOISE_SMOOTH] /= biome.fill;

    // Clamp the center point to the elevation.
    oe_GroundCover_clamp(vertex_view, oe_UpVectorView, tilec.st);

    // Calculate the normalized camera range (oe_Camera.z = LOD Scale)
    float maxRange = oe_GroundCover_maxDistance / VRV_OSG_LOD_SCALE;
    float nRange = clamp(-vertex_view.z/maxRange, 0.0, 1.0);

    // Distance culling:
    if ( nRange == 1.0 )
        return;

    // Instancing? We are finished
    if (oe_GroundCover_instancedModel == 1)
    {
        oe_GroundCover_atlasIndex = 1.0;
        return;
    }

    // select a billboard at random
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

    int which = gl_VertexID & 7; // mod8 - there are 8 verts per instance

#ifdef OE_IS_SHADOW_CAMERA

    // For a shadow camera, draw the tree as a cross hatch model instead of a billboard.
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
    vec3 color = vec3(0.75+0.25*noise[NOISE_RANDOM_2]);
    //color = ( ((color - 0.5) * oe_GroundCover_contrast + 0.5) * oe_GroundCover_brightness);

    float d = clamp(dot(vec3(0,0,1), oe_UpVectorView), 0, 1);
    float topDownAmount = rescale(d, 0.4, 0.6);
    float billboardAmount = rescale(1.0-d, 0.0, 0.25);

    if (which < 4 && billboard.atlasIndexSide >= 0 && billboardAmount > 0.0) // Front-facing billboard
    {
        vertex_view = 
            which == 0? vec4(vertex_view.xyz - halfWidthTangentVector, 1.0) :
            which == 1? vec4(vertex_view.xyz + halfWidthTangentVector, 1.0) :
            which == 2? vec4(vertex_view.xyz - halfWidthTangentVector + heightVector, 1.0) :
            vec4(vertex_view.xyz + halfWidthTangentVector + heightVector, 1.0);
#
        // animate based on wind parameters.
        if (which >= 2 && oe_GroundCover_windFactor > 0)
        {
            float nw = noise[NOISE_SMOOTH];
            float wind = width*oe_GroundCover_windFactor*nw;
            vertex_view.x += oe_GroundCover_applyWind(osg_FrameTime*(1+nw), wind, tilec.s);
            vertex_view.x += oe_GroundCover_applyWind(osg_FrameTime*(1-nw), wind, tilec.t);
        }

        // calculates normals:
        vec3 faceNormalVector = normalize(cross(tangentVector, heightVector));

        if (billboardAmount > 0.1)
        {
            vp_Color = vec4(color*oe_GroundCover_ao, falloff * billboardAmount);

            float blend = 0.25 + (noise[NOISE_RANDOM_2]*0.25);

            vp_Normal =
                which == 0 || which == 2? mix(-tangentVector, faceNormalVector, blend) :
                mix( tangentVector, faceNormalVector, blend);

            oe_GroundCover_atlasIndex = float(billboard.atlasIndexSide);
        }
    }

    else if (which >= 4 && billboard.atlasIndexTop >= 0 && topDownAmount > 0.0) // top-down billboard
    {
        oe_GroundCover_atlasIndex = float(billboard.atlasIndexTop);

        // estiblish the local tangent plane:
        vec3 Z = mat3(osg_ViewMatrix) * vec3(0,0,1); //north pole
        vec3 E = cross(Z, oe_UpVectorView);
        vec3 N = cross(oe_UpVectorView, E);

        // now introduce a "random" rotation
        vec2 b = normalize(clamp(vec2(noise[NOISE_RANDOM], noise[NOISE_RANDOM_2]), 0.01, 1.0)*2.0-1.0);
        N = normalize(E*b.x + N*b.y);
        E = normalize(cross(N, oe_UpVectorView));

        // a little trick to mitigate z-fighting amongst the topdowns.
        float yclip = noise[NOISE_RANDOM] * 0.1;

        float k = width * 0.5;
        vec3 C = vertex_view.xyz + (heightVector*(0.4+yclip));
        vertex_view =
            which == 4? vec4(C - E*k - N*k, 1.0) :
            which == 5? vec4(C + E*k - N*k, 1.0) :
            which == 6? vec4(C - E*k + N*k, 1.0) :
                        vec4(C + E*k + N*k, 1.0);

        vp_Normal = vertex_view.xyz - C;

        vp_Color = vec4(color, topDownAmount);
    }

#endif // !OE_IS_SHADOW_CAMERA

    oe_GroundCover_texCoord =
        which == 0 || which == 4? vec2(0, 0) :
        which == 1 || which == 5? vec2(1, 0) :
        which == 2 || which == 6? vec2(0, 1) :
        vec2(1, 1);
}
