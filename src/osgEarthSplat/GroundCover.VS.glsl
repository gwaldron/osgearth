#version 430
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name       GroundCover vertex shader
#pragma vp_entryPoint oe_GroundCover_VS
#pragma vp_location   vertex_view

#pragma import_defines(OE_GROUNDCOVER_MASK_SAMPLER)
#pragma import_defines(OE_GROUNDCOVER_MASK_MATRIX)
#pragma import_defines(OE_IS_SHADOW_CAMERA)

// Instance data from compute shader
struct RenderData
{
    vec4 vertex;      // 16
    vec2 tilec;       // 8
    int sideIndex;    // 4
    int  topIndex;    // 4
    float width;      // 4
    float height;     // 4
    float fillEdge;   // 4
    float _padding;   // 4
};

layout(binding=1, std430) readonly buffer RenderBuffer {
    RenderData render[];
};

// Noise texture:
uniform sampler2D oe_GroundCover_noiseTex;
#define NOISE_SMOOTH   0
#define NOISE_RANDOM   1
#define NOISE_RANDOM_2 2
#define NOISE_CLUMPY   3

uniform float oe_GroundCover_wind; // wind blowing the foliage
uniform float oe_GroundCover_maxDistance;     // distance at which flora disappears

uniform vec3 oe_Camera;  // (vp width, vp height, lodscale)

uniform float osg_FrameTime; // Frame time (seconds) used for wind animation
uniform mat4 osg_ViewMatrix;

// Stage globals
vec3 oe_UpVectorView;
vec4 vp_Color;
vec3 vp_Normal;
out vec4 oe_layer_tilec;
// Output grass texture coordinates to the fragment shader
out vec2 oe_GroundCover_texCoord;

// Output that selects the land cover texture from the texture array (non interpolated)
flat out float oe_GroundCover_atlasIndex;


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

    vertex_view = gl_ModelViewMatrix * render[gl_InstanceID].vertex;
    oe_layer_tilec = vec4(render[gl_InstanceID].tilec, 0, 1);

    vec4 noise = textureLod(oe_GroundCover_noiseTex, oe_layer_tilec.st, 0);  

    vp_Color = vec4(1);
    vp_Normal = vec3(0,0,1);

    // Calculate the normalized camera range (oe_Camera.z = LOD Scale)
    float maxRange = oe_GroundCover_maxDistance / oe_Camera.z;
    float nRange = clamp(-vertex_view.z/maxRange, 0.0, 1.0);

    // cull verts that are out of range. Sadly we can't do this in COMPUTE.
    if (nRange >= 0.99)
        return;

    oe_GroundCover_atlasIndex = float(render[gl_InstanceID].sideIndex);

    // push the falloff closer to the max distance.
    float falloff = 1.0-(nRange*nRange*nRange);
    float width = render[gl_InstanceID].width * falloff;
    float height = render[gl_InstanceID].width * falloff;

    int which = gl_VertexID & 7; // mod8 - there are 8 verts per instance

#ifdef OE_IS_SHADOW_CAMERA

    // For a shadow camera, draw the tree as a cross hatch model instead of a billboard.
    vp_Color = vec4(1.0);
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
    //vec3 color = vec3(0.75+0.25*noise[NOISE_RANDOM_2]);
    //color = ( ((color - 0.5) * oe_GroundCover_contrast + 0.5) * oe_GroundCover_brightness);

    float d = clamp(dot(vec3(0,0,1), oe_UpVectorView), 0, 1);
    float topDownAmount = rescale(d, 0.4, 0.6);
    float billboardAmount = rescale(1.0-d, 0.0, 0.25);

    if (which < 4 && render[gl_InstanceID].sideIndex >= 0 && billboardAmount > 0.0) // Front-facing billboard
    {
        vertex_view = 
            which == 0? vec4(vertex_view.xyz - halfWidthTangentVector, 1.0) :
            which == 1? vec4(vertex_view.xyz + halfWidthTangentVector, 1.0) :
            which == 2? vec4(vertex_view.xyz - halfWidthTangentVector + heightVector, 1.0) :
            vec4(vertex_view.xyz + halfWidthTangentVector + heightVector, 1.0);
#
        // animate based on wind parameters.
        if (which >= 2 && oe_GroundCover_wind > 0)
        {
            float nw = noise[NOISE_SMOOTH];
            float wind = width*oe_GroundCover_wind*nw;
            vertex_view.x += oe_GroundCover_applyWind(osg_FrameTime*(1+nw), wind, oe_layer_tilec.s);
            vertex_view.x += oe_GroundCover_applyWind(osg_FrameTime*(1-nw), wind, oe_layer_tilec.t);
        }

        // calculates normals:
        vec3 faceNormalVector = normalize(cross(tangentVector, heightVector));

        if (billboardAmount > 0.1)
        {
            vp_Color.a = falloff * billboardAmount;

            float blend = 0.25 + (noise[NOISE_RANDOM_2]*0.25);

            vp_Normal =
                which == 0 || which == 2? mix(-tangentVector, faceNormalVector, blend) :
                mix( tangentVector, faceNormalVector, blend);

            //oe_GroundCover_atlasIndex = float(billboard.atlasIndexSide);
            oe_GroundCover_atlasIndex = float(render[gl_InstanceID].sideIndex);
        }
    }

    else if (which >= 4 && render[gl_InstanceID].topIndex >= 0 && topDownAmount > 0.0) // top-down billboard
    {
        oe_GroundCover_atlasIndex = float(render[gl_InstanceID].topIndex);

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

        vp_Color.a = topDownAmount;
    }

#endif // !OE_IS_SHADOW_CAMERA

    oe_GroundCover_texCoord =
        which == 0 || which == 4? vec2(0, 0) :
        which == 1 || which == 5? vec2(1, 0) :
        which == 2 || which == 6? vec2(0, 1) :
        vec2(1, 1);
}
