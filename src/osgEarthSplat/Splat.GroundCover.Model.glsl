#pragma vp_name       GroundCover model shader VS
#pragma vp_entryPoint oe_GroundCover_Model_VS
#pragma vp_location   vertex_model

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

layout(binding=1, std430) readonly buffer RenderBuffer
{
    RenderData render[];
};

// Noise texture:
uniform sampler2D oe_GroundCover_noiseTex;
#define NOISE_SMOOTH   0
#define NOISE_RANDOM   1
#define NOISE_RANDOM_2 2
#define NOISE_CLUMPY   3

uniform float oe_GroundCover_maxDistance;     // distance at which flora disappears

uniform vec3 oe_Camera;  // (vp width, vp height, lodscale)

out vec4 oe_layer_tilec;

// Output texture coordinates to the fragment shader
out vec3 oe_GroundCover_texCoord;

// Output that selects the land cover texture from the texture array (non interpolated)
out float oe_GroundCover_falloff;

out vec3 vp_Normal;

// MAIN ENTRY POINT  
void oe_GroundCover_Model_VS(inout vec4 vertex)
{
    // intialize with a "no draw" value
    //oe_GroundCover_atlasIndex = 0.0;

    vec4 noise = texture(oe_GroundCover_noiseTex, render[gl_InstanceID].tilec);

    float a = 3.1415927 * 2.0 * noise[2];
    float s = sin(a), c = cos(a);
    mat2 rot = mat2(c, -s, s, c);
    vertex.xy = rot * vertex.xy;

    vp_Normal.xy = rot * vp_Normal.xy;

    // add anchor vertex to current vertex:
    vertex.xyz += render[gl_InstanceID].vertex.xyz;

    // Calculate the normalized camera range (oe_Camera.z = LOD Scale)
    float maxRange = oe_GroundCover_maxDistance / oe_Camera.z;
    float nRange = clamp(-vertex.z/maxRange, 0.0, 1.0);

    //// cull verts that are out of range. Sadly we can't do this in COMPUTE.
    //if (nRange >= 0.99)
    //    return;

    // push the falloff closer to the max distance.
    oe_GroundCover_falloff = 1.0-(nRange*nRange);

    oe_GroundCover_texCoord = gl_MultiTexCoord7.xyz;

#ifdef OE_IS_SHADOW_CAMERA

#else // normal render camera

#endif // !OE_IS_SHADOW_CAMERA

}


[break]
#pragma vp_name       GroundCover Model FS
#pragma vp_entryPoint oe_GroundCover_Model_FS
#pragma vp_location   fragment_coloring

#pragma import_defines(OE_IS_SHADOW_CAMERA)

in float oe_GroundCover_falloff;
in float oe_layer_opacity;

uniform int oe_GroundCover_A2C;
uniform float oe_GroundCover_maxAlpha;

uniform sampler2DArray oe_GroundCover_atlas;
in vec3 oe_GroundCover_texCoord;

void oe_GroundCover_Model_FS(inout vec4 color)
{
    color = texture(oe_GroundCover_atlas, oe_GroundCover_texCoord);

    color.a *= (oe_GroundCover_falloff * oe_layer_opacity);

#ifdef OE_IS_SHADOW_CAMERA
    //if (color.a < oe_GroundCover_maxAlpha)
    //{
    //    discard;
    //}
#else
    if (oe_GroundCover_A2C == 1)
    {
        // Remove the "wispyness" you get from multisampling
        // https://medium.com/@bgolus/anti-aliased-alpha-test-the-esoteric-alpha-to-coverage-8b177335ae4f
        color.a = (color.a - oe_GroundCover_maxAlpha) / max(fwidth(color.a), 0.0001) + 0.5;
    }
    else if (color.a < oe_GroundCover_maxAlpha)
    {
        discard;
    }
#endif
}
