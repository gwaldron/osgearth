#version 430
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name       GroundCover vertex shader
#pragma vp_entryPoint oe_Grass_VS
#pragma vp_location   vertex_view

// Instance data from compute shader
struct RenderData // vec4 aligned please
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

//uncomment to activate
//#define OE_GROUNDCOVER_USE_ACTOR

// Noise texture:
uniform sampler2D oe_GroundCover_noiseTex;
#define NOISE_SMOOTH   0
#define NOISE_RANDOM   1
#define NOISE_RANDOM_2 2
#define NOISE_CLUMPY   3

vec3 vp_Normal;
vec4 vp_Color;
out vec4 oe_layer_tilec;
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


                        // Output grass texture coords to the FS
out vec2 oe_GroundCover_texCoord;

// Output that selects the land cover texture from the texture array (flat)
flat out float oe_GroundCover_atlasIndex;

float decel(float x) {
    return 1.0-(1.0-x)*(1.0-x);
}

void oe_Grass_VS(inout vec4 vertex)
{
    // intialize with a "no draw" value:
    oe_GroundCover_atlasIndex = -1.0;

    vertex = gl_ModelViewMatrix * render[gl_InstanceID].vertex;
    oe_layer_tilec = vec4(render[gl_InstanceID].tilec, 0, 1);

    // Sample our noise texture
    vec4 oe_noise = textureLod(oe_GroundCover_noiseTex, oe_layer_tilec.st, 0);

    // Calculate the normalized camera range (oe_Camera.z = LOD Scale)
    float maxRange = oe_GroundCover_maxDistance / oe_Camera.z;
    float zv = vertex.z;
    float nRange = clamp(-zv/maxRange, 0.0, 1.0);

    // cull verts that are out of range. Sadly we can't do this in COMPUTE.
    if (nRange >= 0.99)
        return;

    oe_GroundCover_atlasIndex = float(render[gl_InstanceID].sideIndex);

    // push the falloff closer to the max distance.
    float falloff = 1.0-(nRange*nRange*nRange);

    float width = render[gl_InstanceID].width * clamp(render[gl_InstanceID].fillEdge*2.0, 0, 1);
    float height = render[gl_InstanceID].height * render[gl_InstanceID].fillEdge * falloff;

    // ratio of adjusted height to nonimal height
    float heightRatio = height/render[gl_InstanceID].height;

    int which = gl_VertexID & 15; // mod16 - there are 16 verts per instance

    vp_Color = vec4(1,1,1,falloff);

    // darken as the fill level decreases
    vp_Color.rgb *= 0.5+( decel(render[gl_InstanceID].fillEdge)*(1.0-0.5) );

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
uniform int oe_GroundCover_A2C;

void oe_Grass_FS(inout vec4 color)
{
    if (oe_GroundCover_atlasIndex < 0.0)
        discard;

    // paint the texture
    color = texture(oe_GroundCover_billboardTex, vec3(oe_GroundCover_texCoord, oe_GroundCover_atlasIndex)) * color;

    if (oe_GroundCover_A2C == 1)
    {
        // https://medium.com/@bgolus/anti-aliased-alpha-test-the-esoteric-alpha-to-coverage-8b177335ae4f
        color.a = (color.a - oe_GroundCover_maxAlpha) / max(fwidth(color.a), 0.0001) + 0.5;
    }
    else if (color.a < oe_GroundCover_maxAlpha)
    {
        discard;
    }

#ifdef OE_GROUNDCOVER_COLOR_SAMPLER
    const float modulation = 0.75;
    float mono = (color.r*0.2126 + color.g*0.7152 + color.b*0.0722);
    vec4 mod_color = texture(OE_GROUNDCOVER_COLOR_SAMPLER, (OE_GROUNDCOVER_COLOR_MATRIX*oe_layer_tilec).st);
    color.rgb = mix(color.rgb, mod_color.rgb*vec3(mono)*2.0, modulation);
#endif
}
