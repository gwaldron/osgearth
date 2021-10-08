#version 460
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name       Grass VS MODEL
#pragma vp_entryPoint oe_Grass_VS_MODEL
#pragma vp_location   vertex_model
// Set the vp_order to run earlier so that a proper view space vertex is generated from our shader so that
// default view space shaders further down the chain such as the VisibleLayer rangeOpacityVS work correctly
#pragma vp_order      0.9

#pragma include Splat.GroundCover.Types.glsl

vec3 vp_Normal;
vec4 vp_Color;

struct oe_VertexSpec {
    //vec4 model;
    vec4 view;
    vec3 normal; // always in view
} oe_vertex;

struct oe_TransformSpec {
    mat4 modelview;
    mat4 projection;
    mat3 normal;
} oe_transform;

void oe_Grass_VS_MODEL(inout vec4 geom_vertex)
{
    uint i = renderLUT[ gl_InstanceID + cmd[gl_DrawID].baseInstance ];
    uint tileNum = instance[i].tileNum;

    oe_transform.modelview = tileData[tileNum].modelViewMatrix;

    // Shortcut works as long as the matrix is isotropic w.r.t. scale
    oe_transform.normal = mat3(tileData[tileNum].modelViewMatrix);

    float s = instance[i].sinrot, c = instance[i].cosrot;
    mat2 rot = mat2(c, -s, s, c);
    geom_vertex.xy = rot * geom_vertex.xy;
    vp_Normal.xy = rot * vp_Normal.xy;

    geom_vertex.xyz *= instance[i].sizeScale;

    vec4 model = vec4(instance[i].vertex.xyz + geom_vertex.xyz, 1.0);
    oe_vertex.view = oe_transform.modelview * model;
    oe_vertex.normal = oe_transform.normal * vp_Normal;

    // override the terrain's shader
    vp_Color = gl_Color;
}


[break]
#version 460
#extension GL_ARB_gpu_shader_int64 : enable

#pragma vp_name       Grass Render VS
#pragma vp_entryPoint oe_Grass_main
#pragma vp_location   vertex_view

#pragma include Splat.GroundCover.Types.glsl

struct oe_VertexSpec {
    //vec4 model;
    vec4 view;
    vec3 normal; // always in view
} oe_vertex;

struct oe_TransformSpec {
    mat4 modelview;
    mat4 projection;
    mat3 normal;
} oe_transform;

// Noise texture:
uniform sampler2D oe_gc_noiseTex;
#define NOISE_SMOOTH   0
#define NOISE_RANDOM   1
#define NOISE_RANDOM_2 2
#define NOISE_CLUMPY   3

// Stage globals
vec3 oe_UpVectorView;
vec4 vp_Color;
vec3 vp_Normal;
out vec4 oe_layer_tilec;

// Output texture coordinates to the fragment shader
out vec3 oe_gc_texCoord;
flat out uint64_t oe_gc_texHandle;

uniform float osg_FrameTime; // OSG frame time (seconds) used for wind animation

uniform vec3 oe_VisibleLayer_ranges; // distance at which flora disappears
uniform vec3 oe_Camera; // (vp width, vp height, LOD scale)

#pragma import_defines(OE_WIND_TEX, OE_WIND_TEX_MATRIX)
#ifdef OE_WIND_TEX
uniform sampler3D OE_WIND_TEX ;
uniform mat4 OE_WIND_TEX_MATRIX ;
#define MAX_WIND_SPEED 50.0  // meters per second
#endif

float decel(float x) {
    return 1.0-(1.0-x)*(1.0-x);
}

// remap x from [0..1] to [lo..hi]
float remap(float x, float lo, float hi) {
    return lo+x*(hi-lo);
}

const float browning = 0.25;

void oe_Grass_parametric(inout vec4 vertex_view)
{
    uint i = renderLUT[ gl_InstanceID + cmd[gl_DrawID].baseInstance ];
    uint t = instance[i].tileNum;

    vp_Color = vec4(1);

    oe_layer_tilec = vec4(instance[i].tilec, 0, 1);
    vertex_view = oe_vertex.view;
    oe_UpVectorView = oe_transform.normal * vec3(0,0,1);

    // Sample our noise texture
    vec4 oe_noise = textureLod(oe_gc_noiseTex, oe_layer_tilec.st, 0);
    vec4 oe_noise_wide = textureLod(oe_gc_noiseTex, oe_layer_tilec.st/16.0, 0);

    // Calculate the normalized camera range (oe_Camera.z = LOD Scale)
    float maxRange = oe_VisibleLayer_ranges[1] / oe_Camera.z;
    float nRange = clamp(-vertex_view.z/maxRange, 0.0, 1.0);

    // find the texture atlas index:
    oe_gc_texHandle = 0UL;
    if (instance[i].sideSamplerIndex >= 0)
        oe_gc_texHandle = texHandle[instance[i].sideSamplerIndex];

    // make the grass smoothly disappear in the distance
    float falloff = clamp(2.0-(nRange + oe_noise[NOISE_SMOOTH]), 0, 1);

    float width = instance[i].width * falloff;
    float height = instance[i].height * falloff;

    height = mix(-browning*height+height, browning*height+height, oe_noise_wide[NOISE_CLUMPY]);

    // ratio of adjusted height to nonimal height
    float heightRatio = height/instance[i].height;

    int which = gl_VertexID & 15; // mod16 - there are 16 verts per instance

    vp_Color = vec4(1,1,1,falloff);

    // darken as the fill level decreases
    vp_Color.rgb *= 0.5+( decel(instance[i].fillEdge)*(1.0-0.5) );

    // texture coordinate:
    float row = float(which/4);
    oe_gc_texCoord.t = (1.0/3.0)*row;

    vec3 faceVec = oe_vertex.normal;

    // local frame side vector
    vec3 sideVec = cross(faceVec, oe_UpVectorView);

    // make a curved billboard
    if ((which&3) == 0) { // col 0
        vertex_view.xyz += -sideVec*width*0.5 -faceVec*width*0.1;
        oe_gc_texCoord.s = 0.0;
    }
    else if (((which-1)&3) == 0) { // col 1
        vertex_view.xyz += -sideVec*width*0.15 +faceVec*width*0.1;
        oe_gc_texCoord.s = (1.0/3.0);
    }
    else if (((which-2)&3) == 0) { // col 2
        vertex_view.xyz += sideVec*width*0.15 +faceVec*width*0.1;
        oe_gc_texCoord.s = (2.0/3.0);
    }
    else { // col 3
        vertex_view.xyz += sideVec*width*0.5 -faceVec*width*0.1;
        oe_gc_texCoord.s = 1.0;
    }

    // extrude to height:
    vec4 vertex_base = vertex_view;

    float vertexHeight = height * oe_gc_texCoord.t;
    vertex_view.xyz += oe_UpVectorView * vertexHeight;

    // normal:
    vp_Normal = oe_UpVectorView;

    // For bending, exaggerate effect as we climb the stalk
    float bendPower = pow(3.0*oe_gc_texCoord.t+0.8, 2.0);

    // effect of gravity:
    const float gravity = 0.025; // 0=no bend, 1=insane megabend
    vec3 bendVec = faceVec * heightRatio * gravity * bendPower;

#ifdef OE_WIND_TEX
    // sample the local wind map.
    const float bendDistance = 0.25*vertexHeight;
    vec4 windData = textureProj(OE_WIND_TEX, (OE_WIND_TEX_MATRIX * vertex_base));
    vec3 windDir  = normalize(windData.rgb*2 - 1); // view space

    const float rate = 0.01;
    vec4 noise_moving = textureLod(oe_gc_noiseTex, oe_layer_tilec.st + osg_FrameTime*rate, 0);
    float windSpeedVariation = remap(noise_moving[NOISE_CLUMPY], -0.2, 1.4);
    float windSpeed = windData.a * windSpeedVariation;

    // wind turbulence - once the wind exceeds a certain speed, grass starts buffeting
    // based on a higher frequency noise function
    vec3 buffetingDir = vec3(0);
    if (windSpeed > 0.2)
    {
        float buffetingSpeed = windSpeed*0.2;
        vec4 noise_b = textureLod(oe_gc_noiseTex, oe_layer_tilec.st + osg_FrameTime*buffetingSpeed, 0);
        buffetingDir = oe_transform.normal * vec3(noise_b.xx*2-1,0) * buffetingSpeed;
    }

    bendVec += (windDir+buffetingDir) * windSpeed * bendPower * bendDistance * falloff;
#endif

    // Keep the bending under control
    float bendLen = length(bendVec);
    if (bendLen > vertexHeight)
    {
        bendVec = (bendVec/bendLen)*vertexHeight;
    }

    vertex_view.xyz += bendVec;

    // Some AO.

    vec4 ao = vp_Color;
    if (row==0)
        ao.rgb *= 0.5;
    if (row==1)
        ao.rgb /= max(1.5*heightRatio,1.0);

    vp_Color = mix(ao, vp_Color, nRange*nRange);

    // Some color variation.
    vp_Color.gb -= browning*oe_noise_wide[NOISE_SMOOTH];
}

float rescale(float d, float v0, float v1)
{
    return clamp((d-v0)/(v1-v0), 0, 1);
}

void oe_Grass_model(inout vec4 vertex_view)
{
    uint i = renderLUT[ gl_InstanceID + cmd[gl_DrawID].baseInstance ];
    oe_layer_tilec = vec4(instance[i].tilec, 0, 1);
    vertex_view = oe_vertex.view;
    vp_Normal = oe_vertex.normal;

    float psr = instance[i].pixelSizeRatio;
    vp_Color.a = clamp(psr, 0, 1);
    //if (psr < 2.0) {
    //    psr = rescale(psr-2.0, 0.0, 1.0);
    //    vp_Color.a *= psr;
    //}

    // TODO: don't hard-code this..? or meh
    oe_gc_texCoord.xyz = gl_MultiTexCoord7.xyz;

    // pick the sampler
    oe_gc_texHandle = instance[i].modelSamplerIndex >= 0?
        texHandle[instance[i].modelSamplerIndex] :
        0UL;
}

void oe_Grass_main(inout vec4 vertex_view)
{
    if (gl_DrawID == 0)
        oe_Grass_parametric(vertex_view);
    else
        oe_Grass_model(vertex_view);
}


[break]
#version 430
#extension GL_ARB_gpu_shader_int64 : enable

#pragma vp_name Grass frag shader
#pragma vp_entryPoint oe_Grass_FS
#pragma vp_location fragment

#pragma import_defines(OE_GROUNDCOVER_COLOR_SAMPLER)
#pragma import_defines(OE_GROUNDCOVER_COLOR_MATRIX)

#ifdef OE_GROUNDCOVER_COLOR_SAMPLER
uniform sampler2D OE_GROUNDCOVER_COLOR_SAMPLER ;
uniform mat4 OE_GROUNDCOVER_COLOR_MATRIX ;
in vec4 oe_layer_tilec;
#endif

in vec3 oe_gc_texCoord;
flat in uint64_t oe_gc_texHandle;
vec3 vp_Normal;

uniform float oe_gc_maxAlpha;
uniform int oe_gc_isMultisampled;

void oe_Grass_FS(inout vec4 color)
{
    if (oe_gc_texHandle > 0UL)
    {
        // paint the texture
        color *= texture(sampler2DArray(oe_gc_texHandle), oe_gc_texCoord);
    }

    // uncomment to see triangles
    //if (color.a < 0.2)
    //    color.a = 0.8;

    if (oe_gc_isMultisampled == 1)
    {
        // https://medium.com/@bgolus/anti-aliased-alpha-test-the-esoteric-alpha-to-coverage-8b177335ae4f
        //color.a = (color.a - oe_gc_maxAlpha) / max(fwidth(color.a), 0.0001) + 0.5;
    }
    else if (color.a < oe_gc_maxAlpha)
    {
        //color.a = (color.a - oe_gc_maxAlpha) / max(fwidth(color.a), 0.0001) + 0.5;
        discard;
    }

#ifdef OE_GROUNDCOVER_COLOR_SAMPLER
    const float modulation = 0.75;
    float mono = (color.r*0.2126 + color.g*0.7152 + color.b*0.0722);
    vec4 mod_color = texture(OE_GROUNDCOVER_COLOR_SAMPLER, (OE_GROUNDCOVER_COLOR_MATRIX*oe_layer_tilec).st);
    color.rgb = mix(color.rgb, mod_color.rgb*vec3(mono)*2.0, modulation);
#endif
}
