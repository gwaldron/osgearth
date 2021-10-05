#version 460

#pragma vp_name       GroundCover VS MODEL
#pragma vp_entryPoint oe_GroundCover_VS_MODEL
#pragma vp_location   vertex_model

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

void oe_GroundCover_VS_MODEL(inout vec4 geom_vertex)
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

#pragma vp_name       GroundCover VS
#pragma vp_entryPoint oe_GroundCover_VS
#pragma vp_location   vertex_view
// Set the vp_order to run earlier so that a proper view space vertex is generated from our shader so that
// default view space shaders further down the chain such as the VisibleLayer rangeOpacityVS work correctly
#pragma vp_order      0.9

#pragma include Splat.GroundCover.Types.glsl

#pragma import_defines(OE_IS_SHADOW_CAMERA)

struct oe_VertexSpec {
    //vec4 model;
    vec4 view;
    vec3 normal;
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

out vec3 oe_gc_texCoord; // Output tx coords
out mat3 oe_gc_TBN; // ref frame for normal maps
out float oe_gc_transition; // fade bb to model

uniform vec3 oe_VisibleLayer_ranges; // from VisibleLayer
uniform vec3 oe_Camera;
uniform mat4 osg_ViewMatrix;


float rescale(float d, float v0, float v1)
{
    return clamp((d-v0)/(v1-v0), 0, 1);
}

flat out uint64_t oe_gc_texHandle;
flat out uint64_t oe_gc_nmlHandle;

void oe_GroundCover_Billboard(inout vec4 vertex_view)
{
    uint i = renderLUT[ gl_InstanceID + cmd[gl_DrawID].baseInstance ];
    uint t = instance[i].tileNum;

    vp_Color = vec4(1,1,1,0); // start alpha at ZERO for billboard transitions
    oe_gc_texHandle = 0UL; // 0UL = untextured
    oe_gc_nmlHandle = 0UL; // oUL = no normal map

    oe_layer_tilec = vec4(instance[i].tilec, 0, 1);
    vertex_view = oe_vertex.view;
    oe_UpVectorView = oe_transform.normal * vec3(0,0,1);

    vec4 noise = textureLod(oe_gc_noiseTex, oe_layer_tilec.st, 0);  

    // Calculate the normalized camera range (oe_Camera.z = LOD Scale)
    float maxRange = oe_VisibleLayer_ranges[1] / oe_Camera.z;
    float nRange = clamp(-vertex_view.z/maxRange, 0.0, 1.0);

    // push the falloff closer to the max distance.
    float falloff = 1.0-(nRange*nRange*nRange);
    float width = instance[i].width * falloff;
    float height = instance[i].height * falloff;

    int which = gl_VertexID & 7; // mod8 - there are 8 verts per instance

#ifdef OE_IS_SHADOW_CAMERA

    // For a shadow camera, draw the tree as a cross hatch model instead of a billboard.
    vp_Color = vec4(1.0);
    vec3 heightVector = oe_UpVectorView*height;
    vec3 tangentVector;

    if (which < 4)
    {
        // first quad
        tangentVector = oe_transform.normal * vec3(1,0,0); // vector pointing east-ish.
    }
    else
    {
        // second quad
        tangentVector = oe_transform.normal * vec3(0,1,0);
        which -= 4;
    }

    vec3 halfWidthTangentVector = cross(tangentVector, oe_UpVectorView) * 0.5 * width;

    vertex_view.xyz =
        which==0? vertex_view.xyz - halfWidthTangentVector :
        which==1? vertex_view.xyz + halfWidthTangentVector :
        which==2? vertex_view.xyz - halfWidthTangentVector + heightVector :
        vertex_view.xyz + halfWidthTangentVector + heightVector;

    vp_Normal = normalize(cross(tangentVector, heightVector));

    if (instance[i].sideSamplerIndex >= 0)
    {
        oe_gc_texHandle = texHandle[instance[i].sideSamplerIndex];
        oe_gc_nmlHandle = texHandle[instance[i].sideSamplerIndex+1];
    }

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

    if (which < 4 && instance[i].sideSamplerIndex >= 0 && billboardAmount > 0.0) // Front-facing billboard
    {
        vertex_view = 
            which == 0? vec4(vertex_view.xyz - halfWidthTangentVector, 1.0) :
            which == 1? vec4(vertex_view.xyz + halfWidthTangentVector, 1.0) :
            which == 2? vec4(vertex_view.xyz - halfWidthTangentVector + heightVector, 1.0) :
                        vec4(vertex_view.xyz + halfWidthTangentVector + heightVector, 1.0);

        // calculates normals:
        vec3 faceNormalVector = normalize(cross(tangentVector, heightVector));

        if (billboardAmount > 0.1)
        {
            vp_Color.a = falloff * billboardAmount;

            float blend = 0.25 + (noise[NOISE_RANDOM_2]*0.25);

            vp_Normal =
                which == 0 || which == 2? mix(-tangentVector, faceNormalVector, blend) :
                mix( tangentVector, faceNormalVector, blend);

            oe_gc_texHandle = texHandle[instance[i].sideSamplerIndex];

            oe_gc_nmlHandle = texHandle[instance[i].sideSamplerIndex+1];

            // normal mapping ref frame
            oe_gc_TBN = mat3(
                tangentVector,
                cross(tangentVector, faceNormalVector),
                faceNormalVector);
        }
    }

    else if (which >= 4 && instance[i].topSamplerIndex > 0 && topDownAmount > 0.0) // top-down billboard
    {
        oe_gc_texHandle = texHandle[instance[i].topSamplerIndex];

        // estiblish the local tangent plane:
        vec3 Z = mat3(osg_ViewMatrix) * vec3(0,0,1); //north pole
        vec3 E = cross(Z, oe_UpVectorView);
        vec3 N = cross(oe_UpVectorView, E);
        Z = cross(E, N);

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

        // normal map handle and ref frame:
        oe_gc_nmlHandle = texHandle[instance[i].topSamplerIndex+1];
        oe_gc_TBN = mat3(E, -N, oe_UpVectorView);

        vp_Color.a = topDownAmount;
    }

#endif // !OE_IS_SHADOW_CAMERA

    oe_gc_texCoord.st =
        which == 0 || which == 4? vec2(0, 0) :
        which == 1 || which == 5? vec2(1, 0) :
        which == 2 || which == 6? vec2(0, 1) :
                                  vec2(1, 1);

    // apply fade from bb->model
    if (instance[i].modelId >= 0)
    {
        oe_gc_transition = clamp(
            (1.0 + PSR_BUFFER - instance[i].pixelSizeRatio) / PSR_BUFFER, 
            0, 1);
    }
}


void oe_GroundCover_Model(inout vec4 vertex_view)
{
    uint i = renderLUT[ gl_InstanceID + cmd[gl_DrawID].baseInstance ];

    oe_layer_tilec = vec4(instance[i].tilec, 0, 1);

    vertex_view = oe_vertex.view;

    vp_Normal = oe_vertex.normal;

    //TODO: hard-coded Coord7, is that OK? I guess we could use zero
    oe_gc_texCoord = gl_MultiTexCoord7.xyz;

    // assign texture sampler for this model
    if (instance[i].modelSamplerIndex >= 0)
        oe_gc_texHandle = texHandle[instance[i].modelSamplerIndex];
    else
        oe_gc_texHandle = 0UL;

    oe_gc_nmlHandle = 0UL; // no normal map

    // apply fade from bb->model
    if (instance[i].pixelSizeRatio < 1.0)
    {
        oe_gc_transition = clamp( 
            (instance[i].pixelSizeRatio - (1.0-PSR_BUFFER))/PSR_BUFFER, 
            0.0, 1.0 );
    }
}

// MAIN ENTRY POINT  
void oe_GroundCover_VS(inout vec4 vertex_view)
{
    oe_gc_transition = 1.0;

    if (gl_DrawID == 0)
    {
        oe_GroundCover_Billboard(vertex_view);
    }
    else
    {
        oe_GroundCover_Model(vertex_view);
    }
}



[break]
#version 430
#extension GL_ARB_gpu_shader_int64 : enable

#pragma vp_name       Land cover billboard texture application
#pragma vp_entryPoint oe_GroundCover_FS
#pragma vp_location   fragment_coloring

#pragma import_defines(OE_IS_SHADOW_CAMERA)

uniform float oe_gc_maxAlpha;
uniform int oe_gc_isMultisampled;

in vec3 oe_gc_texCoord;
vec3 vp_Normal;
flat in uint64_t oe_gc_texHandle;

flat in uint64_t oe_gc_nmlHandle;
in mat3 oe_gc_TBN;

in float oe_gc_transition;

void oe_GroundCover_FS(inout vec4 color)
{
    // apply the transition fade
    color.a *= oe_gc_transition;

    if (oe_gc_texHandle > 0UL)
    {
        // modulate the texture.
        // "cast" the bindless handle to a sampler array and sample it
        color *= texture(sampler2DArray(oe_gc_texHandle), oe_gc_texCoord);

        if (oe_gc_nmlHandle > 0UL)
        {
            vec4 n = texture(sampler2DArray(oe_gc_nmlHandle), oe_gc_texCoord);
            n.xyz = n.xyz*2.0-1.0;
            float curv = n.z;
            n.z = 1.0 - abs(n.x) - abs(n.y);
            float t = clamp(-n.z, 0, 1);
            n.x += (n.x > 0)? -t : t;
            n.y += (n.y > 0)? -t : t;
            vp_Normal = normalize(oe_gc_TBN * n.xyz);
        }
    }

#ifdef OE_IS_SHADOW_CAMERA
    if (color.a < oe_gc_maxAlpha)
    {
        discard;
    }
#else

    if (oe_gc_isMultisampled == 1)
    {
        // maybe use this for billboards/impostors?
        // https://medium.com/@bgolus/anti-aliased-alpha-test-the-esoteric-alpha-to-coverage-8b177335ae4f
        //color.a = (color.a - oe_gc_maxAlpha) / max(fwidth(color.a), 0.0001) + 0.5;
    }
    else if (color.a < oe_gc_maxAlpha)
    {
        //color.a = (color.a - oe_gc_maxAlpha) / max(fwidth(color.a), 0.0001) + 0.5;
        discard;
    }
#endif

    //color.rgb = (vp_Normal+1.0)*0.5;
}
