#version 460

#pragma vp_name       GroundCover VS MODEL
#pragma vp_entryPoint oe_GroundCover_VS_MODEL
#pragma vp_location   vertex_model

#pragma include Procedural.Vegetation.Types.glsl

vec3 vp_Normal;
vec4 vp_Color;
out vec3 oe_UpVectorView;
out float elev;

struct oe_VertexSpec {
    vec3 local;  // instance-local vert
    vec4 view;   // view space vert
    vec3 normal; // view space normal
} oe_vertex;

struct oe_TransformSpec {
    mat4 modelview;
    mat4 projection;
    mat3 normal;
} oe_transform;

void oe_GroundCover_VS_MODEL(inout vec4 geom_vertex)
{
    RenderLeaf leaf = renderSet[gl_InstanceID + cmd[gl_DrawID].baseInstance];
    uint i = leaf.instance;
    uint tileNum = instance[i].tileNum;

    oe_transform.modelview = tile[tileNum].modelViewMatrix;

    // Shortcut works as long as the matrix is isotropic w.r.t. scale
    oe_transform.normal = mat3(tile[tileNum].modelViewMatrix);

    float s = instance[i].sinrot, c = instance[i].cosrot;
    mat2 rot = mat2(c, -s, s, c);
    geom_vertex.xy = rot * geom_vertex.xy;
    vp_Normal.xy = rot * vp_Normal.xy;

    geom_vertex.xyz *= instance[i].sizeScale;

    oe_vertex.local = geom_vertex.xyz;
    oe_vertex.view = oe_transform.modelview * vec4(instance[i].vertex.xyz + geom_vertex.xyz, 1.0);
    oe_vertex.normal = oe_transform.normal * vp_Normal;

    // override the terrain's shader
    vp_Color = gl_Color;

    elev = instance[i].vertex.z;
}


[break]
#version 460
#extension GL_ARB_gpu_shader_int64 : enable

#pragma vp_name       GroundCover VS
#pragma vp_entryPoint oe_GroundCover_VS
#pragma vp_location   vertex_view

#pragma include Procedural.Vegetation.Types.glsl

#pragma import_defines(OE_IS_SHADOW_CAMERA)

struct oe_VertexSpec {
    vec3 local;  // instance-local vert
    vec4 view;   // view space vert
    vec3 normal; // view space normal
} oe_vertex;

struct oe_TransformSpec {
    mat4 modelview;
    mat4 projection;
    mat3 normal;
} oe_transform;

// Noise texture:
uniform sampler2D oe_veg_noiseTex;
#define NOISE_SMOOTH   0
#define NOISE_RANDOM   1
#define NOISE_RANDOM_2 2
#define NOISE_CLUMPY   3

// Vertex attributes in
layout(location = 6) in int oe_veg_texArenaIndex; // texture handle LUT index
layout(location = 7) in int oe_veg_nmlArenaIndex; // normal map LUT index

// Stage globals
out vec3 oe_UpVectorView;
vec4 vp_Color;
vec3 vp_Normal;
out vec4 oe_layer_tilec;

out vec3 oe_veg_texCoord; // Output tx coords
out mat3 oe_veg_TBN; // ref frame for normal maps
out float oe_veg_transition; // fade bb to model
out float oe_veg_distance;

uniform float oe_veg_maxRange;
uniform vec3 oe_Camera;
uniform mat4 osg_ViewMatrix;

#pragma import_defines(OE_WIND_TEX, OE_WIND_TEX_MATRIX)
#ifdef OE_WIND_TEX
uniform sampler3D OE_WIND_TEX;
uniform mat4 OE_WIND_TEX_MATRIX;
#endif

float rescale(float d, float v0, float v1)
{
    return clamp((d-v0)/(v1-v0), 0, 1);
}

uniform float osg_FrameTime;

// remap x from [0..1] to [lo..hi]
float remap(float x, float lo, float hi) {
    return lo + x * (hi - lo);
}
float unit(float x, float lo, float hi) {
    return clamp((x - lo) / (hi - lo), 0.0, 1.0);
}

flat out uint64_t oe_veg_texHandle;
flat out uint64_t oe_veg_nmlHandle;

void oe_GroundCover_Billboard(inout vec4 vertex_view, in uint i)
{
    vp_Color = vec4(1,1,1,0); // start alpha at ZERO for billboard transitions

    oe_layer_tilec = vec4(instance[i].tilec, 0, 1);
    vertex_view = oe_vertex.view;
    oe_UpVectorView = oe_transform.normal * vec3(0,0,1);

    vec4 noise = textureLod(oe_veg_noiseTex, oe_layer_tilec.st, 0);  

    // Calculate the normalized camera range (oe_Camera.z = LOD Scale)
    float maxRange = oe_veg_maxRange / oe_Camera.z;
    float nRange = clamp(-vertex_view.z/maxRange, 0.0, 1.0);

    // push the falloff closer to the max distance.
    float falloff = 1.0-(nRange*nRange*nRange);
    float width = instance[i].width * falloff;
    float height = instance[i].height * falloff;

    oe_veg_distance = 1.0 - falloff;

    int which = gl_VertexID & 7; // mod8 - there are 8 verts per instance

    // apply color darkening by distance
    //vp_Color.rgb *= clamp(1.0 - nRange, 0.5, 1.0);

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

    vec3 halfWidthTangentVector = tangentVector * 0.5 * width;

    vertex_view.xyz =
        which==0? vertex_view.xyz - halfWidthTangentVector :
        which==1? vertex_view.xyz + halfWidthTangentVector :
        which==2? vertex_view.xyz - halfWidthTangentVector + heightVector :
        vertex_view.xyz + halfWidthTangentVector + heightVector;

    oe_veg_texHandle = oe_veg_texArenaIndex >= 0 ? texArena[oe_veg_texArenaIndex] : 0UL;
    oe_veg_nmlHandle = 0UL; // no normal map for shadows

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

    if (which < 4 && billboardAmount > 0.0) // Front-facing billboard
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

            // normal mapping ref frame
            oe_veg_TBN = mat3(
                tangentVector,
                -normalize(cross(tangentVector, faceNormalVector)),
                faceNormalVector);

            // up frame prob works better.
            //oe_veg_TBN = mat3(
            //    tangentVector,
            //    -faceNormalVector,
            //    oe_UpVectorView);
        }
    }

    else if (which >= 4 && topDownAmount > 0.0) // top-down billboard
    {
        // estiblish the local tangent plane:
        vec3 Z = mat3(osg_ViewMatrix) * vec3(0,0,1); //north pole
        vec3 E = cross(Z, oe_UpVectorView);
        vec3 N = cross(oe_UpVectorView, E);
        Z = cross(E, N);

        // now introduce a "random" rotation
        vec2 b = vec2(instance[i].sinrot, instance[i].cosrot);
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
        oe_veg_TBN = mat3(E, N, oe_UpVectorView);

        vp_Color.a = topDownAmount;
    }

    oe_veg_texHandle = oe_veg_texArenaIndex >= 0 ? texArena[oe_veg_texArenaIndex] : 0UL;
    oe_veg_nmlHandle = oe_veg_nmlArenaIndex >= 0 ? texArena[oe_veg_nmlArenaIndex] : 0UL;

#endif // !OE_IS_SHADOW_CAMERA

    oe_veg_texCoord.st =
        which == 0 || which == 4? vec2(0, 0) :
        which == 1 || which == 5? vec2(1, 0) :
        which == 2 || which == 6? vec2(0, 1) :
                                  vec2(1, 1);

    // apply fade from bb->model
    if (instance[i].modelCommand >= 0)
    {
        oe_veg_transition = clamp(
            (1.0 + PSR_BUFFER - instance[i].pixelSizeRatio) / PSR_BUFFER, 
            0, 1);
    }
}

//uniform float demo_wind;
void oe_veg_apply_wind(inout vec4 vert_view, in float width, in float height)
{
#ifdef OE_WIND_TEX
    // sample the local wind map.
    const float stiffness = 3.8; // todo: tree parameter

    float xy_len = length(oe_vertex.local.xy) + oe_vertex.local.z*0.2;
    float bendDistance = xy_len;
    float xy_comp = unit(xy_len, 0, width);
    float stiffness_factor = pow(xy_comp, stiffness);
    bendDistance *= stiffness_factor;

    vec4 windData = textureProj(OE_WIND_TEX, (OE_WIND_TEX_MATRIX * vert_view));
    vec3 windDir = (windData.rgb * 2 - 1);

    const float rate = 0.01;
    vec4 noise_moving = textureLod(oe_veg_noiseTex, oe_layer_tilec.st + osg_FrameTime * rate, 0);
    float windSpeedVariation = remap(noise_moving[NOISE_CLUMPY], -0.2, 8.0-stiffness);
    float windSpeed = windData.a * windSpeedVariation;

    // wind turbulence - once the wind exceeds a certain speed, grass starts buffeting
    // based on a higher frequency noise function
    vec3 buffetingDir = vec3(0);
    if (windSpeed > 0.1 && xy_comp > 0.6)
    {
        float buffetingSpeed = windSpeed * 0.2 * stiffness_factor; //  xy_comp * xy_comp * xy_comp;
        vec4 noise_b = textureLod(oe_veg_noiseTex, oe_layer_tilec.st + osg_FrameTime * buffetingSpeed, 0);
        buffetingDir = vec3(0, noise_b.x * 2 - 1, 0) * buffetingSpeed; // vec3(noise_b.xx * 2 - 1, 0)
    }

    vec3 bendVec = (windDir + buffetingDir) * windSpeed * bendDistance;

    vert_view.xyz += bendVec;
#endif
}

void oe_GroundCover_Model(inout vec4 vertex_view, in uint i)
{
    oe_layer_tilec = vec4(instance[i].tilec, 0, 1);

    vertex_view = oe_vertex.view;

    vp_Normal = oe_vertex.normal;

    oe_UpVectorView = oe_transform.normal * vec3(0, 0, 1);

    oe_veg_texCoord = gl_MultiTexCoord7.xyz; // 7 why not

    // samplers for this model. The LUT index is in a vert attrib. 0UL=empty
    oe_veg_texHandle = oe_veg_texArenaIndex >= 0 ? texArena[oe_veg_texArenaIndex] : 0UL;
    oe_veg_nmlHandle = oe_veg_nmlArenaIndex >= 0 ? texArena[oe_veg_nmlArenaIndex] : 0UL;

    if (oe_veg_nmlHandle != 0UL)
    {
        vec3 E = cross(vp_Normal, oe_UpVectorView);
        vec3 N = cross(vp_Normal, E);
        oe_veg_TBN = mat3(normalize(E), normalize(N), vp_Normal);
    }

    // apply fade from bb->model
    if (instance[i].pixelSizeRatio < 1.0)
    {
        oe_veg_transition = clamp( 
            (instance[i].pixelSizeRatio - (1.0-PSR_BUFFER))/PSR_BUFFER, 
            0.0, 1.0 );
    }

    oe_veg_distance = 0.0;

#ifdef OE_WIND_TEX
    oe_veg_apply_wind(vertex_view, instance[i].width, instance[i].height);
#endif
}

// MAIN ENTRY POINT  
void oe_GroundCover_VS(inout vec4 vertex_view)
{
    oe_veg_transition = 1.0;

    RenderLeaf leaf = renderSet[gl_InstanceID + cmd[gl_DrawID].baseInstance];

    if (leaf.drawMask == 0x01)
    {
        oe_GroundCover_Billboard(vertex_view, leaf.instance);
    }
    else //if (leaf.drawMask == 0x02)
    {
        oe_GroundCover_Model(vertex_view, leaf.instance);
    }
}

[break]
#version 430
#extension GL_ARB_gpu_shader_int64 : enable

#pragma vp_name       Land cover billboard texture application
#pragma vp_entryPoint oe_GroundCover_FS
#pragma vp_location   fragment_coloring

#pragma import_defines(OE_IS_SHADOW_CAMERA)
#pragma import_defines(OE_IS_MULTISAMPLE)

uniform float oe_veg_maxAlpha;

in vec3 oe_veg_texCoord;
vec3 vp_Normal;
in float oe_roughness;
flat in uint64_t oe_veg_texHandle;
flat in uint64_t oe_veg_nmlHandle;
in mat3 oe_veg_TBN;
in float elev;

in float oe_veg_transition;
in float oe_veg_distance;
in vec3 oe_UpVectorView;

float harden(in float x)
{
    return 1.0 - (1.0 - x)*(1.0 - x);
}

float mapToNormalizedRange(in float value, in float lo, in float hi)
{
    return clamp((value - lo) / (hi - lo), 0.0, 1.0);
}

uniform float oe_snow = 0.0;

void oe_GroundCover_FS(inout vec4 color)
{
    // apply the transition fade
    color.a *= oe_veg_transition;

    if (oe_veg_texHandle > 0UL)
    {
        // modulate the texture.
        // "cast" the bindless handle to a sampler array and sample it
        color *= texture(sampler2D(oe_veg_texHandle), oe_veg_texCoord.st);

#ifndef OE_IS_SHADOW_CAMERA
        if (oe_veg_nmlHandle > 0UL)
        {
            vec4 n = texture(sampler2D(oe_veg_nmlHandle), oe_veg_texCoord.st);
            n.xyz = n.xyz*2.0-1.0;
            n.z = 1.0 - abs(n.x) - abs(n.y);
            float t = clamp(-n.z, 0, 1);
            n.x += (n.x > 0)? -t : t;
            n.y += (n.y > 0)? -t : t;
            vp_Normal = normalize(oe_veg_TBN * n.xyz);
            //color.rgb = (vp_Normal + 1.0)*0.5; // debug

        }
#endif
    }

#ifdef OE_IS_SHADOW_CAMERA
    if (color.a < oe_veg_maxAlpha)
    {
        discard;
    }
#else
    #ifdef OE_IS_MULTISAMPLE
        // mitigate the screen-door effect of A2C in the distance
        // https://tinyurl.com/y7bbbpl9
        float a = (color.a - oe_veg_maxAlpha) / max(fwidth(color.a), 0.0001) + 0.5;
        color.a = mix(color.a, a, oe_veg_distance);
    #else
        if (color.a < oe_veg_maxAlpha)
        {
            discard;
        }
    #endif

#if 0
    // TODO: revisit once we can figure out how to get terrain elevation
    float coldness = mapToNormalizedRange(elev, 1000, 3500);
    float cos_angle = clamp(dot(vp_Normal, normalize(oe_UpVectorView)), 0, 1);
    if (cos_angle > 0.3) {
        float snowiness = step(1.0 - oe_snow*coldness, cos_angle);
        color.rgb = mix(color.rgb, vec3(1), snowiness);
        oe_roughness = mix(oe_roughness, 0.1, snowiness);
    }
#endif

#endif // OE_LIGHTING
}

