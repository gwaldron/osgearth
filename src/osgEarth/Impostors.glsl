#version 330
#pragma vp_name Impostor Vertex Shader
#pragma vp_entryPoint oe_Impostor_VS
#pragma vp_location vertex_view

#pragma import_defines(IMPOSTOR_DEBUG)

uniform float oe_impostorRadius;

// stage globals
vec3 vp_Normal;
mat4 oe_instanceModelMatrix;    // from Instancing.glsl
mat3 oe_instanceNormalMatrix;   // from Instancing.glsl

// send to frag shader:
out vec2 oe_impostorTC;
out mat3 oe_normalMatrix;
out vec2 oe_impostorBlend;

#ifdef IMPOSTOR_DEBUG
out vec2 rawTC;
#endif

#define USE_INSTANCING

// given the octahedral vector, create a local coordinate system
// in which to create the billboard plane. This returns the offset
// vector for one of the billboard corners, whichever one is
// represented by offset.
// vector points from camera to object; "uv" is [-1..1]
// assume octahedralVector is normalized
vec3 projectOffset(in vec3 octahedralVector, in vec2 uv, in float halfSize)
{
    // Derive the reference frame of the original viewer that was
    // used to capture the impostor: FRU=forward/right/up vectors.
    vec3 F = -octahedralVector;
    vec3 R = normalize(cross(F, vec3(0,0,1)));
    vec3 U = normalize(cross(R, F));
    vec3 newX = R * uv.x * halfSize;
    vec3 newZ = U * uv.y * halfSize;
    return newX + newZ;
}

// transform from 2D frame-grid coordinates to a 3D octahedral vector
// Assume normalized input on +Z hemisphere. Output is [0..1]
vec2 vec3_to_hemioct(in vec3 v)
{
    // Project the hemisphere onto the hemi-octahedron, and then into the xy plane
    vec2 p = v.xy * (1.0 / (abs(v.x) + abs(v.y) + v.z));
    // Rotate and scale the center diamond to the unit square
    return (vec2(p.x + p.y, p.x - p.y)+1.0)*0.5; // [0..1]
    //"    return vec2(p.x + p.y, p.x - p.y); // [-1..1]
}

// transform from 3D octahedral vector to a 3D frame-grid coordinate
// Input is [0..1]
vec3 hemioct_to_vec3(in vec2 t)
{
    vec2 e = t*2.0-1.0;
    // Rotate and scale the unit square back to the center diamond
    vec2 temp = vec2(e.x + e.y, e.x - e.y) * 0.5;
    vec3 v = vec3(temp, 1.0 - abs(temp.x) - abs(temp.y));
    return normalize(v);
}

void oe_Impostor_VS(inout vec4 vertexView)
{
    // number of frames in each dimension - make this a define or something
    const float numFrames = 16; //24; //12.0;

    // calcuate the billboard corner offset vector for the current vertex:
    vec2 offset;
    int seq = gl_VertexID & 0x03; // mod(4)
    if      (seq == 0) offset = vec2(-1,-1);
    else if (seq == 1) offset = vec2( 1,-1);
    else if (seq == 2) offset = vec2(-1, 1);
    else               offset = vec2( 1, 1);

    // get grid vector is the vector from the model to the camera
    // that will get mapped to 2D octahedral space. I tried to do 
    // this in view space but it didn't work - need to revisit that.
    // Don't like the inverse. Can we ditch it?
#ifdef USE_INSTANCING
    mat4 instanceToView = gl_ModelViewMatrix * oe_instanceModelMatrix;
    mat4 viewToInstance = inverse(instanceToView);
    oe_normalMatrix = gl_NormalMatrix * oe_instanceNormalMatrix;
#else
    mat4 viewToInstance = inverse(gl_ModelViewMatrix);
    oe_normalMatrix = gl_NormalMatrix;
#endif

    // TODO: bring pivotOffset in through a uniform, and apply it properly
    // throughout the shader.    
    vec3 pivotOffset = vec3(0, 0, oe_impostorRadius);
    vec4 cameraInInstanceFrame = viewToInstance * vec4(0,0,0,1);
    vec3 octahedralVector = normalize(cameraInInstanceFrame.xyz - pivotOffset);
    if (octahedralVector.z < 0.01)
    {
        octahedralVector.z = 0.01;
        octahedralVector = normalize(octahedralVector);
    }
    
    
    // transform to 2D frame-grid space
    vec2 octa = vec3_to_hemioct(octahedralVector);

    // quantize to the nearest octahedral vertex:
    float numFramesMinusOne = numFrames-1.0;
    vec2 octaQuantized = floor(octa*numFramesMinusOne)/ numFramesMinusOne;

    // calculate 1/2 the size of one frame:
    float t = (0.5/numFrames);

    // blending factor for adjacent frames (TBD)
    //oe_impostorBlend = (octa - octaQuantized) / t;

    // derive the texture coordinate at the center of the frame:
    vec2 tc = octaQuantized *numFramesMinusOne/numFrames + t;

    // establish the texture coordinates:
    oe_impostorTC = tc +(offset*t);

    // use the octahedral vector to establish a reference frame for
    // creating the billboard in model space, then xform that into
    // view space and create the billboard by offsetting the vertices:
    vec3 octahedralVectorQuantized = hemioct_to_vec3(octaQuantized);

#if 1
    vec3 vertexOffset = projectOffset(octahedralVectorQuantized, offset, oe_impostorRadius) + pivotOffset;
#else
    vec3 projectedOffset = projectOffset(quantizedVector, offset, halfSize); // offset projected into object space
    vec3 vertexOffset = projectedOffset + pivotOffset;
    vertexOffset = normalize(cameraInInstanceFrame.xyz-vertexOffset);
    vertexOffset += projectedOffset;
    //vertexOffset -= gl_Vertex.xyz; // impostor vertex is always 0,0,0 anyway
    vertexOffset += pivotOffset;
#endif

#ifdef USE_INSTANCING
    mat3 rotateInstanceToView = mat3(instanceToView);
    vertexOffset = rotateInstanceToView * vertexOffset;    
#else
    vertexOffset = gl_NormalMatrix*vertexOffset;
#endif

    // deal with top-down view causing billboards to z-fight with terrain
    //if (dot(normalize(-vertexView.xyz), vp_Normal) > 0.99)
    //    vertexOffset.z += 0.5;

    vertexView.xyz += vertexOffset;

    // encode the normal matrix so we can sample the normal map
    // in the fragment shader.
#ifdef USE_INSTANCING
    oe_normalMatrix = gl_NormalMatrix * oe_instanceNormalMatrix;
#else
    oe_normalMatrix = gl_NormalMatrix;
#endif

#ifdef IMPOSTOR_DEBUG
    rawTC = (offset+1.0)*0.5;
#endif
}



//................................................................
[break]

#version 330
#pragma vp_entryPoint oe_Impostor_FS
#pragma vp_location fragment_coloring

#pragma import_defines(IMPOSTOR_DEBUG)

uniform sampler2D impostorTex;
uniform sampler2D impostorNormalMap;

in vec2 oe_impostorTC;
in mat3 oe_normalMatrix;
vec3 vp_Normal;

#ifdef IMPOSTOR_DEBUG
in vec2 rawTC;
#endif

void oe_Impostor_FS(inout vec4 color)
{    
    vec4 texel = texture(impostorTex, oe_impostorTC);
    color = texel;

#ifdef IMPOSTOR_DEBUG
    // debugging - draw the billboard geometry, red on top
    if (any(greaterThan(abs(rawTC*2.0-1.0), vec2(.99)))) color=vec4(1);
    if (rawTC.t > .99) color=vec4(1,0,0,1);
#endif

    if (color.a < 0.15)
        discard;

    // sample the normal map and transform the normal into view space.
    // todo: support octahedral compressed normals
    vec3 encodedNormal = texture(impostorNormalMap, oe_impostorTC).xyz;
    vec3 normal = normalize(encodedNormal.xyz*2.0-1.0);
    vp_Normal = oe_normalMatrix * normal;

    // debug the normal map
    //color.rgb = encodedNormal;
}
